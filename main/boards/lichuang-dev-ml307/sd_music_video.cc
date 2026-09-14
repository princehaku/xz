#include "sd_music_player.h"

#include "audio_service.h"
#include "avi_reader.h"
#include "board.h"
#include "display/lvgl_display/jpg/jpeg_to_image.h"
#include "sd_video_url.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <memory>

#include <esp_ae_rate_cvt.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr const char* kVideoTag = "SdVideo";
constexpr const char* kVideoDirectory = "/sdcard/video";
constexpr const char* kVideoPart = "/sdcard/video/test.part";
constexpr const char* kVideoFile = "/sdcard/video/test.avi";
constexpr const char* kVideoBackup = "/sdcard/video/test-backup.avi";
constexpr size_t kMaxDownloadBytes = 16 * 1024 * 1024;
constexpr size_t kDownloadBlockBytes = 4096;
constexpr int kVideoHttpConnection = 4;

struct VideoFileCloser {
    void operator()(FILE* file) const { fclose(file); }
};

struct PartialVideo {
    FILE* file = nullptr;
    bool created = false;
    ~PartialVideo() {
        if (file)
            fclose(file);
        if (created)
            remove(kVideoPart);
    }
};

// Separate cursors let the same SD worker keep audio queued while a video frame
// waits for its timestamp. No second SD task or whole-track PCM buffer is needed.
class VideoAudioCursor {
public:
    ~VideoAudioCursor() {
        if (converter_)
            esp_ae_rate_cvt_close(converter_);
    }
    bool Open(const char* path, uint32_t output_rate) {
        file_.reset(fopen(path, "rb"));
        if (!file_ || !reader_.Open(file_.get()) || !reader_.info().audio_supported)
            return false;
        info_ = reader_.info();
        block_samples_ = output_rate / 50;
        if (info_.audio_sample_rate != output_rate) {
            esp_ae_rate_cvt_cfg_t config{};
            config.src_rate = info_.audio_sample_rate;
            config.dest_rate = output_rate;
            config.channel = 1;
            config.bits_per_sample = 16;
            config.complexity = 2;
            config.perf_type = ESP_AE_RATE_CVT_PERF_TYPE_SPEED;
            if (esp_ae_rate_cvt_open(&config, &converter_) != ESP_AE_ERR_OK)
                return false;
        }
        return true;
    }
    // Produces at most 20 ms; empty output means a validated end of stream.
    bool Next(std::vector<int16_t>& pcm) {
        pcm.clear();
        while (pcm.size() < block_samples_) {
            if (converted_position_ < converted_.size()) {
                const size_t count =
                    std::min(block_samples_ - pcm.size(), converted_.size() - converted_position_);
                pcm.insert(pcm.end(), converted_.data() + converted_position_,
                           converted_.data() + converted_position_ + count);
                converted_position_ += count;
                continue;
            }
            if (position_ == chunk_.size()) {
                if (ended_)
                    break;
                const auto result = reader_.NextAudio(chunk_);
                position_ = 0;
                if (result == AviReader::Result::kEnd) {
                    ended_ = true;
                    break;
                }
                if (result != AviReader::Result::kAudio)
                    return false;
            }
            const size_t bytes_per_sample = info_.audio_channels * 2;
            const size_t count = std::min<size_t>((chunk_.size() - position_) / bytes_per_sample,
                                                  info_.audio_sample_rate / 50);
            if (!count)
                return false;
            mono_.resize(count);
            auto read_sample = [](const uint8_t* p) {
                return static_cast<int16_t>(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
            };
            for (size_t i = 0; i < count; ++i) {
                const auto* sample = chunk_.data() + position_ + i * bytes_per_sample;
                const int32_t left = read_sample(sample);
                mono_[i] = info_.audio_channels == 1 ? left : (left + read_sample(sample + 2)) / 2;
            }
            position_ += count * bytes_per_sample;
            converted_position_ = 0;
            if (converter_) {
                uint32_t capacity = 0;
                if (esp_ae_rate_cvt_get_max_out_sample_num(converter_, count, &capacity) !=
                        ESP_AE_ERR_OK ||
                    capacity > 4096)
                    return false;
                converted_.resize(std::max<uint32_t>(capacity, 1));
                if (esp_ae_rate_cvt_process(converter_, mono_.data(), count, converted_.data(),
                                            &capacity) != ESP_AE_ERR_OK ||
                    capacity > converted_.size())
                    return false;
                converted_.resize(capacity);
            } else {
                converted_.assign(mono_.begin(), mono_.end());
            }
        }
        return true;
    }

private:
    std::unique_ptr<FILE, VideoFileCloser> file_;
    AviReader reader_;
    AviReader::Info info_;
    esp_ae_rate_cvt_handle_t converter_ = nullptr;
    std::vector<uint8_t> chunk_;
    std::vector<int16_t> mono_, converted_;
    size_t position_ = 0, converted_position_ = 0, block_samples_ = 0;
    bool ended_ = false;
};

std::shared_ptr<SdMusicPlayer::VideoFrame> DecodeVideoFrame(const std::vector<uint8_t>& jpeg,
                                                            const AviReader::Info& info) {
    uint8_t* output = nullptr;
    size_t bytes = 0, width = 0, height = 0, stride = 0;
    const auto error =
        jpeg_to_image(jpeg.data(), jpeg.size(), &output, &bytes, &width, &height, &stride);
    std::unique_ptr<uint8_t, decltype(&heap_caps_free)> owned(output, heap_caps_free);
    if (error != ESP_OK || !output || width != info.width || height != info.height || width == 0 ||
        width > 320 || height == 0 || height > 240 || stride < width * 2 || stride > 640 ||
        bytes < stride * height || bytes > 320 * 240 * 2)
        return {};
    auto frame = std::make_shared<SdMusicPlayer::VideoFrame>();
    frame->width = width;
    frame->height = height;
    frame->stride = stride;
    frame->pixels.assign(output, output + stride * height);
    return frame;
}
}  // namespace

std::shared_ptr<const SdMusicPlayer::VideoFrame> SdMusicPlayer::GetVideoFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return video_frame_;
}

bool SdMusicPlayer::DownloadVideo(const std::string& url) {
    if (!IsSdVideoUrlValid(url))
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || !EnsureWorkerLocked())
        return false;
    running_ = true;
    RestartLocked(true);
    download_url_ = url;
    snapshot_.state = State::kDownloading;
    snapshot_.title = "AVI test";
    snapshot_.is_video = true;
    snapshot_.message = "Downloading AVI...";
    return true;
}

bool SdMusicPlayer::DownloadVideoFile(const Command& command) {
    if (!IsCurrent(command.generation))
        return false;
    SetState(command.generation, State::kDownloading, "Downloading AVI...");
    auto fail = [&](const char* message) {
        ESP_LOGE(kVideoTag, "%s", message);
        SetState(command.generation, State::kError, message);
        return false;
    };
    if (mkdir(kVideoDirectory, 0755) != 0 && errno != EEXIST)
        return fail("Cannot create video folder");
    struct stat info{};
    if (stat(kVideoDirectory, &info) != 0 || !S_ISDIR(info.st_mode))
        return fail("Invalid video folder");

    // Reserve only our staging file. A pre-existing file is never truncated.
    const int fd = open(kVideoPart, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd < 0)
        return fail("Cannot create test.part");
    PartialVideo partial;
    partial.created = true;
    partial.file = fdopen(fd, "wb");
    if (!partial.file) {
        close(fd);
        return fail("Cannot open download file");
    }
    auto* network = Board::GetInstance().GetNetwork();
    if (!network)
        return fail("Network unavailable");
    auto http = network->CreateHttp(kVideoHttpConnection);
    if (!http)
        return fail("HTTP unavailable");
    http->SetTimeout(5000);
    http->SetKeepAlive(false);
    http->SetHeader("Accept-Encoding", "identity");
    if (!http->Open("GET", command.download_url))
        return fail("Download connection failed");
    if (!IsCurrent(command.generation))
        return false;
    if (http->GetStatusCode() != 200)
        return fail("Download requires HTTP 200");
    const size_t expected = http->GetBodyLength();
    if (expected < 12 || expected > kMaxDownloadBytes)
        return fail("AVI requires Content-Length, max 16 MiB");
    std::array<char, kDownloadBlockBytes> buffer{};
    size_t written = 0;
    while (IsCurrent(command.generation)) {
        const int count = http->Read(buffer.data(), buffer.size());
        if (!IsCurrent(command.generation))
            return false;
        if (count < 0)
            return fail("Download read failed");
        if (count == 0)
            break;
        if (static_cast<size_t>(count) > buffer.size() ||
            static_cast<size_t>(count) > expected - written) {
            return fail("Download exceeds advertised size");
        }
        if (fwrite(buffer.data(), 1, count, partial.file) != static_cast<size_t>(count)) {
            return fail("SD write failed");
        }
        written += count;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!IsCurrent(command.generation))
                return false;
            snapshot_.download_percent = static_cast<int>(written * 100 / expected);
        }
        // Keep Wi-Fi, UI and watchdog tasks responsive during fast local downloads.
        vTaskDelay(1);
    }
    if (!IsCurrent(command.generation))
        return false;
    http->Close();
    if (written != expected)
        return fail("Download truncated");
    if (fflush(partial.file) != 0 || ferror(partial.file) || fsync(fileno(partial.file)) != 0) {
        return fail("SD flush failed");
    }
    FILE* completed = partial.file;
    partial.file = nullptr;
    if (fclose(completed) != 0)
        return fail("SD close failed");

    SetState(command.generation, State::kDownloading, "Checking AVI...");
    {
        std::unique_ptr<FILE, VideoFileCloser> file(fopen(kVideoPart, "rb"));
        AviReader reader;
        std::vector<uint8_t> jpeg;
        if (!file || !reader.Open(file.get()))
            return fail("Unsupported or damaged MJPEG AVI");
        uint32_t checked = 0;
        while (IsCurrent(command.generation)) {
            const auto result = reader.NextFrame(jpeg);
            if (!IsCurrent(command.generation))
                return false;
            if (result == AviReader::Result::kEnd)
                break;
            if (result != AviReader::Result::kFrame ||
                (checked == 0 && !DecodeVideoFrame(jpeg, reader.info()))) {
                return fail("Unsupported or damaged MJPEG AVI");
            }
            ++checked;
            vTaskDelay(1);
        }
        if (!IsCurrent(command.generation))
            return false;
        if (!checked || checked != reader.info().frame_count)
            return fail("AVI frame count mismatch");
        if (reader.info().has_audio) {
            if (!reader.info().audio_supported)
                return fail("AVI audio needs PCM16 mono/stereo, 8-48 kHz");
            // Reopen the cursor on this worker; validate audio before replacing
            // the old file as well as before passing samples to the codec.
            if (!reader.Open(file.get()))
                return fail("Cannot read AVI audio");
            while (IsCurrent(command.generation)) {
                const auto result = reader.NextAudio(jpeg);
                if (result == AviReader::Result::kEnd)
                    break;
                if (result != AviReader::Result::kAudio)
                    return fail("Damaged AVI PCM audio");
                vTaskDelay(1);
            }
            if (!IsCurrent(command.generation))
                return false;
        }
    }

    // FAT rename does not replace existing files. Keep the previous test until
    // the validated replacement is committed, and roll it back on rename failure.
    // The short commit is serialized with Stop/Next/Rescan generation changes.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!IsCurrent(command.generation))
            return false;
        const bool exists = stat(kVideoFile, &info) == 0;
        if (exists && !S_ISREG(info.st_mode)) {
            snapshot_.state = State::kError;
            snapshot_.message = "test.avi is not a file";
            return false;
        }
        if (exists && (stat(kVideoBackup, &info) == 0 || errno != ENOENT)) {
            snapshot_.state = State::kError;
            snapshot_.message = "Video backup already exists";
            return false;
        }
        if (exists && rename(kVideoFile, kVideoBackup) != 0) {
            snapshot_.state = State::kError;
            snapshot_.message = "Cannot preserve previous AVI";
            return false;
        }
        if (rename(kVideoPart, kVideoFile) != 0) {
            const bool restored = !exists || rename(kVideoBackup, kVideoFile) == 0;
            snapshot_.state = State::kError;
            snapshot_.message =
                restored ? "Cannot save downloaded AVI" : "Old AVI retained in test-backup.avi";
            return false;
        }
        partial.created = false;
        if (exists && remove(kVideoBackup) != 0) {
            ESP_LOGE(kVideoTag, "Previous AVI retained in test-backup.avi");
        }
        snapshot_.download_percent = 100;
    }
    ESP_LOGI(kVideoTag, "Saved verified AVI: %u bytes", static_cast<unsigned>(written));
    return true;
}

SdMusicPlayer::TrackResult SdMusicPlayer::PlayVideoTrack(const Command& command) {
    if (!IsCurrent(command.generation))
        return TrackResult::kCancelled;
    std::unique_ptr<FILE, VideoFileCloser> file(fopen(tracks_[command.index].c_str(), "rb"));
    AviReader reader;
    if (!file || !reader.Open(file.get()))
        return TrackResult::kBadFile;
    const auto info = reader.info();
    if (!info.frame_interval_us)
        return TrackResult::kBadFile;
    if (info.has_audio && !info.audio_supported) {
        SetState(command.generation, State::kError, "AVI audio needs PCM16 mono/stereo, 8-48 kHz");
        return TrackResult::kCancelled;
    }
    const auto output_rate = Board::GetInstance().GetAudioCodec()->output_sample_rate();
    VideoAudioCursor audio_cursor;
    if (info.has_audio && (output_rate < 8000 || output_rate > 48000 ||
                           !audio_cursor.Open(tracks_[command.index].c_str(), output_rate)))
        return TrackResult::kAudioError;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!IsCurrent(command.generation))
            return TrackResult::kCancelled;
        auto title = tracks_[command.index].substr(tracks_[command.index].find_last_of('/') + 1);
        const auto dot = title.rfind('.');
        if (dot != std::string::npos)
            title.resize(dot);
        snapshot_.index = command.index;
        snapshot_.title = title;
        snapshot_.is_video = true;
        snapshot_.elapsed_seconds = 0;
        snapshot_.video_frames = 0;
        snapshot_.video_has_audio = info.has_audio;
        snapshot_.video_audio_samples = 0;
        snapshot_.state = paused_ ? State::kPaused : State::kPlaying;
        snapshot_.message =
            paused_ ? "Paused" : (info.has_audio ? "Video + audio" : "Silent video");
    }
    ESP_LOGI(kVideoTag, "MJPEG AVI %ux%u, interval %u us, PCM %u Hz / %u channels, volume %u",
             static_cast<unsigned>(info.width), static_cast<unsigned>(info.height),
             static_cast<unsigned>(info.frame_interval_us),
             static_cast<unsigned>(info.audio_sample_rate),
             static_cast<unsigned>(info.audio_channels),
             static_cast<unsigned>(Board::GetInstance().GetAudioCodec()->output_volume()));
    std::vector<uint8_t> jpeg;
    std::vector<int16_t> pending_pcm;
    uint32_t frames = 0, dropped = 0;
    uint64_t submitted = 0;
    bool audio_end = !info.has_audio;
    const int64_t started = esp_timer_get_time();
    int64_t paused_time = 0;
    int64_t audio_tail_started = -1;
    auto wait_playing = [&]() {
        const int64_t before = esp_timer_get_time();
        if (!WaitUntilPlaying(command.generation))
            return false;
        const int64_t delay = esp_timer_get_time() - before;
        paused_time += delay;
        if (audio_tail_started >= 0)
            audio_tail_started += delay;
        return true;
    };
    auto feed_audio = [&]() -> TrackResult {
        // Keep six 20-ms blocks ahead of the writer so SD/JPEG work has room.
        // Bound each pass even if a fast consumer completes writes immediately.
        for (unsigned blocks = 0; !audio_end && blocks < 6; ++blocks) {
            if (!wait_playing())
                return TrackResult::kCancelled;
            const auto played = audio_.GetLocalPlaybackSamples(command.token);
            if (submitted >= played + uint64_t(output_rate) * 120 / 1000)
                break;
            if (pending_pcm.empty() && !audio_cursor.Next(pending_pcm))
                return TrackResult::kBadFile;
            if (pending_pcm.empty()) {
                audio_end = true;
                break;
            }
            const size_t count = pending_pcm.size();
            unsigned stalls = 0;
            while (!audio_.PushLocalPcm(command.token, pending_pcm)) {
                if (!wait_playing())
                    return TrackResult::kCancelled;
                if (++stalls >= 30)
                    return TrackResult::kAudioError;
            }
            submitted += count;
            pending_pcm.clear();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (IsCurrent(command.generation))
                snapshot_.video_audio_samples =
                    static_cast<uint32_t>(audio_.GetLocalPlaybackSamples(command.token));
        }
        return TrackResult::kComplete;
    };
    auto media_time = [&]() -> int64_t {
        if (!info.has_audio)
            return esp_timer_get_time() - started - paused_time;
        const auto played = audio_.GetLocalPlaybackSamples(command.token);
        const int64_t audio_us = played * 1000000 / output_rate;
        if (audio_end && played >= submitted) {
            if (audio_tail_started < 0)
                audio_tail_started = esp_timer_get_time();
            return audio_us + esp_timer_get_time() - audio_tail_started;
        }
        return audio_us;
    };
    auto wait_until = [&](int64_t target) -> TrackResult {
        while (wait_playing()) {
            const auto status = feed_audio();
            if (status != TrackResult::kComplete)
                return status;
            if (media_time() >= target)
                return TrackResult::kComplete;
            ulTaskNotifyTake(pdTRUE, std::max<uint32_t>(1, pdMS_TO_TICKS(5)));
        }
        return TrackResult::kCancelled;
    };
    while (IsCurrent(command.generation)) {
        if (!wait_playing())
            return TrackResult::kCancelled;
        const auto feed = feed_audio();
        if (feed != TrackResult::kComplete)
            return feed;
        const auto result = reader.NextFrame(jpeg);
        if (!IsCurrent(command.generation))
            return TrackResult::kCancelled;
        if (result == AviReader::Result::kEnd) {
            // Hold the last picture for its duration and finish any longer audio.
            const auto status = wait_until(uint64_t(frames) * info.frame_interval_us);
            if (status != TrackResult::kComplete)
                return status;
            while (wait_playing()) {
                const auto feed = feed_audio();
                if (feed != TrackResult::kComplete)
                    return feed;
                if (audio_end && (!info.has_audio || audio_.IsPlaybackComplete()))
                    break;
                ulTaskNotifyTake(pdTRUE, std::max<uint32_t>(1, pdMS_TO_TICKS(5)));
            }
            if (!IsCurrent(command.generation))
                return TrackResult::kCancelled;
            const int64_t elapsed =
                std::max<int64_t>(1, esp_timer_get_time() - started - paused_time);
            const auto fps_hundredths =
                static_cast<uint32_t>(uint64_t(frames) * 100000000 / elapsed);
            // ESP-IDF nano printf may omit long-long/float formatting support.
            ESP_LOGI(kVideoTag,
                     "AVI complete: %u frames, %u ms, %u.%02u fps, PCM %u samples, dropped %u",
                     static_cast<unsigned>(frames), static_cast<unsigned>(elapsed / 1000),
                     static_cast<unsigned>(fps_hundredths / 100),
                     static_cast<unsigned>(fps_hundredths % 100), static_cast<unsigned>(submitted),
                     static_cast<unsigned>(dropped));
            return frames ? TrackResult::kComplete : TrackResult::kBadFile;
        }
        if (result != AviReader::Result::kFrame)
            return TrackResult::kBadFile;
        const int64_t target = uint64_t(frames++) * info.frame_interval_us;
        // Under load, discard an expired picture rather than slowing the sound.
        if (info.has_audio && media_time() >= target + info.frame_interval_us) {
            ++dropped;
            continue;
        }
        auto frame = DecodeVideoFrame(jpeg, info);
        if (!frame)
            return TrackResult::kBadFile;
        while (true) {
            const auto status = wait_until(target);
            if (status != TrackResult::kComplete)
                return status;
            std::lock_guard<std::mutex> lock(mutex_);
            if (!IsCurrent(command.generation))
                return TrackResult::kCancelled;
            if (paused_)
                continue;
            video_frame_ = std::move(frame);
            snapshot_.video_frames = frames;
            snapshot_.elapsed_seconds =
                static_cast<uint32_t>(uint64_t(frames - 1) * info.frame_interval_us / 1000000);
            break;
        }
    }
    return TrackResult::kCancelled;
}

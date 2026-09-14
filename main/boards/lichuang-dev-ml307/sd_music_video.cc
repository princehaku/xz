#include "sd_music_player.h"

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
        snapshot_.state = paused_ ? State::kPaused : State::kPlaying;
        snapshot_.message = paused_ ? "Paused" : "Silent video";
    }
    ESP_LOGI(kVideoTag, "MJPEG AVI %ux%u, interval %u us, audio %s (silent playback)",
             static_cast<unsigned>(info.width), static_cast<unsigned>(info.height),
             static_cast<unsigned>(info.frame_interval_us), info.has_audio ? "present" : "absent");
    std::vector<uint8_t> jpeg;
    uint32_t frames = 0;
    const int64_t started = esp_timer_get_time();
    int64_t paused_time = 0;
    int64_t deadline = started;
    auto wait_for_frame = [&]() {
        while (IsCurrent(command.generation)) {
            bool paused;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                paused = paused_;
            }
            if (paused) {
                const int64_t before = esp_timer_get_time();
                if (!WaitUntilPlaying(command.generation))
                    return false;
                const int64_t delay = esp_timer_get_time() - before;
                paused_time += delay;
                deadline += delay;
            }
            const int64_t remaining = deadline - esp_timer_get_time();
            if (remaining <= 0)
                return true;
            const uint32_t ms =
                static_cast<uint32_t>(std::min<int64_t>((remaining + 999) / 1000, 20));
            ulTaskNotifyTake(pdTRUE, std::max<uint32_t>(1, pdMS_TO_TICKS(ms)));
        }
        return false;
    };
    while (IsCurrent(command.generation)) {
        if (!wait_for_frame())
            return TrackResult::kCancelled;
        const auto result = reader.NextFrame(jpeg);
        if (!IsCurrent(command.generation))
            return TrackResult::kCancelled;
        if (result == AviReader::Result::kEnd) {
            const int64_t elapsed =
                std::max<int64_t>(1, esp_timer_get_time() - started - paused_time);
            const auto fps_hundredths =
                static_cast<uint32_t>(uint64_t(frames) * 100000000 / elapsed);
            // ESP-IDF nano printf may omit long-long/float formatting support.
            ESP_LOGI(kVideoTag, "AVI complete: %u frames, %u ms, %u.%02u fps",
                     static_cast<unsigned>(frames), static_cast<unsigned>(elapsed / 1000),
                     static_cast<unsigned>(fps_hundredths / 100),
                     static_cast<unsigned>(fps_hundredths % 100));
            return frames ? TrackResult::kComplete : TrackResult::kBadFile;
        }
        if (result != AviReader::Result::kFrame)
            return TrackResult::kBadFile;
        auto frame = DecodeVideoFrame(jpeg, info);
        if (!frame)
            return TrackResult::kBadFile;
        while (true) {
            if (!wait_for_frame())
                return TrackResult::kCancelled;
            std::lock_guard<std::mutex> lock(mutex_);
            if (!IsCurrent(command.generation))
                return TrackResult::kCancelled;
            if (paused_)
                continue;
            video_frame_ = std::move(frame);
            snapshot_.video_frames = ++frames;
            snapshot_.elapsed_seconds =
                static_cast<uint32_t>(uint64_t(frames - 1) * info.frame_interval_us / 1000000);
            break;
        }
        deadline += info.frame_interval_us;
        // Slower cards/decoders show every frame without a burst of catch-up work.
        deadline = std::max<int64_t>(deadline, esp_timer_get_time());
    }
    return TrackResult::kCancelled;
}

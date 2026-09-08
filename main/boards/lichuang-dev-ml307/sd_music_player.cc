#include "sd_music_player.h"

#include "audio_service.h"
#include "board.h"
#include "config.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <functional>
#include <memory>
#include <limits>
#include <sys/stat.h>

#include <driver/sdmmc_host.h>
#include <esp_ae_rate_cvt.h>
#include <esp_audio_simple_dec.h>
#include <esp_log.h>
#include <esp_mp3_dec.h>
#include <esp_pcm_dec.h>
#include <esp_vfs_fat.h>
#include <impl/esp_wav_dec.h>
#include <sdmmc_cmd.h>

namespace {
constexpr const char* kTag = "SdMusic";
constexpr const char* kMountPoint = "/sdcard";
constexpr size_t kMaxTracks = 256;
constexpr size_t kMaxEntries = 4096;
constexpr unsigned kMaxDepth = 3;
constexpr size_t kReadBytes = 4096;
constexpr size_t kMaxInputBytes = 16384;
constexpr size_t kMaxOutputBytes = 16384;
constexpr size_t kMaxBytesBeforeAudio = 256 * 1024;

std::string Extension(const std::string& name) {
    auto dot = name.rfind('.');
    if (dot == std::string::npos) return {};
    auto extension = name.substr(dot);
    for (char& c : extension) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return extension;
}

std::string TrackTitle(const std::string& path) {
    auto title = path.substr(path.find_last_of('/') + 1);
    const auto dot = title.rfind('.');
    if (dot != std::string::npos) title.resize(dot);
    return title;
}

uint16_t ReadLe16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
uint32_t ReadLe32(const uint8_t* p) {
    return p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

bool SupportedRate(uint32_t rate) {
    return rate >= 8000 && rate <= 96000 && (rate % 4000 == 0 || rate % 11025 == 0);
}

bool SkipId3Tag(FILE* file) {
    std::array<uint8_t, 10> header{};
    const auto count = fread(header.data(), 1, header.size(), file);
    if (count < 3 || memcmp(header.data(), "ID3", 3) != 0) return fseek(file, 0, SEEK_SET) == 0;
    if (count != header.size() || header[3] < 2 || header[3] > 4 || header[4] == 0xff) return false;
    uint32_t size = 0;
    for (size_t i = 6; i < 10; ++i) {
        if (header[i] & 0x80) return false;
        size = (size << 7) | header[i];
    }
    const bool footer = header[3] == 4 && (header[5] & 0x10);
    const uint64_t offset = uint64_t(size) + 10 + (footer ? 10 : 0);
    struct stat info{};
    if (fstat(fileno(file), &info) != 0 || offset >= static_cast<uint64_t>(info.st_size) ||
        offset > std::numeric_limits<long>::max()) return false;
    if (footer) {
        std::array<uint8_t, 10> tail{};
        if (fseek(file, static_cast<long>(offset - 10), SEEK_SET) != 0 ||
            fread(tail.data(), 1, tail.size(), file) != tail.size() ||
            memcmp(tail.data(), "3DI", 3) || memcmp(tail.data() + 3, header.data() + 3, 7)) return false;
    }
    return fseek(file, static_cast<long>(offset), SEEK_SET) == 0;
}

// Reject compressed/float/malformed WAVs before handing them to the WAV parser.
// Chunk metadata is bounded; no audio payload is read during this validation.
bool ValidateWav(FILE* file, const std::function<bool()>& active) {
    std::array<uint8_t, 12> header{};
    if (fread(header.data(), 1, header.size(), file) != header.size() ||
        memcmp(header.data(), "RIFF", 4) || memcmp(header.data() + 8, "WAVE", 4)) return false;
    struct stat info{};
    if (fstat(fileno(file), &info) != 0 || info.st_size < 44) return false;
    const uint64_t limit = std::min<uint64_t>(uint64_t(ReadLe32(header.data() + 4)) + 8, info.st_size);
    uint64_t offset = 12;
    bool pcm = false;
    for (unsigned chunks = 0; chunks < 128 && offset + 8 <= limit && offset <= 65536; ++chunks) {
        if (!active()) return false;
        std::array<uint8_t, 8> chunk{};
        if (fread(chunk.data(), 1, chunk.size(), file) != chunk.size()) return false;
        const uint32_t size = ReadLe32(chunk.data() + 4);
        offset += 8;
        if (uint64_t(size) > limit - offset) return false;
        if (memcmp(chunk.data(), "fmt ", 4) == 0) {
            std::array<uint8_t, 16> format{};
            if (size < format.size() || fread(format.data(), 1, format.size(), file) != format.size()) return false;
            const auto channels = ReadLe16(format.data() + 2);
            const auto rate = ReadLe32(format.data() + 4);
            pcm = ReadLe16(format.data()) == 1 && (channels == 1 || channels == 2) &&
                ReadLe16(format.data() + 14) == 16 && ReadLe16(format.data() + 12) == channels * 2 &&
                SupportedRate(rate);
            if (!pcm) return false;
        } else if (memcmp(chunk.data(), "data", 4) == 0) {
            return pcm && size > 0 && fseek(file, 0, SEEK_SET) == 0;
        }
        offset += uint64_t(size) + (size & 1U);
        if (offset > limit || offset > 65536 || fseek(file, static_cast<long>(offset), SEEK_SET) != 0) return false;
    }
    return false;
}

bool RegisterDecoders() {
    static std::once_flag once;
    static bool available = false;
    std::call_once(once, [] {
        auto accepted = [](esp_audio_err_t error) {
            return error == ESP_AUDIO_ERR_OK || error == ESP_AUDIO_ERR_ALREADY_EXIST;
        };
        const bool mp3 = accepted(esp_mp3_dec_register());
        const bool pcm = accepted(esp_pcm_dec_register());
        const bool wav = accepted(esp_wav_dec_register());
        available = mp3 && pcm && wav;
    });
    return available;
}

struct CloseFile { void operator()(FILE* file) const { fclose(file); } };
struct CloseDirectory { void operator()(DIR* directory) const { closedir(directory); } };

struct TrackResources {
    std::unique_ptr<FILE, CloseFile> file;
    esp_audio_simple_dec_handle_t decoder = nullptr;
    esp_ae_rate_cvt_handle_t converter = nullptr;
    ~TrackResources() {
        if (converter) esp_ae_rate_cvt_close(converter);
        if (decoder) esp_audio_simple_dec_close(decoder);
    }
};
}  // namespace

SdMusicPlayer::SdMusicPlayer(AudioService& audio) : audio_(audio) {}

SdMusicPlayer::~SdMusicPlayer() {
    Stop();
    std::unique_lock<std::mutex> lock(mutex_);
    shutdown_ = true;
    if (worker_) xTaskNotifyGive(worker_);
    worker_done_.wait(lock, [this] { return worker_ == nullptr; });
}

bool SdMusicPlayer::EnsureWorkerLocked() {
    if (worker_) return true;
    if (xTaskCreate([](void* context) {
            static_cast<SdMusicPlayer*>(context)->Worker();
        }, "sd_music", 8192, this, 3, &worker_) != pdPASS) {
        worker_ = nullptr;
        snapshot_.state = State::kError;
        snapshot_.message = "Player unavailable";
        return false;
    }
    return true;
}

void SdMusicPlayer::RestartLocked(bool scan) {
    ++generation_;
    if (token_) audio_.EndLocalPlayback(token_);
    token_ = audio_.BeginLocalPlayback();
    paused_ = false;
    scan_requested_ = scan_requested_ || scan;
    snapshot_.state = scan ? State::kScanning : State::kPlaying;
    snapshot_.elapsed_seconds = 0;
    snapshot_.message = scan ? "Scanning..." : "Loading...";
    if (scan) {
        snapshot_.title.clear();
        snapshot_.total = 0;
    }
    xTaskNotifyGive(worker_);
}

void SdMusicPlayer::Start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || running_ || !EnsureWorkerLocked()) return;
    running_ = true;
    RestartLocked(true);
}

void SdMusicPlayer::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    running_ = false;
    paused_ = false;
    scan_requested_ = false;
    if (token_) audio_.EndLocalPlayback(token_);
    token_ = 0;
    snapshot_.state = State::kStopped;
    snapshot_.message.clear();
    if (worker_) xTaskNotifyGive(worker_);
}

void SdMusicPlayer::TogglePause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !token_ || (snapshot_.state != State::kPlaying && snapshot_.state != State::kPaused)) return;
    paused_ = !paused_;
    audio_.PauseLocalPlayback(token_, paused_);
    snapshot_.state = paused_ ? State::kPaused : State::kPlaying;
    snapshot_.message = paused_ ? "Paused" : "Playing";
    xTaskNotifyGive(worker_);
}

void SdMusicPlayer::ChangeTrack(int direction) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !snapshot_.total || snapshot_.state == State::kScanning) return;
    selected_index_ = (selected_index_ + snapshot_.total + direction) % snapshot_.total;
    RestartLocked(false);
}

void SdMusicPlayer::Next() { ChangeTrack(1); }
void SdMusicPlayer::Previous() { ChangeTrack(-1); }

void SdMusicPlayer::Rescan() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || !EnsureWorkerLocked()) return;
    running_ = true;
    RestartLocked(true);
}

SdMusicPlayer::Snapshot SdMusicPlayer::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

bool SdMusicPlayer::IsCurrent(uint32_t generation) const { return generation_.load() == generation; }

bool SdMusicPlayer::WaitUntilPlaying(uint32_t generation) {
    while (IsCurrent(generation)) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_ || shutdown_) return false;
            if (!paused_) return true;
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    return false;
}

void SdMusicPlayer::SetState(uint32_t generation, State state, const char* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!IsCurrent(generation) || !running_) return;
    snapshot_.state = state;
    snapshot_.message = message;
}

void SdMusicPlayer::Unmount() {
    if (card_) {
        esp_vfs_fat_sdcard_unmount(kMountPoint, card_);
        card_ = nullptr;
    }
    tracks_.clear();
}

void SdMusicPlayer::ScanDirectory(const std::string& directory, unsigned depth, size_t& visited,
                                uint32_t generation, std::vector<std::string>& found) {
    if (!IsCurrent(generation) || depth > kMaxDepth || visited >= kMaxEntries || found.size() >= kMaxTracks) return;
    std::unique_ptr<DIR, CloseDirectory> dir(opendir(directory.c_str()));
    if (!dir) return;
    while (IsCurrent(generation) && visited < kMaxEntries && found.size() < kMaxTracks) {
        const auto* entry = readdir(dir.get());
        if (!entry) break;
        ++visited;
        if ((visited & 31U) == 0) vTaskDelay(1);
        const std::string name(entry->d_name);
        if (name.empty() || name.front() == '.') continue;
        const auto path = directory + "/" + name;
        if (path.size() > 512) continue;
#ifdef DT_LNK
        if (entry->d_type == DT_LNK) continue;
#endif
        struct stat info{};
        if (stat(path.c_str(), &info) != 0) continue;
        if (S_ISDIR(info.st_mode)) {
            auto lower = name;
            for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (depth > 0 || lower == "music") ScanDirectory(path, depth + 1, visited, generation, found);
        } else if (S_ISREG(info.st_mode) && info.st_size > 0) {
            const auto extension = Extension(name);
            if (extension == ".mp3" || extension == ".wav") found.push_back(path);
        }
    }
}

bool SdMusicPlayer::MountAndScan(uint32_t generation) {
    Unmount();
    if (!IsCurrent(generation)) return false;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = SD_MMC_CLK_GPIO;
    slot.cmd = SD_MMC_CMD_GPIO;
    slot.d0 = SD_MMC_D0_GPIO;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_sdmmc_mount_config_t mount{};
    mount.format_if_mount_failed = false;
    mount.max_files = 5;
    mount.allocation_unit_size = 16 * 1024;
    const auto result = esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &card_);
    if (!IsCurrent(generation)) return false;
    if (result != ESP_OK) {
        card_ = nullptr;
        SetState(generation, State::kNoCard, "Insert a FAT32 SD card");
        return false;
    }
    size_t visited = 0;
    std::vector<std::string> found;
    found.reserve(kMaxTracks);
    ScanDirectory(kMountPoint, 0, visited, generation, found);
    if (!IsCurrent(generation)) return false;
    std::sort(found.begin(), found.end());
    tracks_ = std::move(found);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!IsCurrent(generation) || !running_) return false;
    snapshot_.total = tracks_.size();
    if (tracks_.empty()) {
        snapshot_.state = State::kEmpty;
        snapshot_.message = "No MP3 or PCM WAV files";
        return false;
    }
    selected_index_ %= tracks_.size();
    snapshot_.index = selected_index_;
    return true;
}

bool SdMusicPlayer::Advance(const Command& command, bool failed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!IsCurrent(command.generation) || !running_ || tracks_.empty()) return false;
    if (failed) ++consecutive_failures_;
    else consecutive_failures_ = 0;
    if (consecutive_failures_ >= tracks_.size()) {
        snapshot_.state = State::kError;
        snapshot_.message = "No playable audio files";
        return false;
    }
    selected_index_ = (command.index + 1) % tracks_.size();
    RestartLocked(false);
    return true;
}

void SdMusicPlayer::Worker() {
    uint32_t handled = generation_.load() - 1;
    while (true) {
        Command command{};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) break;
            command = {generation_.load(), token_, selected_index_, running_, scan_requested_};
            if (command.generation != handled) scan_requested_ = false;
        }
        if (command.generation == handled) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        handled = command.generation;
        if (!command.running) {
            Unmount();
            continue;
        }
        try {
            if (command.scan) {
                consecutive_failures_ = 0;
                if (!MountAndScan(command.generation)) continue;
                std::lock_guard<std::mutex> lock(mutex_);
                if (!IsCurrent(command.generation)) continue;
                command.index = selected_index_;
            }
            if (!IsCurrent(command.generation) || tracks_.empty()) continue;
            const auto result = PlayTrack(command);
            if (result == TrackResult::kComplete || result == TrackResult::kBadFile) {
                Advance(command, result == TrackResult::kBadFile);
            } else if (result == TrackResult::kAudioError) {
                SetState(command.generation, State::kError, "Audio unavailable");
            }
        } catch (const std::exception& error) {
            ESP_LOGE(kTag, "Player operation failed: %s", error.what());
            SetState(command.generation, State::kError, "Player resource error");
        }
    }
    Unmount();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_ = nullptr;
        worker_done_.notify_all();
    }
    vTaskDelete(nullptr);
}

SdMusicPlayer::TrackResult SdMusicPlayer::PlayTrack(const Command& command) {
    if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
    if (command.index >= tracks_.size() || !RegisterDecoders()) return TrackResult::kBadFile;
    TrackResources resources;
    resources.file.reset(fopen(tracks_[command.index].c_str(), "rb"));
    if (!resources.file) return TrackResult::kBadFile;
    const bool wav = Extension(tracks_[command.index]) == ".wav";
    if (wav && !ValidateWav(resources.file.get(), [this, &command] { return IsCurrent(command.generation); })) {
        return IsCurrent(command.generation) ? TrackResult::kBadFile : TrackResult::kCancelled;
    }
    if (!wav && !SkipId3Tag(resources.file.get())) return TrackResult::kBadFile;
    if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
    esp_audio_simple_dec_cfg_t config{};
    config.dec_type = wav ? ESP_AUDIO_SIMPLE_DEC_TYPE_WAV : ESP_AUDIO_SIMPLE_DEC_TYPE_MP3;
    config.use_frame_dec = false;
    if (esp_audio_simple_dec_open(&config, &resources.decoder) != ESP_AUDIO_ERR_OK) return TrackResult::kBadFile;
    const uint32_t output_rate = Board::GetInstance().GetAudioCodec()->output_sample_rate();
    if (!SupportedRate(output_rate) || output_rate > 48000) return TrackResult::kAudioError;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
        snapshot_.index = command.index;
        snapshot_.title = TrackTitle(tracks_[command.index]);
        snapshot_.elapsed_seconds = 0;
        snapshot_.state = paused_ ? State::kPaused : State::kPlaying;
        snapshot_.message = paused_ ? "Paused" : "Playing";
    }
    std::vector<uint8_t> input(kMaxInputBytes);
    std::vector<uint8_t> decoded(8192);
    std::vector<int16_t> mono;
    std::vector<int16_t> converted;
    std::vector<int16_t> pending;
    const size_t block_samples = output_rate / 50;
    pending.reserve(block_samples);
    uint64_t submitted_samples = 0;
    size_t input_size = 0;
    size_t read_before_audio = 0;
    bool eof = false;
    bool got_audio = false;
    unsigned decode_errors = 0;
    unsigned no_input_progress = 0;
    uint32_t source_rate = 0;

    auto push_pending = [&]() -> TrackResult {
        unsigned stalls = 0;
        while (!pending.empty()) {
            if (!WaitUntilPlaying(command.generation)) return TrackResult::kCancelled;
            const auto count = pending.size();
            if (audio_.PushLocalPcm(command.token, pending)) {
                submitted_samples += count;
                pending.clear();
                std::lock_guard<std::mutex> lock(mutex_);
                if (IsCurrent(command.generation)) snapshot_.elapsed_seconds = submitted_samples / output_rate;
                return TrackResult::kComplete;
            }
            if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
            if (++stalls >= 30) return TrackResult::kAudioError;
        }
        return TrackResult::kComplete;
    };

    auto emit_pcm = [&](const uint8_t* data, size_t size) -> TrackResult {
        esp_audio_simple_dec_info_t info{};
        if (esp_audio_simple_dec_get_info(resources.decoder, &info) != ESP_AUDIO_ERR_OK ||
            info.bits_per_sample != 16 || (info.channel != 1 && info.channel != 2) ||
            !SupportedRate(info.sample_rate) || size % (info.channel * 2) != 0) return TrackResult::kBadFile;
        if (source_rate && source_rate != info.sample_rate) return TrackResult::kBadFile;
        if (!source_rate) {
            source_rate = info.sample_rate;
            if (source_rate != output_rate) {
                esp_ae_rate_cvt_cfg_t rate_config{};
                rate_config.src_rate = source_rate;
                rate_config.dest_rate = output_rate;
                rate_config.channel = 1;
                rate_config.bits_per_sample = 16;
                rate_config.complexity = 2;
                rate_config.perf_type = ESP_AE_RATE_CVT_PERF_TYPE_SPEED;
                if (esp_ae_rate_cvt_open(&rate_config, &resources.converter) != ESP_AE_ERR_OK) return TrackResult::kBadFile;
            }
        }
        const size_t frames = size / (info.channel * 2);
        for (size_t first = 0; first < frames;) {
            if (!WaitUntilPlaying(command.generation)) return TrackResult::kCancelled;
            const size_t count = std::min<size_t>(frames - first, source_rate / 50);
            mono.resize(count);
            for (size_t i = 0; i < count; ++i) {
                const auto* sample = data + (first + i) * info.channel * 2;
                const int32_t left = static_cast<int16_t>(ReadLe16(sample));
                mono[i] = info.channel == 1 ? left : (left + static_cast<int16_t>(ReadLe16(sample + 2))) / 2;
            }
            const int16_t* samples = mono.data();
            size_t sample_count = mono.size();
            if (resources.converter) {
                uint32_t capacity = 0;
                if (esp_ae_rate_cvt_get_max_out_sample_num(resources.converter, count, &capacity) != ESP_AE_ERR_OK ||
                    capacity > 4096) return TrackResult::kBadFile;
                converted.resize(std::max<uint32_t>(capacity, 1));
                if (esp_ae_rate_cvt_process(resources.converter, mono.data(), count, converted.data(), &capacity) != ESP_AE_ERR_OK ||
                    capacity > converted.size()) return TrackResult::kBadFile;
                samples = converted.data();
                sample_count = capacity;
            }
            for (size_t position = 0; position < sample_count;) {
                const auto copy = std::min(block_samples - pending.size(), sample_count - position);
                pending.insert(pending.end(), samples + position, samples + position + copy);
                position += copy;
                if (pending.size() == block_samples) {
                    const auto status = push_pending();
                    if (status != TrackResult::kComplete) return status;
                }
            }
            first += count;
        }
        return TrackResult::kComplete;
    };

    auto read_more = [&]() -> bool {
        if (eof || input_size == input.size()) return false;
        const size_t read = fread(input.data() + input_size, 1,
                                  std::min(kReadBytes, input.size() - input_size), resources.file.get());
        input_size += read;
        if (!got_audio) read_before_audio += read;
        eof = feof(resources.file.get());
        return read != 0;
    };
    while (WaitUntilPlaying(command.generation)) {
        if (!input_size && !eof) read_more();
        if (ferror(resources.file.get()) || read_before_audio > kMaxBytesBeforeAudio) return TrackResult::kBadFile;
        esp_audio_simple_dec_raw_t raw{};
        raw.buffer = input.data();
        raw.len = input_size;
        raw.eos = eof;
        esp_audio_simple_dec_out_t output{};
        output.buffer = decoded.data();
        output.len = decoded.size();
        const auto error = esp_audio_simple_dec_process(resources.decoder, &raw, &output);
        if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
        if (raw.consumed > input_size || output.decoded_size > decoded.size()) return TrackResult::kBadFile;
        if (error == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
            if (output.needed_size <= decoded.size() || output.needed_size > kMaxOutputBytes) return TrackResult::kBadFile;
            decoded.resize(output.needed_size);
            // The simple decoder contract retries the same input on insufficient output capacity.
            continue;
        }
        if (error != ESP_AUDIO_ERR_OK && error != ESP_AUDIO_ERR_DATA_LACK && error != ESP_AUDIO_ERR_CONTINUE) {
            if (eof && !input_size && got_audio) break;
            if (!raw.consumed || ++decode_errors > 4) return TrackResult::kBadFile;
        } else if (output.decoded_size) {
            const auto status = emit_pcm(decoded.data(), output.decoded_size);
            if (status != TrackResult::kComplete) return status;
            got_audio = true;
            decode_errors = 0;
        }
        if (raw.consumed) {
            input_size -= raw.consumed;
            memmove(input.data(), input.data() + raw.consumed, input_size);
            no_input_progress = 0;
        } else if (++no_input_progress > 32) {
            return TrackResult::kBadFile;
        }
        if (!raw.consumed && !output.decoded_size) {
            if (eof) {
                if (input_size || !got_audio) return TrackResult::kBadFile;
                break;
            }
            // A short/empty final read may be the first time EOF is known.
            // Retry with eos=true so a parser can release its buffered frame.
            if (!read_more() && (!eof || ferror(resources.file.get()))) return TrackResult::kBadFile;
        }
    }
    if (!IsCurrent(command.generation)) return TrackResult::kCancelled;
    if (!got_audio) return TrackResult::kBadFile;
    const auto status = push_pending();
    if (status != TrackResult::kComplete) return status;
    while (WaitUntilPlaying(command.generation)) {
        if (audio_.IsPlaybackComplete()) return TrackResult::kComplete;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return TrackResult::kCancelled;
}

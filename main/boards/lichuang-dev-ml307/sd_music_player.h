#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>

class AudioService;

class SdMusicPlayer {
public:
    enum class State { kStopped, kScanning, kPlaying, kPaused, kNoCard, kEmpty, kError };
    struct Snapshot {
        State state = State::kStopped;
        std::string title;
        size_t index = 0;
        size_t total = 0;
        uint32_t elapsed_seconds = 0;
        std::string message;
    };

    explicit SdMusicPlayer(AudioService& audio);
    ~SdMusicPlayer();
    SdMusicPlayer(const SdMusicPlayer&) = delete;
    SdMusicPlayer& operator=(const SdMusicPlayer&) = delete;

    void Start();
    // Immediately invalidates pending work and releases the local audio token.
    // File/decoder/unmount cleanup then runs on the SD worker.
    void Stop();
    void TogglePause();
    void Next();
    void Previous();
    void Rescan();
    Snapshot GetSnapshot() const;

private:
    enum class TrackResult { kComplete, kBadFile, kCancelled, kAudioError };
    struct Command {
        uint32_t generation;
        uint32_t token;
        size_t index;
        bool running;
        bool scan;
    };
    AudioService& audio_;
    mutable std::mutex mutex_;
    std::condition_variable worker_done_;
    TaskHandle_t worker_ = nullptr;
    std::atomic<uint32_t> generation_{0};
    bool shutdown_ = false;
    bool running_ = false;
    bool scan_requested_ = false;
    bool paused_ = false;
    uint32_t token_ = 0;
    size_t selected_index_ = 0;
    Snapshot snapshot_;

    // Accessed only by Worker().
    sdmmc_card_t* card_ = nullptr;
    std::vector<std::string> tracks_;
    size_t consecutive_failures_ = 0;

    bool EnsureWorkerLocked();
    void RestartLocked(bool scan);
    void ChangeTrack(int direction);
    bool IsCurrent(uint32_t generation) const;
    bool WaitUntilPlaying(uint32_t generation);
    void SetState(uint32_t generation, State state, const char* message);
    void Worker();
    bool MountAndScan(uint32_t generation);
    void ScanDirectory(const std::string& directory, unsigned depth, size_t& visited,
                       uint32_t generation, std::vector<std::string>& found);
    void Unmount();
    TrackResult PlayTrack(const Command& command);
    bool Advance(const Command& command, bool failed);
};

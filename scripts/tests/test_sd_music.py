#!/usr/bin/env python3
"""Run the complete production SD music player with host filesystem/audio stubs.

Run in Linux/WSL with Python 3 and g++. Does not invoke ESP-IDF or touch an SD card.
The parser/decoder/resampler hardware boundary is simulated; control, scanning,
WAV validation, streaming, cancellation and PCM chunking use production code.
"""

from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "main/boards/lichuang-dev-ml307"

HARNESS = r'''
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace fs = std::filesystem;
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define pdPASS 1
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
struct FakeTask { std::mutex mutex; std::condition_variable cv; unsigned notifications = 0; };
using TaskHandle_t = FakeTask*;
thread_local FakeTask* current_task;
int xTaskCreate(void (*fn)(void*), const char*, unsigned, void* arg, unsigned, TaskHandle_t* handle) {
    auto* task = new FakeTask;
    *handle = task;
    std::thread([=] { current_task = task; fn(arg); current_task = nullptr; delete task; }).detach();
    return pdPASS;
}
void xTaskNotifyGive(FakeTask* task) {
    std::lock_guard<std::mutex> lock(task->mutex);
    ++task->notifications;
    task->cv.notify_all();
}
unsigned ulTaskNotifyTake(int, uint32_t timeout) {
    assert(current_task);
    std::unique_lock<std::mutex> lock(current_task->mutex);
    if (timeout == portMAX_DELAY) current_task->cv.wait(lock, [] { return current_task->notifications != 0; });
    else current_task->cv.wait_for(lock, std::chrono::milliseconds(timeout), [] { return current_task->notifications != 0; });
    const auto count = current_task->notifications;
    current_task->notifications = 0;
    return count;
}
void vTaskDelay(uint32_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
void vTaskDelete(void*) {}

using esp_audio_err_t = int;
constexpr int ESP_AUDIO_ERR_OK = 0, ESP_AUDIO_ERR_FAIL = -1, ESP_AUDIO_ERR_DATA_LACK = -3;
constexpr int ESP_AUDIO_ERR_CONTINUE = 1, ESP_AUDIO_ERR_ALREADY_EXIST = -6, ESP_AUDIO_ERR_BUFF_NOT_ENOUGH = -8;
constexpr int ESP_AUDIO_SIMPLE_DEC_TYPE_WAV = 1, ESP_AUDIO_SIMPLE_DEC_TYPE_MP3 = 2;
struct esp_audio_simple_dec_cfg_t { int dec_type = 0; bool use_frame_dec = false; };
struct esp_audio_simple_dec_raw_t { uint8_t* buffer; uint32_t len; bool eos; uint32_t consumed; };
struct esp_audio_simple_dec_out_t { uint8_t* buffer; uint32_t len, needed_size, decoded_size; };
struct esp_audio_simple_dec_info_t { uint32_t sample_rate; uint8_t bits_per_sample, channel; };
struct FakeDecoder { char mode = 0; unsigned calls = 0; };
using esp_audio_simple_dec_handle_t = FakeDecoder*;
static std::atomic<int> decoder_calls, decoder_opens, decoder_closes, registrations, converter_opens;
static std::string consumed_input;
static std::function<void()> decode_hook;
int esp_mp3_dec_register() { ++registrations; return ESP_AUDIO_ERR_ALREADY_EXIST; }
int esp_pcm_dec_register() { ++registrations; return ESP_AUDIO_ERR_OK; }
int esp_wav_dec_register() { ++registrations; return ESP_AUDIO_ERR_OK; }
int esp_audio_simple_dec_open(esp_audio_simple_dec_cfg_t*, esp_audio_simple_dec_handle_t* out) {
    *out = new FakeDecoder; ++decoder_opens; return 0;
}
void esp_audio_simple_dec_close(FakeDecoder* decoder) { delete decoder; ++decoder_closes; }
int esp_audio_simple_dec_process(FakeDecoder* decoder, esp_audio_simple_dec_raw_t* raw, esp_audio_simple_dec_out_t* out) {
    ++decoder_calls;
    if (decode_hook) decode_hook();
    if (!raw->len) return 0;
    if (!decoder->mode) decoder->mode = static_cast<char>(raw->buffer[0]);
    ++decoder->calls;
    if (decoder->mode == 'X') return ESP_AUDIO_ERR_FAIL;
    if (decoder->mode == 'Z') return 0;
    if (decoder->mode == 'E' && !raw->eos) return 0;
    if (decoder->mode == 'H') { out->needed_size = 100000; return ESP_AUDIO_ERR_BUFF_NOT_ENOUGH; }
    if (decoder->mode == 'C') { raw->consumed = raw->len + 1; return 0; }
    if (decoder->mode == 'B' && out->len < 12000) { out->needed_size = 12000; return ESP_AUDIO_ERR_BUFF_NOT_ENOUGH; }
    raw->consumed = decoder->mode == 'P' ? std::min<uint32_t>(13, raw->len) : raw->len;
    consumed_input.append(reinterpret_cast<char*>(raw->buffer), raw->consumed);
    if (decoder->mode == 'D') return ESP_AUDIO_ERR_DATA_LACK;
    out->decoded_size = decoder->mode == 'B' ? 12000 : 1280;
    assert(out->decoded_size <= out->len);
    for (size_t i = 0; i < out->decoded_size; i += 4) {
        const int16_t left = 1000, right = -500;
        memcpy(out->buffer + i, &left, 2); memcpy(out->buffer + i + 2, &right, 2);
    }
    return 0;
}
int esp_audio_simple_dec_get_info(FakeDecoder* decoder, esp_audio_simple_dec_info_t* out) {
    out->sample_rate = decoder->mode == 'S' ? 48000 : 16000;
    out->bits_per_sample = 16; out->channel = 2; return 0;
}
constexpr int ESP_AE_ERR_OK = 0, ESP_AE_RATE_CVT_PERF_TYPE_SPEED = 1;
struct esp_ae_rate_cvt_cfg_t {
    uint32_t src_rate, dest_rate; uint8_t channel, bits_per_sample, complexity; int perf_type;
};
struct FakeConverter { uint32_t src, dst; };
using esp_ae_rate_cvt_handle_t = FakeConverter*;
int esp_ae_rate_cvt_open(esp_ae_rate_cvt_cfg_t* config, FakeConverter** out) {
    assert(config->channel == 1 && config->bits_per_sample == 16 && config->dest_rate == 16000);
    *out = new FakeConverter{config->src_rate, config->dest_rate}; ++converter_opens; return 0;
}
void esp_ae_rate_cvt_close(FakeConverter* converter) { delete converter; }
int esp_ae_rate_cvt_get_max_out_sample_num(FakeConverter* converter, uint32_t in, uint32_t* out) {
    *out = (in * converter->dst + converter->src - 1) / converter->src + 2; return 0;
}
int esp_ae_rate_cvt_process(FakeConverter* converter, int16_t* in, uint32_t count, int16_t* out, uint32_t* written) {
    assert(count <= converter->src / 50);
    *written = count * converter->dst / converter->src;
    std::fill(out, out + *written, in[0]); return 0;
}
struct AudioService {
    std::mutex mutex;
    uint32_t sequence = 0, active = 0;
    bool paused = false;
    unsigned refusals = 0;
    std::vector<int16_t> received;
    std::vector<int16_t> refused;
    std::function<void()> push_hook;
    uint32_t BeginLocalPlayback() { std::lock_guard lock(mutex); paused = false; return active = ++sequence; }
    void EndLocalPlayback(uint32_t token) { std::lock_guard lock(mutex); if (active == token) active = 0; }
    void PauseLocalPlayback(uint32_t token, bool value) { std::lock_guard lock(mutex); if (active == token) paused = value; }
    bool IsLocalPlaybackActive() { std::lock_guard lock(mutex); return active != 0; }
    bool IsPlaybackComplete() { return true; }
    bool PushLocalPcm(uint32_t token, std::vector<int16_t>& pcm) {
        if (current_task) std::this_thread::sleep_for(1ms);
        if (push_hook) push_hook();
        std::lock_guard lock(mutex);
        assert(!pcm.empty() && pcm.size() <= 320);
        if (token != active || paused) return false;
        if (refusals) { --refusals; if (refused.empty()) refused = pcm; else assert(refused == pcm); return false; }
        if (!refused.empty()) { assert(refused == pcm); refused.clear(); }
        received.insert(received.end(), pcm.begin(), pcm.end());
        pcm.clear();
        return true;
    }
};
struct Codec { int output_sample_rate() const { return 16000; } };
struct Board {
    static Board& GetInstance() { static Board board; return board; }
    Codec* GetAudioCodec() { static Codec codec; return &codec; }
};
struct sdmmc_card_t {};
struct sdmmc_host_t { int flags = 0, max_freq_khz = 0; };
struct sdmmc_slot_config_t { int width = 0, clk = 0, cmd = 0, d0 = 0, flags = 0; };
struct esp_vfs_fat_sdmmc_mount_config_t { bool format_if_mount_failed; int max_files; size_t allocation_unit_size; };
#define SDMMC_HOST_DEFAULT() sdmmc_host_t{}
#define SDMMC_SLOT_CONFIG_DEFAULT() sdmmc_slot_config_t{}
#define SDMMC_HOST_FLAG_1BIT 1
#define SDMMC_FREQ_DEFAULT 20000
#define SDMMC_SLOT_FLAG_INTERNAL_PULLUP 1
#define SD_MMC_CLK_GPIO 47
#define SD_MMC_CMD_GPIO 48
#define SD_MMC_D0_GPIO 21
#define ESP_OK 0
static std::atomic<bool> card_available{true};
static std::atomic<int> mounts, unmounts;
static std::function<void()> mount_hook;
int esp_vfs_fat_sdmmc_mount(const char*, sdmmc_host_t* host, sdmmc_slot_config_t* slot,
        esp_vfs_fat_sdmmc_mount_config_t* config, sdmmc_card_t** card) {
    assert(!config->format_if_mount_failed);
    assert(host->flags == SDMMC_HOST_FLAG_1BIT && slot->width == 1);
    assert(slot->clk == 47 && slot->cmd == 48 && slot->d0 == 21);
    ++mounts;
    if (mount_hook) mount_hook();
    if (!card_available) return -1;
    *card = new sdmmc_card_t;
    return 0;
}
void esp_vfs_fat_sdcard_unmount(const char*, sdmmc_card_t* card) { ++unmounts; delete card; }
static fs::path card_root;
std::string actual_path(const char* path) {
    std::string text(path);
    if (text.starts_with("/sdcard")) return card_root.string() + text.substr(7);
    return text;
}
FILE* test_fopen(const char* path, const char* mode) { return fopen(actual_path(path).c_str(), mode); }
DIR* test_opendir(const char* path) { return opendir(actual_path(path).c_str()); }
int test_stat(const char* path, struct stat* info) { return stat(actual_path(path).c_str(), info); }
'''

TESTS = r'''
void write_file(const fs::path& path, const std::string& data) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary); file.write(data.data(), data.size());
}
std::string wav(uint16_t format = 1, uint16_t bits = 16) {
    std::string data(44 + 128, '\0');
    auto u16 = [&](size_t offset, uint16_t n) { data[offset] = n; data[offset + 1] = n >> 8; };
    auto u32 = [&](size_t offset, uint32_t n) { for (int i = 0; i < 4; ++i) data[offset + i] = n >> (i * 8); };
    data.replace(0, 4, "RIFF"); u32(4, data.size() - 8); data.replace(8, 4, "WAVE");
    data.replace(12, 4, "fmt "); u32(16, 16); u16(20, format); u16(22, 2);
    u32(24, 16000); u32(28, 64000); u16(32, 4); u16(34, bits);
    data.replace(36, 4, "data"); u32(40, 128);
    return data;
}
std::string id3(size_t size, bool footer = false) {
    std::string tag(10, '\0');
    tag.replace(0, 3, "ID3"); tag[3] = 4; tag[5] = footer ? 0x10 : 0;
    for (int i = 9; i >= 6; --i) { tag[i] = size & 0x7f; size >>= 7; }
    return tag;
}
void prepare(SdMusicPlayer& player, AudioService& audio, std::string file) {
    player.generation_ = 1; player.running_ = true; player.token_ = audio.BeginLocalPlayback();
    player.tracks_ = {std::move(file)}; player.snapshot_.total = 1;
}
template<class Predicate> void wait_until(Predicate ready) {
    auto limit = std::chrono::steady_clock::now() + 3s;
    while (!ready()) { assert(std::chrono::steady_clock::now() < limit); std::this_thread::sleep_for(2ms); }
}
int main(int argc, char** argv) {
    assert(argc == 2);
    const fs::path fixture(argv[1]);
    card_root = fixture / "scan";
    write_file(card_root / "中文.MP3", "G");
    write_file(card_root / "plain.WaV", wav());
    write_file(card_root / "music/a/b/three.mp3", "G");
    write_file(card_root / "music/a/b/c/too-deep.mp3", "G");
    write_file(card_root / "other/ignored.mp3", "G");
    write_file(card_root / ".hidden.mp3", "G");
    write_file(card_root / "music/wrong.flac", "G");
    {
        AudioService audio; SdMusicPlayer player(audio);
        player.generation_ = 1;
        size_t visited = 0; std::vector<std::string> found;
        player.ScanDirectory("/sdcard", 0, visited, 1, found);
        assert(found.size() == 3);
        assert(std::find(found.begin(), found.end(), "/sdcard/music/a/b/three.mp3") != found.end());
        found.clear(); visited = 4095;
        player.ScanDirectory("/sdcard", 0, visited, 1, found); assert(visited == 4096);
        found.clear(); visited = 0; player.generation_ = 2;
        player.ScanDirectory("/sdcard", 0, visited, 1, found); assert(found.empty() && visited == 0);
        for (int i = 0; i < 260; ++i) write_file(card_root / ("song" + std::to_string(i) + ".mp3"), "G");
        player.ScanDirectory("/sdcard", 0, visited, 2, found); assert(found.size() == 256);
    }
    std::cout << "PASS: scan paths, case/UTF-8 names, recursion/entry/track bounds and cancellation\n";

    card_root = fixture / "play";
    write_file(card_root / "good.wav", wav());
    write_file(card_root / "float.wav", wav(3));
    write_file(card_root / "24bit.wav", wav(1, 24));
    write_file(card_root / "short.wav", "RIFF");
    for (const auto* name : {"good.wav", "float.wav", "24bit.wav", "short.wav"}) {
        FILE* file = test_fopen((std::string("/sdcard/") + name).c_str(), "rb");
        assert(file);
        assert(ValidateWav(file, [] { return true; }) == (std::string(name) == "good.wav"));
        if (std::string(name) == "good.wav") {
            assert(ftell(file) == 0);
            assert(!ValidateWav(file, [] { return false; }));
        }
        fclose(file);
    }
    std::cout << "PASS: real RIFF chunk validation accepts only supported 16-bit PCM WAV\n";

    for (bool footer : {false, true}) {
        auto header = id3(300000, footer);
        auto tagged = header + std::string(300000, 'x');
        if (footer) { header.replace(0, 3, "3DI"); tagged += header; }
        tagged += "Gmusic";
        write_file(card_root / "tagged.mp3", tagged);
        AudioService audio; SdMusicPlayer player(audio);
        prepare(player, audio, "/sdcard/tagged.mp3");
        consumed_input.clear();
        assert(player.PlayTrack({1, player.token_, 0, true, false}) == SdMusicPlayer::TrackResult::kComplete);
        assert(consumed_input == "Gmusic");
    }
    for (auto malformed : {id3(999999) + "short", std::string("ID3"),
            id3(0, true) + "bad footerGmusic", std::string("ID3\4\0\0\xff\0\0\0", 10)}) {
        write_file(card_root / "tagged.mp3", malformed);
        FILE* file = test_fopen("/sdcard/tagged.mp3", "rb");
        assert(file && !SkipId3Tag(file)); fclose(file);
    }
    std::cout << "PASS: large ID3 artwork is skipped; malformed sizes/headers/footers are rejected\n";

    for (char mode : std::string("GPBSEZHCXD")) {
        const auto data = std::string(1, mode) + std::string(mode == 'Z' ? 20000 : mode == 'E' ? 4095 : 99, 'x');
        write_file(card_root / "test.mp3", data);
        AudioService audio; SdMusicPlayer player(audio);
        prepare(player, audio, "/sdcard/test.mp3");
        audio.refusals = 2;
        consumed_input.clear();
        const int before = decoder_calls;
        const auto result = player.PlayTrack({1, player.token_, 0, true, false});
        const bool good = std::string("GPBSE").find(mode) != std::string::npos;
        assert((result == SdMusicPlayer::TrackResult::kComplete) == good);
        assert(decoder_calls - before < 40);
        if (good) {
            assert(consumed_input == data);
            assert(!audio.received.empty());
            assert(std::all_of(audio.received.begin(), audio.received.end(), [](int16_t value) { return value == 250; }));
        }
        assert(decoder_opens == decoder_closes);
    }
    assert(registrations == 3 && converter_opens > 0);
    std::cout << "PASS: decode partial consumption, bounded buffer growth/no-progress/errors, downmix/rate/chunks/retry\n";

    write_file(card_root / "cancel.mp3", "Gdata");
    {
        AudioService audio; SdMusicPlayer player(audio); prepare(player, audio, "/sdcard/cancel.mp3");
        uint32_t replacement = 0;
        decode_hook = [&] { player.Stop(); replacement = audio.BeginLocalPlayback(); };
        assert(player.PlayTrack({1, player.token_, 0, true, false}) == SdMusicPlayer::TrackResult::kCancelled);
        decode_hook = {};
        assert(audio.received.empty() && audio.active == replacement);
        player.Stop(); assert(audio.active == replacement); // A stale stop cannot clear another session.
    }
    std::cout << "PASS: cancellation after decode prevents stale PCM and preserves a newer audio session\n";

    card_root = fixture / "worker";
    fs::create_directories(card_root);
    {
        AudioService audio; SdMusicPlayer player(audio);
        card_available = false; player.Start();
        wait_until([&] { return player.GetSnapshot().state == SdMusicPlayer::State::kNoCard; });
        assert(audio.IsLocalPlaybackActive());
        const int tried = mounts;
        std::this_thread::sleep_for(20ms); assert(mounts == tried);
        card_available = true; player.Rescan();
        wait_until([&] { return player.GetSnapshot().state == SdMusicPlayer::State::kEmpty; });
        write_file(card_root / "bad-a.mp3", "X"); write_file(card_root / "bad-b.mp3", "X");
        const int opened = decoder_opens;
        player.Rescan();
        wait_until([&] { return player.GetSnapshot().state == SdMusicPlayer::State::kError; });
        assert(decoder_opens - opened == 2); // One bounded pass through an all-bad playlist.
        player.Stop(); assert(!audio.IsLocalPlaybackActive());
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kStopped);
    }
    assert(decoder_opens == decoder_closes);
    std::cout << "PASS: async no-card/empty/rescan/all-bad lifecycle and immediate stop\n";

    card_root = fixture / "paused";
    write_file(card_root / "a.mp3", "G" + std::string(300000, 'x'));
    write_file(card_root / "b.mp3", "G" + std::string(300000, 'x'));
    {
        std::atomic<bool> entered = false, release = false, first = true;
        AudioService audio;
        SdMusicPlayer player(audio);
        audio.push_hook = [&] {
            if (first.exchange(false)) {
                entered = true;
                while (!release) std::this_thread::sleep_for(1ms);
            }
        };
        player.Start(); wait_until([&] { return entered.load(); });
        player.TogglePause();
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kPaused);
        const int calls_at_pause = decoder_calls;
        release = true;
        std::this_thread::sleep_for(20ms);
        assert(decoder_calls == calls_at_pause);
        { std::lock_guard lock(audio.mutex); assert(audio.received.empty() && audio.active != 0); }
        player.TogglePause();
        wait_until([&] { std::lock_guard lock(audio.mutex); return !audio.received.empty(); });
        player.Next(); wait_until([&] { return player.GetSnapshot().index == 1; });
        player.Previous(); wait_until([&] { return player.GetSnapshot().index == 0; });
        player.Stop();
        size_t after_stop;
        { std::lock_guard lock(audio.mutex); after_stop = audio.received.size(); }
        std::this_thread::sleep_for(20ms);
        { std::lock_guard lock(audio.mutex); assert(audio.received.size() == after_stop && audio.active == 0); }
    }
    std::cout << "PASS: pause retains decoder/PCM position, resume retries, next/previous cancel old tracks\n";

    {
        std::atomic<bool> entered = false, release = false;
        mount_hook = [&] { entered = true; while (!release) std::this_thread::sleep_for(1ms); };
        AudioService audio;
        SdMusicPlayer player(audio);
        player.Start(); wait_until([&] { return entered.load(); });
        const auto start = std::chrono::steady_clock::now();
        player.Stop();
        assert(std::chrono::steady_clock::now() - start < 200ms);
        const auto replacement = audio.BeginLocalPlayback();
        release = true;
        wait_until([&] { std::lock_guard lock(player.mutex_); return player.snapshot_.state == SdMusicPlayer::State::kStopped; });
        std::this_thread::sleep_for(20ms);
        assert(audio.active == replacement);
    }
    mount_hook = {};
    assert(decoder_opens == decoder_closes);
    std::cout << "PASS: blocked SD mount does not block Stop or invalidate a new audio session\n";
}
'''


def without_includes(text):
    return re.sub(r"^\s*#(?:include|pragma)[^\n]*", "", text, flags=re.MULTILINE)


def main():
    header = without_includes((BOARD / "sd_music_player.h").read_text(encoding="utf-8"))
    source = without_includes((BOARD / "sd_music_player.cc").read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory(prefix="sd-music-test-") as name:
        directory = Path(name)
        cpp = directory / "test.cc"
        binary = directory / "test"
        cpp.write_text(HARNESS + "\n#define private public\n" + header + "\n#undef private\n" +
                       "#define fopen test_fopen\n#define opendir test_opendir\n#define stat(path, info) test_stat(path, info)\n" +
                       source + "\n#undef fopen\n#undef opendir\n#undef stat\n" + TESTS, encoding="utf-8")
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-g", "-pthread", "-fsanitize=address,undefined",
                        str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary), str(directory / "fixture")], check=True, timeout=30)


if __name__ == "__main__":
    main()

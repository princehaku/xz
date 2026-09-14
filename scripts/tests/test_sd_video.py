#!/usr/bin/env python3
"""Exercise production SD video/download code with host HTTP/JPEG hardware stubs.

Run in Linux/WSL using Python 3 and g++; never builds ESP-IDF or accesses a card.
Uses the real AVI parser, temporary files, production worker and cancellation.
JPEG output is simulated; hardware JPEG/display and network need board testing.
"""

import os
from pathlib import Path
import struct
import subprocess
import tempfile

import test_avi_reader as avi
import test_sd_music as music

BOARD = music.BOARD

AUDIO = r'''
// Demand-driven 16-kHz codec model: queued PCM drains at wall-clock speed,
// pauses freeze its position, and a new token starts a fresh output timeline.
struct AudioService {
    std::mutex mutex;
    uint32_t sequence = 0, active = 0;
    bool paused = false;
    unsigned refusals = 0;
    uint64_t queued = 0, played = 0, fraction = 0;
    size_t max_queued = 0;
    std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();
    std::vector<int16_t> received, refused;
    std::function<void()> push_hook;
    void AdvanceLocked() {
        const auto now = std::chrono::steady_clock::now();
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(now - tick).count();
        tick = now;
        if (active && !paused && queued > played) {
            const uint64_t budget = uint64_t(us) * 16000 + fraction;
            played = std::min(queued, played + budget / 1000000);
            fraction = played == queued ? 0 : budget % 1000000;
        }
    }
    uint32_t BeginLocalPlayback() {
        std::lock_guard lock(mutex);
        queued = played = fraction = 0; max_queued = 0; received.clear(); refused.clear();
        paused = false; tick = std::chrono::steady_clock::now(); return active = ++sequence;
    }
    void EndLocalPlayback(uint32_t token) {
        std::lock_guard lock(mutex); AdvanceLocked(); if (active == token) active = 0;
    }
    void PauseLocalPlayback(uint32_t token, bool value) {
        std::lock_guard lock(mutex); AdvanceLocked(); if (active == token) paused = value;
    }
    bool IsLocalPlaybackActive() { std::lock_guard lock(mutex); return active != 0; }
    uint64_t GetLocalPlaybackSamples(uint32_t token) {
        std::lock_guard lock(mutex); AdvanceLocked(); return token == active ? played : 0;
    }
    bool IsPlaybackComplete() {
        std::lock_guard lock(mutex); AdvanceLocked(); return !active || played == queued;
    }
    bool PushLocalPcm(uint32_t token, std::vector<int16_t>& pcm) {
        if (push_hook) push_hook();
        std::lock_guard lock(mutex); AdvanceLocked();
        assert(!pcm.empty() && pcm.size() <= 320);
        if (token != active || paused) return false;
        if (refusals) {
            --refusals; if (refused.empty()) refused = pcm; else assert(refused == pcm);
            return false;
        }
        if (!refused.empty()) { assert(refused == pcm); refused.clear(); }
        received.insert(received.end(), pcm.begin(), pcm.end());
        queued += pcm.size(); max_queued = std::max<size_t>(max_queued, queued - played);
        pcm.clear(); return true;
    }
};
'''

HTTP = r'''
static std::string http_body;
static int http_status = 200;
static size_t http_length = 0;
static bool http_open_ok = true;
static std::atomic<bool> http_read_error = false;
static std::function<void()> http_read_hook;
static std::atomic<int> http_reads = 0;
struct HttpStub {
    size_t position = 0;
    void SetTimeout(int ms) { assert(ms > 0 && ms <= 5000); }
    void SetKeepAlive(bool enabled) { assert(!enabled); }
    void SetHeader(const std::string&, const std::string&) {}
    bool Open(const std::string& method, const std::string&) { assert(method == "GET"); return http_open_ok; }
    int GetStatusCode() { return http_status; }
    size_t GetBodyLength() { return http_length; }
    int Read(char* data, size_t capacity) {
        assert(capacity <= 4096);
        ++http_reads;
        if (http_read_hook) http_read_hook();
        if (http_read_error) return -1;
        const size_t count = std::min(capacity, http_body.size() - position);
        memcpy(data, http_body.data() + position, count);
        position += count;
        return count;
    }
    void Close() {}
};
struct NetworkStub {
    std::unique_ptr<HttpStub> CreateHttp(int id) { assert(id == 4); return std::make_unique<HttpStub>(); }
};
'''

STUBS = r'''
static std::atomic<bool> fail_rename = false, fail_write = false, fail_sync = false, fail_jpeg = false;
static std::atomic<int> decoded_frames = 0;
int test_open(const char* path, int flags, int mode) { return open(actual_path(path).c_str(), flags, mode); }
int test_mkdir(const char* path, mode_t mode) { return mkdir(actual_path(path).c_str(), mode); }
int test_remove(const char* path) { return remove(actual_path(path).c_str()); }
int test_rename(const char* old_path, const char* new_path) {
    if (std::string(old_path) == "/sdcard/video/test.part" && fail_rename.exchange(false)) {
        errno = EIO; return -1;
    }
    return rename(actual_path(old_path).c_str(), actual_path(new_path).c_str());
}
size_t test_fwrite(const void* data, size_t size, size_t count, FILE* file) {
    if (fail_write) { errno = ENOSPC; return 0; }
    return fwrite(data, size, count, file);
}
int test_fsync(int fd) { if (fail_sync) { errno = EIO; return -1; } return fsync(fd); }
int64_t esp_timer_get_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
void heap_caps_free(void* data) { free(data); }
int jpeg_to_image(const uint8_t* data, size_t size, uint8_t** output, size_t* bytes,
                  size_t* width, size_t* height, size_t* stride) {
    assert(size >= 4 && data[0] == 0xff && data[1] == 0xd8);
    if (fail_jpeg) { *output = nullptr; return -1; }
    *width = 320; *height = 240; *stride = 640; *bytes = *stride * *height;
    *output = static_cast<uint8_t*>(malloc(*bytes)); assert(*output);
    memset(*output, ++decoded_frames, *bytes);
    return 0;
}
'''

TESTS = r'''
void write_file(const fs::path& path, const std::string& bytes) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary); file.write(bytes.data(), bytes.size());
}
std::string read_file(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}
template<class Predicate> void wait_until(Predicate predicate) {
    const auto limit = std::chrono::steady_clock::now() + 4s;
    while (!predicate()) { assert(std::chrono::steady_clock::now() < limit); std::this_thread::sleep_for(2ms); }
}
void prepare_download(SdMusicPlayer& player, AudioService& audio) {
    player.generation_ = 1; player.running_ = true; player.token_ = audio.BeginLocalPlayback();
}
int main(int argc, char** argv) {
    assert(argc == 4);
    const fs::path fixture(argv[1]);
    const std::string video = read_file(argv[2]);
    const fs::path audio_fixtures(argv[3]);
    assert(video.size() > 4096);
    auto reset_http = [&] {
        http_body = video; http_length = video.size(); http_status = 200; http_open_ok = true;
        http_read_error = false; http_read_hook = {}; http_reads = 0;
        fail_rename = false; fail_write = false; fail_sync = false; fail_jpeg = false;
    };
    reset_http();
    card_root = fixture / "urls";
    fs::create_directories(card_root);
    {
        AudioService audio; SdMusicPlayer player(audio);
        for (const auto& url : {"ftp://host/test.avi", "http://", "http://user:secret@host/test.avi",
                               "http://host/a\r\nHeader:x", "http://host/test.avi#fragment"}) {
            assert(!player.DownloadVideo(url));
        }
        assert(!player.DownloadVideo("http://host/" + std::string(1024, 'a')));
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kStopped && !player.worker_);
    }
    std::cout << "PASS: URL scheme/length/header/userinfo validation preserves current session\n";

    for (const auto* mode : {"truncated", "overflow", "status", "length", "large", "open", "read",
                             "write", "sync", "jpeg", "riff", "bad_second", "missing_frames", "rename",
                             "existing_part", "existing_backup", "ok"}) {
        reset_http(); card_root = fixture / mode;
        write_file(card_root / "video/test.avi", "previous video");
        write_file(card_root / "unrelated.txt", "keep me");
        const std::string test(mode);
        if (test == "truncated") http_body.resize(http_body.size() - 1);
        if (test == "overflow") --http_length;
        if (test == "status") http_status = 404;
        if (test == "length") http_length = 0;
        if (test == "large") http_length = 16 * 1024 * 1024 + 1;
        if (test == "open") http_open_ok = false;
        if (test == "read") http_read_error = true;
        if (test == "write") fail_write = true;
        if (test == "sync") fail_sync = true;
        if (test == "jpeg") fail_jpeg = true;
        if (test == "riff") http_body[0] = 'X';
        if (test == "bad_second") {
            const auto first = http_body.find(std::string("\xff\xd8", 2));
            const auto second = http_body.find(std::string("\xff\xd8", 2), first + 2);
            assert(second != std::string::npos); http_body[second + 1] = 0;
        }
        if (test == "missing_frames") {
            http_body.at(http_body.find("avih") + 8 + 16) = 5;
            http_body.at(http_body.find("strh") + 8 + 32) = 5;
        }
        if (test == "rename") fail_rename = true;
        if (test == "existing_part") write_file(card_root / "video/test.part", "user part");
        if (test == "existing_backup") write_file(card_root / "video/test-backup.avi", "user backup");
        AudioService audio; SdMusicPlayer player(audio); prepare_download(player, audio);
        assert(player.DownloadVideoFile({1, player.token_, 0, true, false, "http://host/video.avi"}) == (test == "ok"));
        assert(read_file(card_root / "video/test.avi") == (test == "ok" ? video : "previous video"));
        assert(read_file(card_root / "unrelated.txt") == "keep me");
        if (test == "existing_part") assert(read_file(card_root / "video/test.part") == "user part");
        else assert(!fs::exists(card_root / "video/test.part"));
        if (test == "existing_backup") assert(read_file(card_root / "video/test-backup.avi") == "user backup");
        else assert(!fs::exists(card_root / "video/test-backup.avi"));
    }
    std::cout << "PASS: HTTP/size/truncation/storage/JPEG errors preserve old file; validated replacement and rename rollback\n";

    for (const auto* mode : {"mono", "stereo", "unsupported", "missing_pcm", "unaligned_pcm"}) {
        reset_http(); card_root = fixture / (std::string("download-audio-") + mode);
        http_body = read_file(audio_fixtures / (std::string(mode) + ".avi"));
        http_length = http_body.size(); assert(http_length > 4096);
        write_file(card_root / "video/test.avi", "previous video");
        AudioService audio; SdMusicPlayer player(audio); prepare_download(player, audio);
        const bool valid = std::string(mode) == "mono" || std::string(mode) == "stereo";
        assert(player.DownloadVideoFile({1, player.token_, 0, true, false, "http://host/pcm.avi"}) == valid);
        assert(read_file(card_root / "video/test.avi") == (valid ? http_body : "previous video"));
        assert(!fs::exists(card_root / "video/test.part") && !fs::exists(card_root / "video/test-backup.avi"));
        if (!valid) {
            const auto status = player.GetSnapshot();
            assert(status.state == SdMusicPlayer::State::kError);
            assert(status.message.find("audio") != std::string::npos);
        }
    }
    std::cout << "PASS: PCM download verifies supported stream and complete aligned samples before replacing old AVI\n";

    for (const auto* mode : {"mono", "stereo"}) {
        card_root = fixture / (std::string("play-audio-") + mode);
        write_file(card_root / "video/test.avi", read_file(audio_fixtures / (std::string(mode) + ".avi")));
        AudioService audio; SdMusicPlayer player(audio); prepare_download(player, audio);
        player.tracks_ = {"/sdcard/video/test.avi"}; player.snapshot_.total = 1;
        audio.refusals = 2; // Rejected blocks must be retried without loss or duplication.
        const auto converters_before = converter_opens.load();
        FakeTask task; current_task = &task;
        const auto begin = std::chrono::steady_clock::now();
        assert(player.PlayVideoTrack({1, player.token_, 0, false, false, ""}) == SdMusicPlayer::TrackResult::kComplete);
        current_task = nullptr;
        const auto duration = std::chrono::steady_clock::now() - begin;
        assert(duration >= 380ms && duration < 2s);
        assert(audio.IsPlaybackComplete() && audio.GetLocalPlaybackSamples(player.token_) == 6400);
        const auto snapshot = player.GetSnapshot();
        assert(snapshot.video_has_audio && snapshot.video_audio_samples == 6400 && snapshot.video_frames == 4);
        std::lock_guard lock(audio.mutex);
        assert(audio.received.size() == 6400 && audio.refused.empty());
        assert(audio.max_queued <= 2240); // 120-ms target plus one indivisible 20-ms block.
        const int16_t mono[] = {0, 1, -1, 32767, -32768, 12345, -12345};
        for (size_t i = 0; i < audio.received.size(); ++i) {
            const int16_t expected = std::string(mode) == "mono" ? mono[i % 7] : (i / 320 % 2 ? -10000 : 10000);
            assert(audio.received[i] == expected);
        }
        assert(converter_opens == converters_before + (std::string(mode) == "stereo" ? 1 : 0));
    }
    std::cout << "PASS: exact PCM16 signed mono samples, bounded 20-ms retry, stereo48k downmix/resampler input and audio-clock duration\n";

    reset_http(); card_root = fixture / "cancel";
    write_file(card_root / "video/test.avi", "previous video");
    {
        std::atomic<bool> entered = false, release = false;
        http_read_hook = [&] {
            if (http_reads == 2) { entered = true; while (!release) std::this_thread::sleep_for(1ms); }
        };
        AudioService audio; SdMusicPlayer player(audio);
        assert(player.DownloadVideo("http://host/video.avi"));
        wait_until([&] { return entered.load(); });
        const auto before = std::chrono::steady_clock::now();
        player.Stop();
        assert(std::chrono::steady_clock::now() - before < 200ms);
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kStopped);
        assert(!player.GetVideoFrame() && !audio.IsLocalPlaybackActive());
        release = true;
        wait_until([&] { return !fs::exists(card_root / "video/test.part") && !host_initialized; });
        assert(read_file(card_root / "video/test.avi") == "previous video");
    }
    std::cout << "PASS: Stop immediately invalidates blocked HTTP, cleans partial file, preserves old AVI, unmounts\n";

    reset_http(); card_root = fixture / "worker"; fs::create_directories(card_root);
    {
        AudioService audio; SdMusicPlayer player(audio);
        assert(player.DownloadVideo("https://host/video.avi"));
        wait_until([&] { return player.GetVideoFrame() != nullptr; });
        assert(read_file(card_root / "video/test.avi") == video);
        auto before = player.GetVideoFrame();
        assert(before->width == 320 && before->height == 240 && before->stride == 640 && before->pixels.size() == 153600);
        auto snapshot = player.GetSnapshot();
        assert(snapshot.is_video && snapshot.state == SdMusicPlayer::State::kPlaying && snapshot.total == 1);
        assert(snapshot.message == "Silent video" && snapshot.download_percent == 100);
        player.TogglePause();
        auto paused = player.GetVideoFrame();
        const auto paused_state = player.GetSnapshot();
        std::this_thread::sleep_for(180ms);
        assert(player.GetVideoFrame() == paused);
        assert(player.GetSnapshot().video_frames == paused_state.video_frames);
        assert(player.GetSnapshot().elapsed_seconds == paused_state.elapsed_seconds);
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kPaused);
        assert(audio.IsLocalPlaybackActive());
        { std::lock_guard lock(audio.mutex); assert(audio.received.empty()); }
        player.TogglePause();
        wait_until([&] { auto frame = player.GetVideoFrame(); return frame && frame != paused; });
        const auto generation = player.generation_.load();
        wait_until([&] { return player.generation_ > generation; });
        assert(player.GetSnapshot().state == SdMusicPlayer::State::kPlaying);
        player.Stop();
        assert(!player.GetVideoFrame());
        assert(before->pixels.size() == 153600 && paused->pixels.size() == 153600);
        wait_until([&] { return !host_initialized; });
    }
    assert(host_inits == host_deinits && wrong_host_deinits == 0 && duplicate_host_inits == 0);
    std::cout << "PASS: worker downloads to empty card, auto plays/loops, pauses media clock, resumes, retains shared frames safely\n";

    card_root = fixture / "audio-pause";
    write_file(card_root / "video/test.avi", read_file(audio_fixtures / "mono.avi"));
    {
        AudioService audio; SdMusicPlayer player(audio); player.Start();
        wait_until([&] { const auto s = player.GetSnapshot(); return s.video_frames >= 2 && s.video_audio_samples > 0; });
        player.TogglePause(); std::this_thread::sleep_for(20ms);
        const auto snapshot = player.GetSnapshot(); const auto frame = player.GetVideoFrame();
        const uint32_t token = player.token_; const auto played = audio.GetLocalPlaybackSamples(token);
        size_t accepted; { std::lock_guard lock(audio.mutex); accepted = audio.received.size(); }
        std::this_thread::sleep_for(180ms);
        const auto still = player.GetSnapshot();
        assert(still.state == SdMusicPlayer::State::kPaused && still.video_has_audio);
        assert(still.video_frames == snapshot.video_frames && still.video_audio_samples == snapshot.video_audio_samples);
        assert(player.GetVideoFrame() == frame && audio.GetLocalPlaybackSamples(token) == played);
        { std::lock_guard lock(audio.mutex); assert(audio.received.size() == accepted); }
        player.TogglePause();
        wait_until([&] { const auto s = player.GetSnapshot(); return s.video_frames > snapshot.video_frames && s.video_audio_samples > snapshot.video_audio_samples; });
        player.Stop(); assert(!audio.IsLocalPlaybackActive() && !player.GetVideoFrame());
        assert(audio.GetLocalPlaybackSamples(token) == 0);
        wait_until([&] { return !host_initialized; });
    }
    std::cout << "PASS: audio and video pause together, resume together and retire the playback token on Stop\n";

    card_root = fixture / "audio-cancel";
    write_file(card_root / "video/test.avi", read_file(audio_fixtures / "mono.avi"));
    {
        AudioService audio; SdMusicPlayer player(audio);
        std::atomic<bool> entered = false, release = false;
        unsigned pushes = 0;
        audio.push_hook = [&] {
            if (++pushes == 12) { entered = true; while (!release) std::this_thread::sleep_for(1ms); }
        };
        player.Start(); wait_until([&] { return entered.load(); });
        size_t accepted; { std::lock_guard lock(audio.mutex); accepted = audio.received.size(); }
        assert(accepted > 0 && accepted < 6400);
        const auto before = std::chrono::steady_clock::now();
        player.Stop(); assert(std::chrono::steady_clock::now() - before < 200ms);
        assert(!audio.IsLocalPlaybackActive() && !player.GetVideoFrame());
        release = true; wait_until([&] { return !host_initialized; });
        std::lock_guard lock(audio.mutex); assert(audio.received.size() == accepted);
    }
    assert(host_inits == host_deinits && wrong_host_deinits == 0 && duplicate_host_inits == 0);
    std::cout << "PASS: Stop cancels an in-flight PCM submission without appending samples or retaining the SD mount\n";
}
'''


def main():
    harness = music.HARNESS.replace("struct Board {", HTTP + "struct Board { "
                                   "NetworkStub* GetNetwork() { static NetworkStub network; return &network; }")
    audio_start, audio_end = harness.index("struct AudioService {"), harness.index("struct Codec {")
    harness = harness[:audio_start] + AUDIO + harness[audio_end:]
    source = "#include <fcntl.h>\n#include <unistd.h>\n#include <climits>\n" + harness + STUBS
    source += "\n#define private public\n" + music.without_includes(
        (BOARD / "sd_music_player.h").read_text(encoding="utf-8")) + "\n#undef private\n"
    source += music.without_includes((BOARD / "avi_reader.h").read_text(encoding="utf-8"))
    source += "\n" + music.without_includes((BOARD / "sd_video_url.h").read_text(encoding="utf-8"))
    source += "\n#define fopen test_fopen\n#define opendir test_opendir\n"
    source += "#define stat(path, info) test_stat(path, info)\n#define open test_open\n"
    source += "#define mkdir test_mkdir\n#define remove test_remove\n#define rename test_rename\n"
    source += "#define fwrite test_fwrite\n#define fsync test_fsync\n"
    for name in ("sd_music_player.cc", "avi_reader.cc", "sd_music_video.cc"):
        source += "\n" + music.without_includes((BOARD / name).read_text(encoding="utf-8"))
    for name in ("fopen", "opendir", "stat", "open", "mkdir", "remove", "rename", "fwrite", "fsync"):
        source += f"\n#undef {name}\n"
    source += TESTS
    with tempfile.TemporaryDirectory(prefix="sd-video-test-") as name:
        directory = Path(name)
        cpp, binary = directory / "test.cc", directory / "test"
        cpp.write_text(source, encoding="utf-8")
        sample = directory / "sample.avi"
        sample.write_bytes(avi.avi(avi.chunk(b"00dc", avi.jpeg()) * 4,
                                   avi.headers(count=4), avi.chunk(b"JUNK", b"x" * 8192)))
        audio_fixtures = directory / "audio"
        audio_fixtures.mkdir()
        values = (0, 1, -1, 32767, -32768, 12345, -12345)
        mono = b"".join(struct.pack("<h", values[i % len(values)]) for i in range(6400))
        stereo = b"".join(struct.pack("<hh", *((-30000, 10000) if i // 960 % 2 else (30000, -10000)))
                          for i in range(19200))
        for mode in ("mono", "stereo", "unsupported", "missing_pcm", "unaligned_pcm"):
            payload = stereo if mode == "stereo" else mono
            channels, rate, count = (2, 48000, 19200) if mode == "stereo" else (1, 16000, 6400)
            stream = avi.pcm_stream(channels=channels, sample_rate=rate,
                                    count=count + (mode == "missing_pcm"),
                                    format_tag=7 if mode == "unsupported" else 1)
            # An awkward mono boundary exercises joining samples across AVI chunks.
            boundary = 4800 * 4 if channels == 2 else 321 * 2
            pieces = [payload[:boundary], payload[boundary:]]
            if mode == "unaligned_pcm":
                pieces[-1] += b"x"
            frames = avi.chunk(b"00dc", avi.jpeg()) + avi.chunk(b"01wb", pieces[0])
            frames += avi.chunk(b"00dc", avi.jpeg()) * 2 + avi.chunk(b"01wb", pieces[1])
            frames += avi.chunk(b"00dc", avi.jpeg())
            (audio_fixtures / (mode + ".avi")).write_bytes(avi.avi(
                frames, avi.headers(streams=[avi.stream(count=4), stream], count=4)))
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-g", "-pthread", "-fsanitize=address,undefined",
                        str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary), str(directory / "fixture"), str(sample), str(audio_fixtures)],
                       check=True, timeout=30)


if __name__ == "__main__":
    main()

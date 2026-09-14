#!/usr/bin/env python3
"""Exercise production SD video/download code with host HTTP/JPEG hardware stubs.

Run in Linux/WSL using Python 3 and g++; never builds ESP-IDF or accesses a card.
Uses the real AVI parser, temporary files, production worker and cancellation.
JPEG output is simulated; hardware JPEG/display and network need board testing.
"""

import os
from pathlib import Path
import subprocess
import tempfile

import test_avi_reader as avi
import test_sd_music as music

BOARD = music.BOARD

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
    assert(argc == 3);
    const fs::path fixture(argv[1]);
    const std::string video = read_file(argv[2]);
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
}
'''


def main():
    harness = music.HARNESS.replace("struct Board {", HTTP + "struct Board { "
                                   "NetworkStub* GetNetwork() { static NetworkStub network; return &network; }")
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
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-g", "-pthread", "-fsanitize=address,undefined",
                        str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary), str(directory / "fixture"), str(sample)], check=True, timeout=30)


if __name__ == "__main__":
    main()

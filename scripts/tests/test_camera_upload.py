#!/usr/bin/env python3
"""Run the production camera upload methods with a host HTTP/camera harness.

Requires Python 3 and g++; does not configure or build ESP-IDF firmware.
Run: python3 scripts/tests/test_camera_upload.py
"""

from pathlib import Path
import os
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def method(source, signature):
    start = source.index(signature)
    body = source.index("{", start)
    depth = 1
    end = body + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


HARNESS = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
enum {
    PIXFORMAT_RGB565, PIXFORMAT_YUV422, PIXFORMAT_YUV420,
    PIXFORMAT_GRAYSCALE, PIXFORMAT_JPEG, PIXFORMAT_RGB888,
    V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_GREY, V4L2_PIX_FMT_JPEG, V4L2_PIX_FMT_RGB24
};
using v4l2_pix_fmt_t = int;
using TaskHandle_t = void*;
TaskHandle_t current_task = reinterpret_cast<void*>(1);
TaskHandle_t xTaskGetCurrentTaskHandle() { return current_task; }
struct camera_fb_t {
    uint16_t width = 320, height = 240;
    int format = PIXFORMAT_RGB565;
    uint8_t* buf = nullptr;
    size_t len = 0;
};
enum class Failure { none, encode, empty_jpeg, create_http, open,
    short_write, write, terminator, status, read, empty_reply, huge_reply };
Failure failure;
int response_status = 503;
int returned_frames, closed_http, opened_http;
size_t encoded_source_size, max_write_size, response_offset;
std::string uploaded;
void esp_camera_fb_return(camera_fb_t*) { ++returned_frames; }
bool image_to_jpeg(uint8_t*, size_t size, uint16_t, uint16_t,
                   v4l2_pix_fmt_t, int, uint8_t** out, size_t* len) {
    encoded_source_size = size;
    if (failure == Failure::encode) return false;
    *len = failure == Failure::empty_jpeg ? 0 : 10001;
    *out = static_cast<uint8_t*>(malloc(10001));
    memset(*out, 'J', 10001);
    return true;
}
class Http {
public:
    void SetTimeout(int ms) { assert(ms > 0 && ms <= 30000); }
    void SetHeader(const std::string&, const std::string&) {}
    bool Open(const std::string&, const std::string&) {
        assert(returned_frames == 1); // The driver is free before network I/O.
        ++opened_http;
        return failure != Failure::open;
    }
    void Close() { ++closed_http; }
    int Write(const char* data, size_t len) {
        max_write_size = std::max(max_write_size, len);
        if (failure == Failure::write) return -1;
        if (failure == Failure::short_write && len) return len - 1;
        if (failure == Failure::terminator && len == 0) return -1;
        uploaded.append(data, len);
        // The TCP implementation reports chunk framing in its byte count.
        return len + 8;
    }
    int GetStatusCode() { return failure == Failure::status ? response_status : 200; }
    int Read(char* data, size_t cap) {
        if (failure == Failure::read) return -1;
        std::string response = failure == Failure::huge_reply ?
            std::string(65537, 'R') : failure == Failure::empty_reply ? "" : "a dog";
        size_t count = std::min(cap, response.size() - response_offset);
        memcpy(data, response.data() + response_offset, count);
        response_offset += count;
        return count;
    }
};
struct Network {
    std::unique_ptr<Http> CreateHttp(int) {
        return failure == Failure::create_http ? nullptr : std::make_unique<Http>();
    }
};
struct Board {
    static Board& GetInstance() { static Board board; return board; }
    Network* GetNetwork() { static Network network; return &network; }
    std::string GetUuid() { return "test-client"; }
};
struct SystemInfo { static std::string GetMacAddress() { return "test-device"; } };
class Esp32Camera {
public:
    std::string explain_url_ = "https://example.invalid/vision";
    std::string explain_token_;
    std::mutex explain_mutex_;
    std::atomic<TaskHandle_t> owner_task_{nullptr};
    camera_fb_t* current_fb_ = nullptr;
    uint8_t* encode_buf_ = nullptr;
    void ReleaseFrame();
    void ReturnFrame();
    bool TryAcquire();
    std::string Explain(const std::string&);
};
'''


TESTS = r'''
void reset(Failure next) {
    failure = next;
    returned_frames = closed_http = opened_http = 0;
    encoded_source_size = max_write_size = response_offset = 0;
    uploaded.clear();
}
int main() {
    int cases = 0;
    for (auto fault : {Failure::none, Failure::encode, Failure::empty_jpeg,
            Failure::create_http, Failure::open, Failure::short_write,
            Failure::write, Failure::terminator, Failure::status,
            Failure::read, Failure::empty_reply, Failure::huge_reply}) {
        reset(fault);
        Esp32Camera camera;
        uint8_t rgb = 0;
        camera_fb_t fb;
        fb.buf = &rgb;
        fb.len = 320 * 240 * 2;
        camera.current_fb_ = &fb;
        camera.encode_buf_ = &rgb;
        bool threw = false;
        try {
            assert(camera.Explain("what is this?") == "a dog");
        } catch (const std::runtime_error& error) {
            threw = true;
            if (fault == Failure::status) {
                assert(std::string(error.what()) == "Failed to upload photo (HTTP 503)");
            }
        }
        assert(threw == (fault != Failure::none));
        assert(camera.current_fb_ == nullptr && returned_frames == 1);
        camera.ReleaseFrame();
        assert(returned_frames == 1);
        bool has_http = fault != Failure::encode &&
            fault != Failure::empty_jpeg && fault != Failure::create_http;
        assert(closed_http == (has_http ? 1 : 0));
        assert(encoded_source_size == 320 * 240 * 2);
        assert(max_write_size <= 4096);
        if (fault == Failure::none) {
            assert(uploaded.find("what is this?") != std::string::npos);
            assert(uploaded.find(std::string(10001, 'J')) != std::string::npos);
            assert(uploaded.ends_with("--\r\n"));
        }
        ++cases;
    }
    for (int invalid = 0; invalid < 3; ++invalid) {
        reset(Failure::none);
        Esp32Camera camera;
        camera_fb_t fb;
        camera.current_fb_ = invalid == 0 ? nullptr : &fb;
        if (invalid == 1) camera.explain_url_.clear();
        if (invalid == 2) fb.format = -1;
        bool threw = false;
        try { camera.Explain("test"); }
        catch (const std::runtime_error&) { threw = true; }
        assert(threw && opened_http == 0 && closed_http == 0);
        assert(returned_frames == (invalid == 0 ? 0 : 1));
        ++cases;
    }
    for (int status : {-1, 401, 403, 429}) {
        reset(Failure::status);
        response_status = status;
        Esp32Camera camera;
        camera.explain_token_ = "test-secret-value";
        camera_fb_t frame;
        camera.current_fb_ = &frame;
        bool threw = false;
        try { camera.Explain("test"); }
        catch (const std::runtime_error& error) {
            threw = true;
            assert(std::string(error.what()) == "Failed to upload photo (HTTP " + std::to_string(status) + ")");
        }
        assert(threw && closed_http == 1 && returned_frames == 1);
        ++cases;
    }
    std::cout << "Camera upload regression: " << cases << " cases passed\n";
    reset(Failure::none);
    Esp32Camera camera;
    camera_fb_t fb;
    assert(camera.TryAcquire());
    assert(camera.TryAcquire());
    camera.current_fb_ = &fb;
    current_task = reinterpret_cast<void*>(2);
    assert(!camera.TryAcquire());
    camera.ReleaseFrame();
    assert(returned_frames == 0 && camera.current_fb_ == &fb);
    bool busy = false;
    try { camera.Explain("test"); }
    catch (const std::runtime_error&) { busy = true; }
    assert(busy && camera.current_fb_ == &fb);
    current_task = reinterpret_cast<void*>(1);
    camera.ReturnFrame();
    current_task = reinterpret_cast<void*>(2);
    assert(!camera.TryAcquire()); // Returning the driver frame keeps upload ownership.
    current_task = reinterpret_cast<void*>(1);
    camera.ReleaseFrame();
    current_task = reinterpret_cast<void*>(2);
    assert(camera.TryAcquire());
    camera.ReleaseFrame();
    assert(returned_frames == 1);
    std::cout << "Camera ownership regression: competing tasks and release passed\n";
}
'''


def main():
    source = (ROOT / "main/boards/common/esp32_camera.cc").read_text(encoding="utf-8")
    production = "\n".join(method(source, signature) for signature in (
        "bool Esp32Camera::TryAcquire()",
        "void Esp32Camera::ReturnFrame()",
        "void Esp32Camera::ReleaseFrame()",
        "std::string Esp32Camera::Explain("))
    with tempfile.TemporaryDirectory(prefix="camera-upload-test-") as directory:
        directory = Path(directory)
        cpp = directory / "test.cc"
        binary = directory / "test"
        cpp.write_text(HARNESS + production + TESTS, encoding="utf-8")
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra",
                        "-Werror", "-fsanitize=address,undefined", "-g", str(cpp),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    main()

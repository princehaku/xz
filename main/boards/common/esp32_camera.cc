#include "sdkconfig.h"

#include <esp_heap_caps.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <esp_log.h>
#include <img_converters.h>

#include "esp32_camera.h"
#include "board.h"
#include "display.h"
#include "lvgl_display.h"
#include "mcp_server.h"
#include "system_info.h"
#include "jpg/image_to_jpeg.h"
#include "esp_timer.h"

#define TAG "Esp32Camera"

Esp32Camera::Esp32Camera(const camera_config_t &config) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        ESP_LOGI(TAG, "Camera sensor detected, PID=0x%04x", s->id.PID);
        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 0); // Control camera mirror: 1 for mirror, 0 for normal
        }
        ESP_LOGI(TAG, "Camera initialized: format=%d", config.pixel_format);
    } else {
        ESP_LOGE(TAG, "No camera sensor detected after esp_camera_init");
    }

    streaming_on_ = true;
}

Esp32Camera::~Esp32Camera() {
    if (streaming_on_) {
        ReturnFrame();
        if (encode_buf_) {
            heap_caps_free(encode_buf_);
            encode_buf_ = nullptr;
            encode_buf_size_ = 0;
        }
        esp_camera_deinit();
        streaming_on_ = false;
    }
}

void Esp32Camera::SetExplainUrl(const std::string &url, const std::string &token) {
    std::lock_guard<std::mutex> lock(explain_mutex_);
    explain_url_ = url;
    explain_token_ = token;
}

bool Esp32Camera::Capture() {
    return Capture(true);
}

bool Esp32Camera::Capture(bool show_preview) {
    if (!TryAcquire()) {
        ESP_LOGW(TAG, "Camera is busy");
        return false;
    }
    struct CaptureGuard {
        Esp32Camera* camera;
        bool captured = false;
        ~CaptureGuard() { if (!captured) camera->ReleaseFrame(); }
    } guard{this};
    if (!streaming_on_) {
        return false;
    }

    // Get the latest frame, discard old frames for real-time performance
    for (int i = 0; i < 2; i++) {
        ReturnFrame();
        current_fb_ = esp_camera_fb_get();
        if (!current_fb_) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    if (current_fb_->buf == nullptr || current_fb_->len == 0 ||
        current_fb_->width == 0 || current_fb_->height == 0 ||
        current_fb_->width > std::numeric_limits<uint16_t>::max() ||
        current_fb_->height > std::numeric_limits<uint16_t>::max()) {
        ESP_LOGE(TAG, "Invalid camera frame");
        ReleaseFrame();
        return false;
    }

    // Prepare encode buffer for RGB565 format (with optional byte swapping)
    if (current_fb_->format == PIXFORMAT_RGB565) {
        if (current_fb_->height > std::numeric_limits<size_t>::max() / current_fb_->width / 2) {
            ESP_LOGE(TAG, "RGB565 frame size overflow");
            return false;
        }
        size_t pixel_count = current_fb_->width * current_fb_->height;
        size_t data_size = pixel_count * 2;
        if (current_fb_->len < data_size) {
            ESP_LOGE(TAG, "Truncated RGB565 camera frame");
            ReleaseFrame();
            return false;
        }

        // Allocate or reallocate encode buffer if needed
        if (encode_buf_size_ < data_size) {
            if (encode_buf_) {
                heap_caps_free(encode_buf_);
            }
            encode_buf_ = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (encode_buf_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate memory for encode buffer");
                encode_buf_size_ = 0;
                ReleaseFrame();
                return false;
            }
            encode_buf_size_ = data_size;
        }

        // Copy data to encode buffer with optional byte swapping
        uint16_t *src = (uint16_t *)current_fb_->buf;
        uint16_t *dst = (uint16_t *)encode_buf_;
        if (swap_bytes_enabled_) {
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
        } else {
            memcpy(encode_buf_, current_fb_->buf, data_size);
        }

        size_t preview_width = current_fb_->width;
        size_t preview_height = current_fb_->height;
        size_t preview_size = data_size;
        if (preview_rotate_90_enabled_) {
            preview_width = current_fb_->height;
            preview_height = current_fb_->width;
            preview_size = preview_width * preview_height * sizeof(uint16_t);
        }

        // Allocate separate buffer for preview display
        uint8_t *preview_data = show_preview ?
            (uint8_t *)heap_caps_malloc(preview_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : nullptr;
        if (preview_data != nullptr) {
            if (!preview_rotate_90_enabled_) {
                memcpy(preview_data, encode_buf_, data_size);
            } else {
                auto* src16 = reinterpret_cast<const uint16_t*>(encode_buf_);
                auto* dst16 = reinterpret_cast<uint16_t*>(preview_data);
                const int src_w = current_fb_->width;
                const int src_h = current_fb_->height;
                const int dst_w = src_h;
                for (int y = 0; y < src_h; ++y) {
                    for (int x = 0; x < src_w; ++x) {
                        const int src_idx = y * src_w + x;
                        int dst_idx = 0;
                        if (preview_rotate_clockwise_) {
                            dst_idx = x * dst_w + (dst_w - 1 - y);
                        } else {
                            dst_idx = (src_w - 1 - x) * dst_w + y;
                        }
                        dst16[dst_idx] = src16[src_idx];
                    }
                }
            }
            auto display = dynamic_cast<LvglDisplay *>(Board::GetInstance().GetDisplay());
            if (display != nullptr) {
                display->SetPreviewImage(std::make_unique<LvglAllocatedImage>(
                    preview_data,
                    preview_size,
                    preview_width,
                    preview_height,
                    preview_width * 2,
                    LV_COLOR_FORMAT_RGB565));
            } else {
                heap_caps_free(preview_data);
            }
        }
    } else if (current_fb_->format == PIXFORMAT_JPEG) {
        // JPEG format preview usually requires decoding, skip preview display for now, just log
        ESP_LOGW(TAG, "JPEG capture success, len=%zu, but not supported for preview", current_fb_->len);
    }

    ESP_LOGI(TAG, "Captured frame: %dx%d, len=%zu, format=%d",
             current_fb_->width, current_fb_->height, current_fb_->len, current_fb_->format);

    guard.captured = true;
    return true;
}

bool Esp32Camera::TryAcquire() {
    auto task = xTaskGetCurrentTaskHandle();
    TaskHandle_t expected = nullptr;
    return owner_task_.compare_exchange_strong(expected, task) || expected == task;
}

void Esp32Camera::ReturnFrame() {
    if (current_fb_ != nullptr) {
        esp_camera_fb_return(current_fb_);
        current_fb_ = nullptr;
    }
}

void Esp32Camera::ReleaseFrame() {
    if (owner_task_.load() == xTaskGetCurrentTaskHandle()) {
        ReturnFrame();
        owner_task_ = nullptr;
    }
}

bool Esp32Camera::SetHMirror(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_hmirror(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetVFlip(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_vflip(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetSwapBytes(bool enabled) {
    swap_bytes_enabled_ = enabled;
    return true;
}

void Esp32Camera::SetPreviewRotation(bool rotate_90, bool clockwise) {
    preview_rotate_90_enabled_ = rotate_90;
    preview_rotate_clockwise_ = clockwise;
}

std::string Esp32Camera::Explain(const std::string &question) {
    if (!TryAcquire()) {
        throw std::runtime_error("Camera is busy");
    }
    // Every exit returns the single driver buffer, including exceptions.
    struct FrameGuard {
        Esp32Camera* camera;
        ~FrameGuard() { camera->ReleaseFrame(); }
    } frame_guard{this};

    std::string explain_url;
    std::string explain_token;
    {
        std::lock_guard<std::mutex> lock(explain_mutex_);
        explain_url = explain_url_;
        explain_token = explain_token_;
    }
    if (explain_url.empty()) {
        throw std::runtime_error("Image explain URL is not set");
    }
    if (current_fb_ == nullptr) {
        throw std::runtime_error("No camera frame captured");
    }

    const uint16_t width = current_fb_->width;
    const uint16_t height = current_fb_->height;
    v4l2_pix_fmt_t format;
    switch (current_fb_->format) {
        case PIXFORMAT_RGB565: format = V4L2_PIX_FMT_RGB565; break;
        case PIXFORMAT_YUV422: format = V4L2_PIX_FMT_YUYV; break;
        case PIXFORMAT_YUV420: format = V4L2_PIX_FMT_YUV420; break;
        case PIXFORMAT_GRAYSCALE: format = V4L2_PIX_FMT_GREY; break;
        case PIXFORMAT_JPEG: format = V4L2_PIX_FMT_JPEG; break;
        case PIXFORMAT_RGB888: format = V4L2_PIX_FMT_RGB24; break;
        default: throw std::runtime_error("Unsupported camera pixel format");
    }

    uint8_t* source = current_fb_->buf;
    size_t source_size = current_fb_->len;
    if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
        source = encode_buf_;
        source_size = static_cast<size_t>(width) * height * 2;
    }

    // The encoder already produces a complete JPEG. Take ownership directly
    // instead of copying it through an extra thread and a blocking chunk queue.
    uint8_t* jpeg_data = nullptr;
    size_t jpeg_size = 0;
    bool encoded = image_to_jpeg(source, source_size, width, height, format, 60,
                                 &jpeg_data, &jpeg_size);
    std::unique_ptr<uint8_t, decltype(&free)> jpeg(jpeg_data, &free);
    if (!encoded || jpeg == nullptr || jpeg_size == 0) {
        throw std::runtime_error("Failed to encode image to JPEG");
    }
    ReturnFrame();

    auto network = Board::GetInstance().GetNetwork();
    if (network == nullptr) {
        throw std::runtime_error("Network is not available");
    }
    auto http = network->CreateHttp(3);
    if (http == nullptr) {
        throw std::runtime_error("Failed to create image upload connection");
    }
    struct HttpGuard {
        Http* http;
        ~HttpGuard() { http->Close(); }
    } http_guard{http.get()};

    const std::string boundary = "----ESP32_CAMERA_BOUNDARY";
    http->SetTimeout(30000);
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    if (!http->Open("POST", explain_url)) {
        throw std::runtime_error("Failed to connect to explain URL");
    }

    auto write_chunk = [&http](const char* data, size_t size) {
        // HttpClient counts chunk framing bytes; ML307 counts payload bytes.
        // A short write is fatal: retrying a partial chunk would corrupt HTTP.
        int written = http->Write(data, size);
        if (written < 0 || static_cast<size_t>(written) < size) {
            throw std::runtime_error("Image upload was interrupted");
        }
    };
    const std::string question_field =
        "--" + boundary + "\r\n"
        "Content-Disposition: form-data; name=\"question\"\r\n\r\n" +
        question + "\r\n";
    write_chunk(question_field.data(), question_field.size());
    const std::string file_header =
        "--" + boundary + "\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n";
    write_chunk(file_header.data(), file_header.size());

    // Bound transient copies made by the TCP-backed HTTP implementation.
    constexpr size_t kUploadChunkSize = 4096;
    for (size_t offset = 0; offset < jpeg_size;) {
        size_t size = std::min(kUploadChunkSize, jpeg_size - offset);
        write_chunk(reinterpret_cast<const char*>(jpeg.get() + offset), size);
        offset += size;
    }
    jpeg.reset();
    const std::string footer = "\r\n--" + boundary + "--\r\n";
    write_chunk(footer.data(), footer.size());
    write_chunk("", 0);

    if (http->GetStatusCode() != 200) {
        throw std::runtime_error("Failed to upload photo");
    }

    // Recognition replies are text; cap retained data even for an invalid server.
    constexpr size_t kMaxResponseSize = 64 * 1024;
    std::string result;
    char buffer[512];
    while (true) {
        int count = http->Read(buffer, sizeof(buffer));
        if (count < 0) {
            throw std::runtime_error("Failed to read image explanation");
        }
        if (count == 0) break;
        if (result.size() + static_cast<size_t>(count) > kMaxResponseSize) {
            throw std::runtime_error("Image explanation is too large");
        }
        result.append(buffer, count);
    }
    if (result.empty()) {
        throw std::runtime_error("Image explanation is empty");
    }
    ESP_LOGI(TAG, "Explained image: %ux%u, JPEG=%u bytes, response=%u bytes",
             static_cast<unsigned>(width), static_cast<unsigned>(height),
             static_cast<unsigned>(jpeg_size), static_cast<unsigned>(result.size()));
    return result;
}

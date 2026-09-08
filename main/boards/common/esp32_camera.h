#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <memory>
#include <vector>
#include <atomic>
#include <mutex>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "camera.h"
#include "esp_camera.h"
#include "jpg/image_to_jpeg.h"

class Esp32Camera : public Camera
{
private:
    bool streaming_on_ = false;
    bool swap_bytes_enabled_ = true;  // Swap pixel byte order for RGB565, enabled by default
    bool preview_rotate_90_enabled_ = false;
    bool preview_rotate_clockwise_ = true;
    std::string explain_url_;
    std::string explain_token_;
    std::mutex explain_mutex_;
    std::atomic<TaskHandle_t> owner_task_{nullptr};
    camera_fb_t *current_fb_ = nullptr;
    uint8_t *encode_buf_ = nullptr;  // Buffer for JPEG encoding (with optional byte swap)
    size_t encode_buf_size_ = 0;
    void ReturnFrame();

public:
    Esp32Camera(const camera_config_t &config);
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string &url, const std::string &token) override;
    virtual bool Capture() override;
    bool Capture(bool show_preview);
    const camera_fb_t* GetCapturedFrame() const { return current_fb_; }
    // Nonblocking ownership also serializes raw preview access with MCP captures.
    bool TryAcquire();
    // Call from the camera owner task after a capture is no longer needed.
    void ReleaseFrame();
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual bool SetSwapBytes(bool enabled) override;
    void SetPreviewRotation(bool rotate_90, bool clockwise);
    virtual std::string Explain(const std::string &question) override;
};

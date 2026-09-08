#include "dual_network_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"
#include "mcp_server.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "led/single_led.h"
#include "lvgl_theme.h"

#include <atomic>
#include <limits>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <font_awesome.h>
#include <wifi_manager.h>

#define TAG "LichuangDevBoardML307"

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
private:
    Pca9557* pca9557_;

public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557) 
        : BoxAudioCodec(i2c_bus, 
                       AUDIO_INPUT_SAMPLE_RATE, 
                       AUDIO_OUTPUT_SAMPLE_RATE,
                       AUDIO_I2S_GPIO_MCLK, 
                       AUDIO_I2S_GPIO_BCLK, 
                       AUDIO_I2S_GPIO_WS, 
                       AUDIO_I2S_GPIO_DOUT, 
                       AUDIO_I2S_GPIO_DIN,
                       GPIO_NUM_NC, 
                       AUDIO_CODEC_ES8311_ADDR, 
                       AUDIO_CODEC_ES7210_ADDR, 
                       AUDIO_INPUT_REFERENCE),
          pca9557_(pca9557) {
    }

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};

class LichuangDevBoardML307 : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Button boot_button_;
    Display* display_ = nullptr;
    Pca9557* pca9557_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    esp_timer_handle_t memory_snapshot_timer_ = nullptr;
    std::atomic<bool> photo_task_running_{false};
    std::atomic<bool> photo_requested_{false};
    std::atomic<uint32_t> photo_generation_{0};
    TaskHandle_t camera_task_ = nullptr;

    enum class AppMode { kHome, kAiGuide, kAiPhoto };
    std::atomic<AppMode> app_mode_{AppMode::kHome};
    std::atomic<uint32_t> page_generation_{0};
    lv_obj_t* home_overlay_ = nullptr;
    lv_obj_t* preview_canvas_ = nullptr;
    lv_obj_t* photo_hint_ = nullptr;
    std::atomic<bool> preview_running_{false};
    std::atomic<uint32_t> preview_generation_{0};
    int preview_fail_count_ = 0;
    static constexpr int kMaxPreviewFails = 10;  // 连续失败后自动停止

    lv_img_dsc_t preview_img_dsc_ = {};
    uint8_t* preview_image_buf_ = nullptr;
    size_t preview_image_size_ = 0;

    void LogMemorySnapshot(const char* stage) {
        const size_t heap_free = esp_get_free_heap_size();
        const size_t heap_min = esp_get_minimum_free_heap_size();
        const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        ESP_LOGI("MEM", "[%s] heap=%u heap_min=%u int=%u int_largest=%u psram=%u psram_largest=%u",
            stage,
            (unsigned)heap_free, (unsigned)heap_min,
            (unsigned)internal_free, (unsigned)internal_largest,
            (unsigned)psram_free, (unsigned)psram_largest);
    }

    static void MemorySnapshotTimerCallback(void* arg) {
        static_cast<LichuangDevBoardML307*>(arg)->LogMemorySnapshot("periodic_5s");
    }

    void StartMemorySnapshotTimer() {
        if (memory_snapshot_timer_ != nullptr) return;
        const esp_timer_create_args_t timer_args = {
            .callback = &LichuangDevBoardML307::MemorySnapshotTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "mem_probe_5s",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&timer_args, &memory_snapshot_timer_) != ESP_OK) {
            ESP_LOGW(TAG, "Create memory snapshot timer failed");
            return;
        }
        if (esp_timer_start_periodic(memory_snapshot_timer_, 5 * 1000 * 1000) != ESP_OK) {
            ESP_LOGW(TAG, "Start memory snapshot timer failed");
            esp_timer_delete(memory_snapshot_timer_);
            memory_snapshot_timer_ = nullptr;
        }
    }

    // ──────────────────── Home screen LVGL callbacks ────────────────────
    static void HomeScreenGuideClicked(lv_event_t* e) {
        auto* self = static_cast<LichuangDevBoardML307*>(lv_event_get_user_data(e));
        ESP_LOGI(TAG, "[home] Entering CALL mode, ToggleChatState");
        self->StopPreview();
        self->DeleteOverlayLocked();
        self->app_mode_ = AppMode::kAiGuide;
        const uint32_t generation = ++self->page_generation_;
        Application::GetInstance().Schedule([self, generation]() {
            if (self->page_generation_ != generation) return;
            auto& app = Application::GetInstance();
            // Phone mode: auto-answer directly, no wake word needed
            app.SetKeepAlive(true);
#if CONFIG_USE_DEVICE_AEC
            app.SetAecMode(kAecOnDeviceSide);
#endif
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.WakeWordInvoke("你好");
            }
        });
    }

    static void HomeScreenPhotoClicked(lv_event_t* e) {
        auto* self = static_cast<LichuangDevBoardML307*>(lv_event_get_user_data(e));
        ESP_LOGI(TAG, "[home] Entering AI Photo mode");
        self->StopPreview();
        self->DeleteOverlayLocked();
        self->app_mode_ = AppMode::kAiPhoto;
        ++self->page_generation_;
        Application::GetInstance().EndConversation();
        self->ShowPhotoPreview();
    }

    // Call only while holding the LVGL lock (including LVGL event callbacks).
    void DeleteOverlayLocked() {
        preview_canvas_ = nullptr;
        photo_hint_ = nullptr;
        if (home_overlay_) {
            lv_obj_del(home_overlay_);
            home_overlay_ = nullptr;
        }
        heap_caps_free(preview_image_buf_);
        preview_image_buf_ = nullptr;
        preview_image_size_ = 0;
        preview_img_dsc_ = {};
    }

    const lv_font_t* TextFont() {
        auto* theme = dynamic_cast<LvglTheme*>(display_->GetTheme());
        return theme && theme->text_font() ? theme->text_font()->font() : LV_FONT_DEFAULT;
    }

    const lv_font_t* IconFont() {
        auto* theme = dynamic_cast<LvglTheme*>(display_->GetTheme());
        return theme && theme->large_icon_font() ? theme->large_icon_font()->font() : LV_FONT_DEFAULT;
    }

    void SetPhotoHint(const char* text) {
        if (!lvgl_port_lock(100)) return;
        if (app_mode_ == AppMode::kAiPhoto && photo_hint_) {
            lv_label_set_text(photo_hint_, text);
        }
        lvgl_port_unlock();
    }

    void ShowPhotoPreview() {
        if (!lvgl_port_lock(500)) {
            ESP_LOGW(TAG, "LVGL lock timeout, skip photo preview");
            return;
        }
        const int W = LV_HOR_RES;
        const int H = LV_VER_RES;

        home_overlay_ = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(home_overlay_);
        lv_obj_set_size(home_overlay_, W, H);
        lv_obj_set_pos(home_overlay_, 0, 0);
        lv_obj_set_style_bg_color(home_overlay_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(home_overlay_, LV_OPA_COVER, 0);
        lv_obj_clear_flag(home_overlay_, LV_OBJ_FLAG_SCROLLABLE);

        // Full screen camera preview
        preview_canvas_ = lv_image_create(home_overlay_);
        lv_obj_set_size(preview_canvas_, W, H);
        lv_obj_align(preview_canvas_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(preview_canvas_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(preview_canvas_, LV_OPA_COVER, 0);

        // Tap a held still image to resume the live preview.
        lv_obj_add_flag(preview_canvas_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(preview_canvas_, [](lv_event_t* e) {
            auto* self = static_cast<LichuangDevBoardML307*>(lv_event_get_user_data(e));
            if (!self->preview_running_ && !self->photo_task_running_) {
                self->StartPreview();
            }
        }, LV_EVENT_CLICKED, this);

        photo_hint_ = lv_label_create(home_overlay_);
        lv_label_set_text(photo_hint_, "按键拍照 / 双击返回");
        lv_obj_set_width(photo_hint_, W - 16);
        lv_obj_set_style_text_align(photo_hint_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(photo_hint_, lv_color_white(), 0);
        lv_obj_set_style_bg_color(photo_hint_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(photo_hint_, LV_OPA_80, 0);
        lv_obj_set_style_text_font(photo_hint_, TextFont(), 0);
        lv_obj_align(photo_hint_, LV_ALIGN_BOTTOM_MID, 0, -6);

        StartPreview();

        lvgl_port_unlock();
        ESP_LOGI(TAG, "Photo mode: live preview started, press button to capture");
    }

    // A single worker owns both preview capture and photo upload. Camera waits
    // never run on the shared ESP timer task or the application event loop.
    void InitializeCameraWorker() {
        if (xTaskCreate([](void* arg) {
                static_cast<LichuangDevBoardML307*>(arg)->CameraWorker();
            }, "board_camera", 8192, this, 3, &camera_task_) != pdPASS) {
            camera_task_ = nullptr;
            ESP_LOGE(TAG, "Failed to create camera worker");
        }
    }

    void CameraWorker() {
        uint32_t previous_preview = 0;
        while (true) {
            ulTaskNotifyTake(pdTRUE, preview_running_ ? pdMS_TO_TICKS(100) : portMAX_DELAY);
            if (photo_requested_.exchange(false)) {
                RunPhotoTask(photo_generation_.load());
            }
            if (!preview_running_ || photo_task_running_) continue;
            const uint32_t generation = preview_generation_;
            if (generation != previous_preview) {
                previous_preview = generation;
                preview_fail_count_ = 0;
            }
            PreviewFrame(generation);
        }
    }

    void PreviewFailed(uint32_t generation) {
        if (generation != preview_generation_ || !preview_running_) return;
        if (++preview_fail_count_ >= kMaxPreviewFails) {
            StopPreview();
            SetPhotoHint("预览失败，点画面重试 / 双击返回");
        }
    }

    bool ValidPreviewFrame(const camera_fb_t* fb) {
        return fb && fb->buf && fb->format == PIXFORMAT_RGB565 && fb->width > 0 &&
            fb->height > 0 && fb->width <= UINT16_MAX / 2 && fb->height <= UINT16_MAX &&
            fb->height <= std::numeric_limits<size_t>::max() / (fb->width * 2) &&
            fb->len >= fb->width * fb->height * 2;
    }

    // The caller owns the camera frame and holds the LVGL lock.
    bool DisplayPreviewFrameLocked(const camera_fb_t* fb) {
        if (!ValidPreviewFrame(fb) || !preview_canvas_) return false;
        const size_t bytes = fb->width * fb->height * 2;
        if (preview_image_size_ < bytes) {
            auto* replacement = static_cast<uint8_t*>(
                heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (!replacement) return false;
            lv_image_set_src(preview_canvas_, nullptr);
            heap_caps_free(preview_image_buf_);
            preview_image_buf_ = replacement;
            preview_image_size_ = bytes;
        }
        // Update shared pixels while rendering is excluded, including odd pixel counts.
        for (size_t i = 0; i < bytes; i += 2) {
            preview_image_buf_[i] = fb->buf[i + 1];
            preview_image_buf_[i + 1] = fb->buf[i];
        }
        preview_img_dsc_.header.magic = LV_IMAGE_HEADER_MAGIC;
        preview_img_dsc_.header.cf = LV_COLOR_FORMAT_RGB565;
        preview_img_dsc_.header.flags = LV_IMAGE_FLAGS_MODIFIABLE;
        preview_img_dsc_.header.w = fb->width;
        preview_img_dsc_.header.h = fb->height;
        preview_img_dsc_.header.stride = fb->width * 2;
        preview_img_dsc_.data = preview_image_buf_;
        preview_img_dsc_.data_size = bytes;
        lv_image_set_src(preview_canvas_, &preview_img_dsc_);
        lv_obj_invalidate(preview_canvas_);
        return true;
    }

    void PreviewFrame(uint32_t generation) {
        if (!camera_->TryAcquire()) return;
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            camera_->ReleaseFrame();
            PreviewFailed(generation);
            return;
        }
        if (lvgl_port_lock(100)) {
            // Stop/delete can happen while the camera driver waits for a frame.
            if (preview_running_ && generation == preview_generation_ &&
                app_mode_ == AppMode::kAiPhoto && preview_canvas_) {
                if (DisplayPreviewFrameLocked(fb)) {
                    preview_fail_count_ = 0;
                } else {
                    PreviewFailed(generation);
                }
            }
            lvgl_port_unlock();
        }
        esp_camera_fb_return(fb);
        camera_->ReleaseFrame();
    }

    void StartPreview() {
        if (preview_running_ || photo_task_running_ || app_mode_ != AppMode::kAiPhoto) return;
        if (!camera_task_ || !camera_) {
            SetPhotoHint("摄像头不可用 / 双击返回");
            return;
        }
        ++preview_generation_;
        preview_running_ = true;
        SetPhotoHint("按键拍照 / 双击返回");
        xTaskNotifyGive(camera_task_);
    }

    void StopPreview() {
        preview_running_ = false;
        ++preview_generation_;
    }

    // ────────────────────────────────────────────────────────────────────
    void ShowHomeScreen() {
        if (!lvgl_port_lock(500)) {
            ESP_LOGW(TAG, "LVGL lock timeout, skip home screen");
            return;
        }

        const int W = LV_HOR_RES;   // 320
        const int H = LV_VER_RES;   // 240
        const int MID = W / 2;      // 160

        // Full-screen overlay sits on top of whatever the display shows
        home_overlay_ = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(home_overlay_);
        lv_obj_set_size(home_overlay_, W, H);
        lv_obj_set_pos(home_overlay_, 0, 0);
        lv_obj_set_style_bg_color(home_overlay_, lv_color_hex(0x0D1117), 0);
        lv_obj_set_style_bg_opa(home_overlay_, LV_OPA_COVER, 0);
        lv_obj_clear_flag(home_overlay_, LV_OBJ_FLAG_SCROLLABLE);

        // ── Left zone: CALL (phone mode, dark blue) ──
        lv_obj_t* left = lv_obj_create(home_overlay_);
        lv_obj_remove_style_all(left);
        lv_obj_set_size(left, MID - 1, H);
        lv_obj_set_pos(left, 0, 0);
        lv_obj_set_style_bg_color(left, lv_color_hex(0x1A237E), 0);
        lv_obj_set_style_bg_opa(left, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(left, lv_color_hex(0x3949AB), LV_STATE_PRESSED);
        lv_obj_add_flag(left, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(left, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(left, HomeScreenGuideClicked, LV_EVENT_CLICKED, this);

        // Phone icon (large)
        lv_obj_t* phone_icon = lv_label_create(left);
        lv_label_set_text(phone_icon, FONT_AWESOME_PHONE);
        lv_obj_set_style_text_color(phone_icon, lv_color_white(), 0);
        lv_obj_set_style_text_align(phone_icon, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(phone_icon, IconFont(), 0);
        lv_obj_align(phone_icon, LV_ALIGN_TOP_MID, 0, 15);

        // CALL label
        lv_obj_t* l_title = lv_label_create(left);
        lv_label_set_text(l_title, "AI Chat");
        lv_obj_set_style_text_color(l_title, lv_color_white(), 0);
        lv_obj_set_style_text_align(l_title, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(l_title, &lv_font_montserrat_24, 0);
        lv_obj_align(l_title, LV_ALIGN_CENTER, 0, 5);

        // ── Center divider ──
        lv_obj_t* div = lv_obj_create(home_overlay_);
        lv_obj_remove_style_all(div);
        lv_obj_set_size(div, 2, H);
        lv_obj_set_pos(div, MID - 1, 0);
        lv_obj_set_style_bg_color(div, lv_color_hex(0x333355), 0);
        lv_obj_set_style_bg_opa(div, LV_OPA_COVER, 0);

        // ── Right zone: 百科相机 (static, like left side) ──
        lv_obj_t* right = lv_obj_create(home_overlay_);
        lv_obj_remove_style_all(right);
        lv_obj_set_size(right, W - MID - 1, H);
        lv_obj_set_pos(right, MID + 1, 0);
        lv_obj_set_style_bg_color(right, lv_color_hex(0x004D40), 0);
        lv_obj_set_style_bg_opa(right, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(right, lv_color_hex(0x00695C), LV_STATE_PRESSED);
        lv_obj_add_flag(right, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(right, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(right, HomeScreenPhotoClicked, LV_EVENT_CLICKED, this);

        // Camera icon (large)
        lv_obj_t* cam_icon = lv_label_create(right);
        lv_label_set_text(cam_icon, FONT_AWESOME_CAMERA);
        lv_obj_set_style_text_color(cam_icon, lv_color_white(), 0);
        lv_obj_set_style_text_align(cam_icon, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(cam_icon, IconFont(), 0);
        lv_obj_align(cam_icon, LV_ALIGN_TOP_MID, 0, 15);

        // Title label
        lv_obj_t* r_title = lv_label_create(right);
        lv_label_set_text(r_title, "AI Camera");
        lv_obj_set_style_text_color(r_title, lv_color_white(), 0);
        lv_obj_set_style_text_align(r_title, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(r_title, &lv_font_montserrat_24, 0);
        lv_obj_align(r_title, LV_ALIGN_CENTER, 0, 5);

        lvgl_port_unlock();
        ESP_LOGI(TAG, "Home screen shown");
    }

    void CaptureAndExplainPhoto() {
        bool expected = false;
        if (!photo_task_running_.compare_exchange_strong(expected, true)) {
            SetPhotoHint("识图进行中 / 双击返回");
            return;
        }
        if (!camera_task_ || !camera_) {
            photo_task_running_ = false;
            SetPhotoHint("摄像头不可用 / 双击返回");
            return;
        }
        StopPreview();
        const uint32_t generation = page_generation_;
        auto& app = Application::GetInstance();
        app.Schedule([this, generation]() {
            if (generation != page_generation_ || app_mode_ != AppMode::kAiPhoto) {
                photo_task_running_ = false;
                if (app_mode_ == AppMode::kAiPhoto) StartPreview();
                return;
            }
            auto& app = Application::GetInstance();
            app.EndConversation();
            // Queue the worker request after the main-task conversation cleanup.
            app.Schedule([this, generation]() {
                if (generation != page_generation_ || app_mode_ != AppMode::kAiPhoto) {
                    photo_task_running_ = false;
                    if (app_mode_ == AppMode::kAiPhoto) StartPreview();
                    return;
                }
                SetPhotoHint("拍照识图中 / 双击返回");
                photo_generation_ = generation;
                photo_requested_ = true;
                xTaskNotifyGive(camera_task_);
            });
        });
    }

    void RunPhotoTask(uint32_t generation) {
        std::string result;
        std::string error;
        try {
            if (generation == page_generation_ && app_mode_ == AppMode::kAiPhoto) {
                if (!camera_->Capture(false)) {
                    throw std::runtime_error("Camera capture failed");
                }
                if (generation == page_generation_ && app_mode_ == AppMode::kAiPhoto) {
                    if (lvgl_port_lock(100)) {
                        if (generation == page_generation_ && preview_canvas_ && home_overlay_) {
                            DisplayPreviewFrameLocked(camera_->GetCapturedFrame());
                        }
                        lvgl_port_unlock();
                    }
                    result = camera_->Explain(
                        "请识别图片中的主要事物，尽量给出具体品种或名称，用中文简要介绍，控制在100字以内。");
                    if (result.empty()) throw std::runtime_error("Empty image explanation");
                }
            }
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Photo explain failed: %s", e.what());
            error = "拍照识图失败，按键重试 / 双击返回";
        }
        // fb_count is one: give the still frame back before restarting preview.
        camera_->ReleaseFrame();
        Application::GetInstance().Schedule([this, generation, result = std::move(result),
                                               error = std::move(error)]() {
            photo_task_running_ = false;
            if (generation != page_generation_ || app_mode_ != AppMode::kAiPhoto) {
                if (app_mode_ == AppMode::kAiPhoto) StartPreview();
                return;
            }
            if (!error.empty()) {
                StartPreview();
                SetPhotoHint(error.c_str());
                return;
            }
            if (result.empty()) {
                StartPreview();
                return;
            }
            SetPhotoHint((result + "\n点画面继续 / 双击返回").c_str());
            // Keep the spoken answer bounded to avoid a second lengthy response.
            Application::GetInstance().NotifySTT(
                "请用中文简短播报以下识图结果，控制在100字以内：" + result);
        });
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_HEIGHT,
            .y_max = DISPLAY_WIDTH,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = { .reset = 0, .interrupt = 0 },
            .flags = { .swap_xy = 1, .mirror_x = 1, .mirror_y = 0 },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        tp_io_config.scl_speed_hz = 400000;
        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);
        assert(tp);

        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        if (touch_cfg.disp) {
            lvgl_port_add_touch(&touch_cfg);
        }
    }

    void InitializeCamera() {
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer   = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk  = CAMERA_PIN_XCLK;
        config.pin_pclk  = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href  = CAMERA_PIN_HREF;
        // Camera SCCB shares I2C bus with audio codec (port 1, GPIO 1/2).
        // Setting sda=-1 tells camera driver to reuse existing I2C port.
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn  = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = 10000000;  // 10MHz for stable operation
        config.pixel_format  = PIXFORMAT_RGB565;
        config.frame_size    = FRAMESIZE_QVGA;
        config.jpeg_quality  = 12;
        config.fb_count      = 1;
        config.fb_location   = CAMERA_FB_IN_PSRAM;
        config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);

        // Restore persisted image-explain endpoint so it survives reboots between
        // MCP sessions (MCP capabilities will override it each new connection).
        Settings camera_settings("camera", false);
        std::string explain_url = camera_settings.GetString("explain_url");
        std::string explain_token = camera_settings.GetString("explain_token");
        if (explain_url.empty()) {
            explain_url = CAMERA_EXPLAIN_URL_DEFAULT;
            ESP_LOGI(TAG, "Using default camera explain endpoint");
        } else {
            ESP_LOGI(TAG, "Loaded persisted camera explain endpoint");
        }
        camera_->SetExplainUrl(explain_url, explain_token);
    }

    void InitializeButtons() {
        // Single click: behavior depends on current app mode.
        //   kHome     → nothing (mode must be selected via touch)
        //   kAiPhoto  → take photo + explain
        //   kAiGuide  → wake / toggle voice assistant
        // During device startup, enter WiFi config mode instead.
        boot_button_.OnClick([this]() {
            Application::GetInstance().Schedule([this]() {
                auto& app = Application::GetInstance();
                if (GetNetworkType() == NetworkType::WIFI &&
                    app.GetDeviceState() == kDeviceStateStarting) {
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.EnterWifiConfigMode();
                    return;
                }
                switch (app_mode_.load()) {
                    case AppMode::kAiPhoto:
                        ESP_LOGI(TAG, "Button: kAiPhoto state=%d", (int)app.GetDeviceState());
                        if (!preview_running_) {
                            if (!photo_task_running_) {
                                StartPreview();
                            } else {
                                SetPhotoHint("识图进行中 / 双击返回");
                            }
                        } else {
                            CaptureAndExplainPhoto();
                        }
                        break;
                    case AppMode::kAiGuide:
                        ESP_LOGI(TAG, "Button: kAiGuide state=%d", (int)app.GetDeviceState());
                        app.ToggleChatState();
                        break;
                    case AppMode::kHome:
                    default:
                        ESP_LOGW(TAG, "Button: kHome - mode not selected yet");
                        if (display_) display_->ShowNotification("请先选择左侧或右侧功能区");
                        break;
                }
            });
        });

        // Double click: during startup switch network type; otherwise go back to home screen.
        boot_button_.OnDoubleClick([this]() {
            // Invalidate a pending HTTP result immediately, before queued UI work.
            ++page_generation_;
            StopPreview();
            Application::GetInstance().Schedule([this]() {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting ||
                    app.GetDeviceState() == kDeviceStateWifiConfiguring) {
                    SwitchNetworkType();
                    return;
                }
                // Return to home screen (re-select mode)
                app_mode_ = AppMode::kHome;
                app.EndConversation();
                if (lvgl_port_lock(500)) {
                    DeleteOverlayLocked();
                    ShowHomeScreen();
                    lvgl_port_unlock();
                }
            });
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnLongPress([this]() {
            Application::GetInstance().Schedule([]() {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateIdle) {
                    app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
                }
            });
        });
#endif
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                if (GetNetworkType() != NetworkType::WIFI) {
                    throw std::runtime_error("Switch to WiFi before reconfiguring WiFi");
                }
                auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                wifi_board.EnterWifiConfigMode();
                return true;
            });
    }

public:
    LichuangDevBoardML307() :
        DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, GPIO_NUM_NC),
        boot_button_(BOOT_BUTTON_GPIO) {

        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeTouch();
        InitializeCamera();
        InitializeCameraWorker();
        InitializeButtons();
        InitializeTools();
        GetBacklight()->RestoreBrightness();
        StartMemorySnapshotTimer();
        ShowHomeScreen();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(i2c_bus_, pca9557_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }
};

DECLARE_BOARD(LichuangDevBoardML307);

#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "esp32_camera.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <exception>
#include <esp_adc/adc_oneshot.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardS3Cam"

class CompactWifiBoardS3Cam : public WifiBoard {
private:
 
    Button boot_button_;
    Button change_photo_button_;
    Button take_photo_button_;
    Button change_photo_set_button_;
    LcdDisplay* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    adc_oneshot_unit_handle_t battery_adc_handle_ = nullptr;
    bool battery_adc_ready_ = false;
    esp_timer_handle_t memory_snapshot_timer_ = nullptr;
    int photo_mode_index_ = 0;
    int photo_set_index_ = 0;
    bool photo_task_running_ = false;

    static const char* DeviceStateToString(DeviceState state) {
        switch (state) {
            case kDeviceStateUnknown: return "unknown";
            case kDeviceStateStarting: return "starting";
            case kDeviceStateWifiConfiguring: return "wifi_configuring";
            case kDeviceStateIdle: return "idle";
            case kDeviceStateConnecting: return "connecting";
            case kDeviceStateListening: return "listening";
            case kDeviceStateSpeaking: return "speaking";
            case kDeviceStateUpgrading: return "upgrading";
            case kDeviceStateActivating: return "activating";
            default: return "other";
        }
    }

    void LogMemorySnapshot(const char* stage) {
        const size_t heap_free = esp_get_free_heap_size();
        const size_t heap_min = esp_get_minimum_free_heap_size();
        const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        const DeviceState state = Application::GetInstance().GetDeviceState();
        ESP_LOGI("MEM", "[%s] state=%s heap=%u heap_min=%u int=%u int_largest=%u psram=%u psram_largest=%u",
            stage,
            DeviceStateToString(state),
            (unsigned)heap_free,
            (unsigned)heap_min,
            (unsigned)internal_free,
            (unsigned)internal_largest,
            (unsigned)psram_free,
            (unsigned)psram_largest);
    }

    static void MemorySnapshotTimerCallback(void* arg) {
        auto* board = static_cast<CompactWifiBoardS3Cam*>(arg);
        if (board == nullptr) {
            return;
        }
        board->LogMemorySnapshot("periodic_5s");
    }

    void StartMemorySnapshotTimer() {
        if (memory_snapshot_timer_ != nullptr) {
            return;
        }
        const esp_timer_create_args_t timer_args = {
            .callback = &CompactWifiBoardS3Cam::MemorySnapshotTimerCallback,
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
            return;
        }
        LogMemorySnapshot("timer_started");
    }

    const char* GetPhotoModeName() const {
        static const char* kModeNames[] = {"百科识图"};
        constexpr size_t kModeCount = sizeof(kModeNames) / sizeof(kModeNames[0]);
        return kModeNames[photo_mode_index_ % kModeCount];
    }

    std::string BuildPhotoQuestion() const {
        static const char* kModeQuestions[] = {
            "请识别这张图片中的主要内容，并用中文+中文百科的方式简要说明。请尽可能之别这张图片中的关键物体。"
        };
        static const char* kSetHints[] = {
            "回答尽量简短，给出关键细节，控制在100字以内。"
        };
        constexpr size_t kModeCount = sizeof(kModeQuestions) / sizeof(kModeQuestions[0]);
        constexpr size_t kSetCount = sizeof(kSetHints) / sizeof(kSetHints[0]);
        return std::string(kModeQuestions[photo_mode_index_ % kModeCount]) + " " + kSetHints[photo_set_index_ % kSetCount];
    }

    void CaptureAndExplainPhoto() {
        LogMemorySnapshot("photo_enter");
        if (photo_task_running_) {
            if (display_) {
                display_->ShowNotification("拍照任务进行中");
            }
            return;
        }
        photo_task_running_ = true;

        auto& app = Application::GetInstance();
        app.Schedule([this]() {
            auto reset_busy = [this]() {
                photo_task_running_ = false;
            };
            auto& app = Application::GetInstance();
            DeviceState state_before_photo = app.GetDeviceState();

            // Quiesce voice pipeline before camera upload to avoid AFE/UDP contention.
            // StopListening() posts an event; here we are already in scheduled main-task code,
            // so force transition to idle directly to stop voice processing immediately.
            if (state_before_photo == kDeviceStateSpeaking) {
                app.AbortSpeaking(kAbortReasonNone);
                vTaskDelay(pdMS_TO_TICKS(80));
            }
            if (state_before_photo == kDeviceStateListening ||
                state_before_photo == kDeviceStateConnecting ||
                state_before_photo == kDeviceStateSpeaking) {
                app.SetDeviceState(kDeviceStateIdle);
                for (int i = 0; i < 10; ++i) {
                    if (app.GetDeviceState() == kDeviceStateIdle) {
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                LogMemorySnapshot("photo_after_quiesce");
            }

            if (camera_ == nullptr) {
                ESP_LOGE(TAG, "Camera is not initialized");
                if (display_) {
                    display_->ShowNotification("摄像头未初始化");
                }
                reset_busy();
                return;
            }

            if (display_) {
                display_->ShowNotification("拍照中...");
            }

            if (!camera_->Capture()) {
                ESP_LOGE(TAG, "Camera capture failed");
                if (display_) {
                    display_->ShowNotification("拍照失败");
                }
                reset_busy();
                return;
            }

            try {
                LogMemorySnapshot("photo_before_explain");
                std::string question = BuildPhotoQuestion();
                std::string result = camera_->Explain(question);
                ESP_LOGI(TAG, "Photo explain result: %s", result.c_str());

                if (display_) {
                    display_->ShowNotification("识图完成");
                    // display_->SetChatMessage("assistant", result.c_str());
                }

                // Do not use WakeWordInvoke for long text prompts; protocol detects wake words only.
                // Keep result on screen and let user ask follow-up naturally.
                if (display_) {
                    display_->SetChatMessage("assistant", result.c_str());
                }
                LogMemorySnapshot("photo_after_explain");
            } catch (const std::exception& e) {
                ESP_LOGE(TAG, "Photo explain failed: %s", e.what());
                if (display_) {
                    display_->ShowNotification("后端识图失败");
                }
            }

            // Restore listening state after photo pipeline completes.
            if (state_before_photo == kDeviceStateListening) {
                app.StartListening();
                LogMemorySnapshot("photo_after_resume_listening");
            }

            LogMemorySnapshot("photo_exit");
            reset_busy();
        });
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        // Keep framebuffer small to avoid DRAM allocation failure on this board.
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(CAMERA_HMIRROR);
        camera_->SetVFlip(CAMERA_VFLIP);
        camera_->SetPreviewRotation(CAMERA_PREVIEW_ROTATE_90, CAMERA_PREVIEW_ROTATE_CW);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        change_photo_button_.OnClick([this]() {
            photo_mode_index_++;
            ESP_LOGI(TAG, "Photo mode switched to: %s", GetPhotoModeName());
            if (display_) {
                display_->ShowNotification(std::string("拍照模式: ") + GetPhotoModeName());
            }
        });

        change_photo_set_button_.OnClick([this]() {
            photo_set_index_++;
            ESP_LOGI(TAG, "Photo set switched to: %d", photo_set_index_);
            if (display_) {
                display_->ShowNotification(photo_set_index_ == 0 ? "回答风格: 简短" : "回答风格: 详细");
            }
        });

        take_photo_button_.OnClick([this]() {
            CaptureAndExplainPhoto();
        });
    }

    void InitializeBatteryMonitor() {
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id = POWER_ADC_UNIT;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        if (adc_oneshot_new_unit(&unit_cfg, &battery_adc_handle_) != ESP_OK) {
            ESP_LOGW(TAG, "Battery ADC unit init failed");
            return;
        }
        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.atten = ADC_ATTEN_DB_12;
        chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (adc_oneshot_config_channel(battery_adc_handle_, POWER_ADC_CHANNEL, &chan_cfg) != ESP_OK) {
            ESP_LOGW(TAG, "Battery ADC channel config failed");
            adc_oneshot_del_unit(battery_adc_handle_);
            battery_adc_handle_ = nullptr;
            return;
        }
        battery_adc_ready_ = true;
    }

public:
    CompactWifiBoardS3Cam() :
        boot_button_(BOOT_BUTTON_GPIO),
        change_photo_button_(CHANGE_PHTOT_GPIO),
        take_photo_button_(TAKE_PHOTO),
        change_photo_set_button_(CHANGE_PHOTO_SET) {
        LogMemorySnapshot("boot_enter");
        InitializeSpi();
        LogMemorySnapshot("after_spi");
        InitializeLcdDisplay();
        display_->SetPreviewHold(true);
        LogMemorySnapshot("after_display");
        InitializeBatteryMonitor();
        LogMemorySnapshot("after_battery");
        InitializeButtons();
        LogMemorySnapshot("after_buttons");
        InitializeCamera();
        LogMemorySnapshot("after_camera");
        StartMemorySnapshotTimer();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        if (!battery_adc_ready_ || battery_adc_handle_ == nullptr) {
            return false;
        }
        int sum = 0;
        constexpr int kSamples = 8;
        for (int i = 0; i < kSamples; ++i) {
            int raw = 0;
            if (adc_oneshot_read(battery_adc_handle_, POWER_ADC_CHANNEL, &raw) != ESP_OK) {
                return false;
            }
            sum += raw;
        }
        int avg = sum / kSamples;
        // Empirical raw range for 1:1 divider and Li-ion battery on ESP32-S3 ADC.
        constexpr int kRawEmpty = 1700;
        constexpr int kRawFull = 2500;
        int pct = (avg - kRawEmpty) * 100 / (kRawFull - kRawEmpty);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        level = pct;
        charging = false;
        discharging = true;
        return true;
    }
};

DECLARE_BOARD(CompactWifiBoardS3Cam);

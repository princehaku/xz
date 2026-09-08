#pragma once

#include <functional>
#include <memory>
#include <lvgl.h>
#include "lvgl_font.h"
#include "sd_music_player.h"

// All screen methods and destruction run with the LVGL lock held.
class SdMusicScreen {
public:
    struct Actions {
        std::function<void()> back;
        std::function<void()> previous;
        std::function<void()> toggle;
        std::function<void()> next;
        std::function<void()> rescan;
        std::function<void(int)> volume;
        std::function<int()> get_volume;
    };

    SdMusicScreen(lv_obj_t* parent, SdMusicPlayer& player, Actions actions);
    ~SdMusicScreen();
    void SetFont(std::shared_ptr<LvglFont> font);

private:
    SdMusicPlayer& player_;
    Actions actions_;
    std::shared_ptr<LvglFont> font_;
    lv_timer_t* timer_ = nullptr;
    lv_obj_t *back_ = nullptr, *rescan_ = nullptr;
    lv_obj_t *title_ = nullptr, *status_ = nullptr, *counter_ = nullptr, *volume_ = nullptr;
    lv_obj_t *previous_ = nullptr, *toggle_ = nullptr, *next_ = nullptr;
    SdMusicPlayer::Snapshot last_{};
    int last_volume_ = -1;
    bool initialized_ = false;

    lv_obj_t* Button(lv_obj_t* parent, int x, int y, int w, int h,
                     const char* text, lv_event_cb_t callback);
    void SetText(lv_obj_t* label, const char* text, const char* fallback);
    void Update(bool force = false);
};

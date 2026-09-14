#include "sd_music_screen.h"

#include <cstdio>
#include <utility>
#include <src/misc/lv_text_private.h>
#include <src/misc/cache/instance/lv_image_cache.h>

namespace {
bool SupportsText(const lv_font_t* font, const char* text) {
    if (!font) return false;
    uint32_t offset = 0;
    while (text[offset]) {
        const auto letter = lv_text_encoded_next(text, &offset);
        if (letter == '\n' || letter == '\r') continue;
        lv_font_glyph_dsc_t glyph{};
        if (!letter || !lv_font_get_glyph_dsc(font, &glyph, letter, 0) || glyph.is_placeholder) return false;
    }
    return true;
}

SdMusicScreen* Self(lv_event_t* event) {
    return static_cast<SdMusicScreen*>(lv_event_get_user_data(event));
}
}  // namespace

lv_obj_t* SdMusicScreen::Button(lv_obj_t* parent, int x, int y, int w, int h,
                              const char* text, lv_event_cb_t callback) {
    auto* button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x253047), 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x405678), LV_STATE_PRESSED);
    lv_obj_set_style_radius(button, 8, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_pad_all(button, 2, 0);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, this);
    auto* label = lv_label_create(button);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    return button;
}

SdMusicScreen::SdMusicScreen(lv_obj_t* parent, SdMusicPlayer& player, Actions actions)
    : player_(player), actions_(std::move(actions)), parent_(parent) {
    lv_obj_set_style_bg_color(parent, lv_color_hex(0x101722), 0);
    lv_obj_set_style_text_color(parent, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_text_font(parent, &lv_font_montserrat_14, 0);

    back_ = Button(parent, 8, 6, 64, 32, "Back", [](lv_event_t* e) { Self(e)->actions_.back(); });
    rescan_ = Button(parent, 248, 6, 64, 32, "Rescan", [](lv_event_t* e) { Self(e)->actions_.rescan(); });
    Button(parent, 80, 6, 160, 32, "Download AVI", [](lv_event_t* e) { Self(e)->ShowDownloadHelp(); });

    title_ = lv_label_create(parent);
    lv_obj_set_pos(title_, 16, 47);
    lv_obj_set_width(title_, 288);
    lv_label_set_long_mode(title_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(title_, LV_TEXT_ALIGN_CENTER, 0);

    status_ = lv_label_create(parent);
    lv_obj_set_pos(status_, 16, 76);
    lv_obj_set_width(status_, 288);
    lv_label_set_long_mode(status_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(status_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(status_, lv_color_hex(0x9DB1C9), 0);

    counter_ = lv_label_create(parent);
    lv_obj_set_pos(counter_, 16, 108);
    lv_obj_set_width(counter_, 288);
    lv_obj_set_style_text_align(counter_, LV_TEXT_ALIGN_CENTER, 0);

    previous_ = Button(parent, 8, 133, 96, 44, "Previous", [](lv_event_t* e) { Self(e)->actions_.previous(); });
    toggle_ = Button(parent, 112, 133, 96, 44, "Play", [](lv_event_t* e) { Self(e)->actions_.toggle(); });
    next_ = Button(parent, 216, 133, 96, 44, "Next", [](lv_event_t* e) { Self(e)->actions_.next(); });
    lv_obj_set_style_bg_color(toggle_, lv_color_hex(0x167663), 0);

    Button(parent, 8, 193, 48, 38, "-", [](lv_event_t* e) { Self(e)->actions_.volume(-5); });
    Button(parent, 264, 193, 48, 38, "+", [](lv_event_t* e) { Self(e)->actions_.volume(5); });
    volume_ = lv_label_create(parent);
    lv_obj_set_pos(volume_, 64, 202);
    lv_obj_set_width(volume_, 192);
    lv_obj_set_style_text_align(volume_, LV_TEXT_ALIGN_CENTER, 0);

    video_overlay_ = lv_obj_create(parent);
    lv_obj_remove_style_all(video_overlay_);
    lv_obj_set_size(video_overlay_, 320, 240);
    lv_obj_set_style_bg_color(video_overlay_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(video_overlay_, LV_OPA_COVER, 0);
    lv_obj_remove_flag(video_overlay_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(video_overlay_, LV_OBJ_FLAG_HIDDEN);
    video_image_ = lv_image_create(video_overlay_);
    lv_obj_center(video_image_);
    video_hint_ = lv_label_create(video_overlay_);
    lv_label_set_text(video_hint_, "AVI / silent");
    lv_obj_set_pos(video_hint_, 6, 4);
    lv_obj_set_style_bg_color(video_hint_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(video_hint_, LV_OPA_70, 0);
    Button(video_overlay_, 4, 202, 64, 34, "Back", [](lv_event_t* e) { Self(e)->actions_.back(); });
    Button(video_overlay_, 72, 202, 76, 34, "Previous", [](lv_event_t* e) { Self(e)->actions_.previous(); });
    video_toggle_ = Button(video_overlay_, 152, 202, 76, 34, "Pause", [](lv_event_t* e) { Self(e)->actions_.toggle(); });
    Button(video_overlay_, 232, 202, 84, 34, "Next", [](lv_event_t* e) { Self(e)->actions_.next(); });

    timer_ = lv_timer_create([](lv_timer_t* timer) {
        static_cast<SdMusicScreen*>(lv_timer_get_user_data(timer))->Update();
    }, 33, this);
    Update(true);
}

SdMusicScreen::~SdMusicScreen() {
    if (timer_) lv_timer_delete(timer_);
    lv_image_cache_drop(&video_dsc_);
    // The owning board deletes the label tree before this object's font owner.
}

void SdMusicScreen::ShowDownloadHelp() {
    if (help_overlay_) return;
    help_overlay_ = lv_obj_create(parent_);
    lv_obj_remove_style_all(help_overlay_);
    lv_obj_set_size(help_overlay_, 320, 240);
    lv_obj_set_style_bg_color(help_overlay_, lv_color_hex(0x101722), 0);
    lv_obj_set_style_bg_opa(help_overlay_, LV_OPA_COVER, 0);
    lv_obj_remove_flag(help_overlay_, LV_OBJ_FLAG_SCROLLABLE);
    auto* title = lv_label_create(help_overlay_);
    lv_label_set_text(title, "Download AVI");
    lv_obj_set_pos(title, 16, 17);
    Button(help_overlay_, 224, 6, 88, 32, "Close", [](lv_event_t* e) {
        auto* self = Self(e);
        auto* overlay = self->help_overlay_;
        self->help_overlay_ = nullptr;
        lv_obj_delete(overlay);
    });
    const auto address = actions_.portal_address ? actions_.portal_address() : std::string();
    const std::string text = address.empty()
        ? "Connect to Wi-Fi first.\nThen open this page on your phone."
        : "Open on a phone or PC\non the same Wi-Fi:\n" + address + "\n\nEnter the AVI download URL.\nMJPEG, up to 320 x 240.\nAudio: 16-bit PCM, 8-48 kHz.";
    auto* label = lv_label_create(help_overlay_);
    lv_obj_set_pos(label, 16, 52);
    lv_obj_set_width(label, 288);
    lv_label_set_text(label, text.c_str());
}

void SdMusicScreen::UpdateVideo(const SdMusicPlayer::Snapshot& snapshot) {
    auto frame = player_.GetVideoFrame();
    const bool show = snapshot.is_video && frame &&
        (snapshot.state == SdMusicPlayer::State::kPlaying || snapshot.state == SdMusicPlayer::State::kPaused);
    if (!show) {
        lv_obj_add_flag(video_overlay_, LV_OBJ_FLAG_HIDDEN);
        if (video_frame_) {
            lv_image_set_src(video_image_, nullptr);
            lv_image_cache_drop(&video_dsc_);
            video_frame_.reset();
        }
        return;
    }
    if (frame != video_frame_) {
        lv_image_cache_drop(&video_dsc_);
        // Keep the old pixels alive until LVGL has switched to the next descriptor.
        auto previous = std::move(video_frame_);
        video_frame_ = std::move(frame);
        video_dsc_ = {};
        video_dsc_.header.magic = LV_IMAGE_HEADER_MAGIC;
        video_dsc_.header.cf = LV_COLOR_FORMAT_RGB565;
        video_dsc_.header.w = video_frame_->width;
        video_dsc_.header.h = video_frame_->height;
        video_dsc_.header.stride = video_frame_->stride;
        video_dsc_.data_size = video_frame_->pixels.size();
        video_dsc_.data = video_frame_->pixels.data();
        lv_image_set_src(video_image_, &video_dsc_);
        lv_obj_center(video_image_);
        lv_obj_invalidate(video_image_);
    }
    lv_label_set_text(lv_obj_get_child(video_toggle_, 0),
        snapshot.state == SdMusicPlayer::State::kPaused ? "Play" : "Pause");
    if (lv_obj_has_flag(video_overlay_, LV_OBJ_FLAG_HIDDEN) ||
        snapshot.video_has_audio != last_.video_has_audio) {
        lv_label_set_text(video_hint_, snapshot.video_has_audio ? "AVI / PCM audio" : "AVI / silent");
    }
    lv_obj_remove_flag(video_overlay_, LV_OBJ_FLAG_HIDDEN);
}

void SdMusicScreen::SetFont(std::shared_ptr<LvglFont> font) {
    if (font == font_) return;
    auto previous = std::move(font_);
    font_ = std::move(font);
    Update(true);
}

void SdMusicScreen::SetText(lv_obj_t* label, const char* text, const char* fallback) {
    const auto* font = font_ ? font_->font() : nullptr;
    const bool supported = SupportsText(font, text);
    lv_obj_set_style_text_font(label, supported ? font : &lv_font_montserrat_14, 0);
    lv_label_set_text(label, supported ? text : fallback);
}

void SdMusicScreen::Update(bool force) {
    const auto snapshot = player_.GetSnapshot();
    const int volume = actions_.get_volume();
    force = force || !initialized_;
    UpdateVideo(snapshot);
    if (force || snapshot.title != last_.title || snapshot.index != last_.index) {
        char fallback[32];
        snprintf(fallback, sizeof(fallback), "Track %u", static_cast<unsigned>(snapshot.index + 1));
        SetText(title_, snapshot.title.empty() ? "SD 卡音乐" : snapshot.title.c_str(),
                snapshot.title.empty() ? "MP3 / PCM WAV / MJPEG AVI" : fallback);
    }
    if (force || snapshot.state != last_.state || snapshot.message != last_.message ||
        snapshot.download_percent != last_.download_percent) {
        using State = SdMusicPlayer::State;
        const char* text = "正在准备";
        const char* fallback = "Preparing...";
        switch (snapshot.state) {
            case State::kScanning: text = "正在读取 SD 卡"; fallback = "Reading SD card..."; break;
            case State::kDownloading: text = "正在下载视频到 SD 卡"; fallback = "Downloading AVI to SD..."; break;
            case State::kPlaying: text = "播放中"; fallback = "Playing"; break;
            case State::kPaused: text = "已暂停"; fallback = "Paused"; break;
            case State::kNoCard: text = "请插入 SD 卡后点重扫"; fallback = "Insert SD card, then Rescan"; break;
            case State::kEmpty: text = "请放入音乐或下载 AVI 视频"; fallback = "Add music or download an AVI"; break;
            case State::kError:
                text = snapshot.message.empty() ? "播放或下载失败" : snapshot.message.c_str();
                fallback = snapshot.message.empty() ? "Playback or download failed" : snapshot.message.c_str();
                break;
            case State::kStopped: break;
        }
        SetText(status_, text, fallback);
        if (snapshot.state == State::kDownloading) {
            char progress[48];
            snprintf(progress, sizeof(progress), "Downloading AVI... %d%%", snapshot.download_percent);
            lv_label_set_text(status_, progress);
        }
        SetText(lv_obj_get_child(toggle_, 0), snapshot.state == State::kPlaying ? "暂停" : "播放",
                snapshot.state == State::kPlaying ? "Pause" : "Play");
        const bool playable = snapshot.state == State::kPlaying || snapshot.state == State::kPaused;
        for (auto* button : {previous_, toggle_, next_}) {
            if (playable) lv_obj_remove_state(button, LV_STATE_DISABLED);
            else lv_obj_add_state(button, LV_STATE_DISABLED);
        }
    }
    if (force || snapshot.elapsed_seconds != last_.elapsed_seconds ||
        snapshot.index != last_.index || snapshot.total != last_.total) {
        char text[64];
        snprintf(text, sizeof(text), "%u / %u     %02u:%02u",
                 snapshot.total ? static_cast<unsigned>(snapshot.index + 1) : 0,
                 static_cast<unsigned>(snapshot.total),
                 static_cast<unsigned>(snapshot.elapsed_seconds / 60),
                 static_cast<unsigned>(snapshot.elapsed_seconds % 60));
        lv_label_set_text(counter_, text);
    }
    if (force || volume != last_volume_) {
        char text[40], fallback[40];
        snprintf(text, sizeof(text), "音量 %d%%", volume);
        snprintf(fallback, sizeof(fallback), "Volume %d%%", volume);
        SetText(volume_, text, fallback);
    }
    if (force) {
        SetText(lv_obj_get_child(back_, 0), "返回", "Back");
        SetText(lv_obj_get_child(rescan_, 0), "重扫", "Rescan");
        SetText(lv_obj_get_child(previous_, 0), "上一首", "Previous");
        SetText(lv_obj_get_child(next_, 0), "下一首", "Next");
    }
    last_ = snapshot;
    last_volume_ = volume;
    initialized_ = true;
}

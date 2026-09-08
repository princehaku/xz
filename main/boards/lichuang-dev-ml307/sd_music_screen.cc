#include "sd_music_screen.h"

#include <cstdio>
#include <utility>
#include <src/misc/lv_text_private.h>

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
    : player_(player), actions_(std::move(actions)) {
    lv_obj_set_style_bg_color(parent, lv_color_hex(0x101722), 0);
    lv_obj_set_style_text_color(parent, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_text_font(parent, &lv_font_montserrat_14, 0);

    back_ = Button(parent, 8, 6, 64, 32, "Back", [](lv_event_t* e) { Self(e)->actions_.back(); });
    rescan_ = Button(parent, 248, 6, 64, 32, "Rescan", [](lv_event_t* e) { Self(e)->actions_.rescan(); });
    auto* heading = lv_label_create(parent);
    lv_label_set_text(heading, "SD Music");
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 14);

    title_ = lv_label_create(parent);
    lv_obj_set_pos(title_, 16, 47);
    lv_obj_set_width(title_, 288);
    lv_label_set_long_mode(title_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(title_, LV_TEXT_ALIGN_CENTER, 0);

    status_ = lv_label_create(parent);
    lv_obj_set_pos(status_, 16, 76);
    lv_obj_set_width(status_, 288);
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

    timer_ = lv_timer_create([](lv_timer_t* timer) {
        static_cast<SdMusicScreen*>(lv_timer_get_user_data(timer))->Update();
    }, 250, this);
    Update(true);
}

SdMusicScreen::~SdMusicScreen() {
    if (timer_) lv_timer_delete(timer_);
    // The owning board deletes the label tree before this object's font owner.
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
    if (force || snapshot.title != last_.title || snapshot.index != last_.index) {
        char fallback[32];
        snprintf(fallback, sizeof(fallback), "Track %u", static_cast<unsigned>(snapshot.index + 1));
        SetText(title_, snapshot.title.empty() ? "SD 卡音乐" : snapshot.title.c_str(),
                snapshot.title.empty() ? "MP3 / PCM WAV" : fallback);
    }
    if (force || snapshot.state != last_.state || snapshot.message != last_.message) {
        using State = SdMusicPlayer::State;
        const char* text = "正在准备";
        const char* fallback = "Preparing...";
        switch (snapshot.state) {
            case State::kScanning: text = "正在读取 SD 卡"; fallback = "Reading SD card..."; break;
            case State::kPlaying: text = "播放中"; fallback = "Playing"; break;
            case State::kPaused: text = "已暂停"; fallback = "Paused"; break;
            case State::kNoCard: text = "请插入 SD 卡后点重扫"; fallback = "Insert SD card, then Rescan"; break;
            case State::kEmpty: text = "请在 music 中放入音乐"; fallback = "Add MP3/WAV to /music, then Rescan"; break;
            case State::kError: text = "读取失败，请检查音乐文件"; fallback = "Cannot play. Check files / Rescan"; break;
            case State::kStopped: break;
        }
        SetText(status_, text, fallback);
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

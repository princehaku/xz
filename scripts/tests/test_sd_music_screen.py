#!/usr/bin/env python3
"""Run the production SD music screen against real host LVGL under sanitizers.

Requires Python 3, gcc/g++ (WSL on Windows). The temporary host build exercises
layout, button actions, player-state updates, glyph fallback, font ownership,
and timer teardown. It never configures firmware or accesses the SD card.
"""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

from test_camera_font import definition


ROOT = Path(__file__).resolve().parents[2]
LVGL = ROOT / "managed_components/lvgl__lvgl"
BOARD = ROOT / "main/boards/lichuang-dev-ml307"


HARNESS = r'''
#include "sd_music_screen.h"
#include "src/misc/lv_timer_private.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <utility>
#include <vector>
LV_FONT_DECLARE(font_noto_basic_20_4);
static lv_obj_t* live_root;
static int flushes = 0;

static lv_obj_t* Find(lv_obj_t* root, const char* text) {
    if (lv_obj_check_type(root, &lv_label_class) &&
        std::string(lv_label_get_text(root)) == text) return root;
    for (uint32_t i = 0; i < lv_obj_get_child_count(root); ++i)
        if (auto* result = Find(lv_obj_get_child(root, i), text)) return result;
    return nullptr;
}
static void AssertFontReleased(lv_obj_t* root, const lv_font_t* font) {
    assert(lv_obj_get_style_text_font(root, LV_PART_MAIN) != font);
    for (uint32_t i = 0; i < lv_obj_get_child_count(root); ++i)
        AssertFontReleased(lv_obj_get_child(root, i), font);
}
static void AssertBounds(lv_obj_t* root) {
    for (uint32_t i = 0; i < lv_obj_get_child_count(root); ++i) {
        auto* child = lv_obj_get_child(root, i);
        lv_area_t a;
        lv_obj_get_coords(child, &a);
        if (!(a.x1 >= 0 && a.y1 >= 0 && a.x2 < 320 && a.y2 < 240)) {
            std::cerr << "Out of bounds: " << a.x1 << ',' << a.y1 << "-"
                      << a.x2 << ',' << a.y2 << '\n';
            assert(false);
        }
        for (uint32_t j = i + 1; j < lv_obj_get_child_count(root); ++j) {
            lv_area_t b;
            lv_obj_get_coords(lv_obj_get_child(root, j), &b);
            if (!(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)) {
                std::cerr << "Overlapping sibling rectangles at " << i << ',' << j << '\n';
                assert(false);
            }
        }
        AssertBounds(child);
    }
}
static void Tick() {
    lv_tick_inc(300);
    lv_timer_handler();
    lv_obj_update_layout(lv_screen_active());
    lv_refr_now(nullptr);
}
static unsigned TimerCount() {
    unsigned count = 0;
    for (auto* timer = lv_timer_get_next(nullptr); timer; timer = lv_timer_get_next(timer)) ++count;
    return count;
}
static void Click(lv_obj_t* root, const char* text, const char* localized = nullptr) {
    auto* label = Find(root, text);
    if (!label && localized) label = Find(root, localized);
    if (!label) std::cerr << "Button label absent: " << text << '\n';
    assert(label);
    auto* button = lv_obj_get_parent(label);
    assert(!lv_obj_has_state(button, LV_STATE_DISABLED));
    lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);
}

// A dynamic font with deterministic Unicode coverage. Rendering delegates to
// real LVGL font bitmaps; non-ASCII glyphs draw '?' while retaining CJK advances.
struct OwnedFont : LvglFont {
    lv_font_t value = font_noto_basic_20_4;
    int* destroyed;
    explicit OwnedFont(int& counter) : destroyed(&counter) {
        value.get_glyph_dsc = [](const lv_font_t*, lv_font_glyph_dsc_t* out,
                                uint32_t letter, uint32_t next) {
            const bool found = lv_font_get_glyph_dsc(&font_noto_basic_20_4, out,
                                                     letter < 128 ? letter : '?', next);
            if (letter >= 128) out->adv_w = 20;
            return found;
        };
    }
    const lv_font_t* font() const override { return &value; }
    ~OwnedFont() override {
        if (live_root) AssertFontReleased(live_root, &value);
        ++*destroyed;
    }
};

static int lock_depth = 0;
static bool lvgl_port_lock(uint32_t) { ++lock_depth; return true; }
static void lvgl_port_unlock() { assert(lock_depth > 0); --lock_depth; }
static void heap_caps_free(void* p) { free(p); }
enum DeviceState { kDeviceStateIdle, kDeviceStateConnecting, kDeviceStateListening,
                   kDeviceStateSpeaking, kDeviceStateWifiConfiguring };
struct AudioService {
    bool wake = false;
    void EnableWakeWordDetection(bool value) { wake = value; }
};
struct Application {
    std::deque<std::function<void()>> queue;
    AudioService audio;
    int cleanups = 0;
    DeviceState state = kDeviceStateIdle;
    bool keep_alive = false;
    static Application& GetInstance() { static Application value; return value; }
    void Schedule(std::function<void()> fn) { queue.push_back(std::move(fn)); }
    void EndConversation() { Schedule([this] { ++cleanups; }); }
    DeviceState GetDeviceState() const { return state; }
    void SetKeepAlive(bool value) { keep_alive = value; }
    AudioService& GetAudioService() { return audio; }
    void Drain() { while (!queue.empty()) { auto fn = std::move(queue.front()); queue.pop_front(); fn(); } }
};
struct Display {
    int notifications = 0;
    void ShowNotification(const char* text, int) {
        assert(std::string(text) == "BOOT: SD Music");
        ++notifications;
    }
};
struct AudioCodec {
    int volume = 50;
    int output_volume() const { return volume; }
    void SetOutputVolume(int value) { volume = value; }
};
enum class NetworkEvent { WifiConfigModeEnter, Connected };
using NetworkEventCallback = std::function<void(NetworkEvent, const std::string&)>;
struct WifiBoard {
    NetworkEventCallback network_callback;
    virtual ~WifiBoard() = default;
    virtual void SetNetworkEventCallback(NetworkEventCallback callback) {
        network_callback = std::move(callback);
    }
};
'''


BOARD_STUB = r'''
class LichuangDevBoardML307 : public WifiBoard {
public:
    enum class AppMode { kHome, kAiGuide, kAiPhoto, kMusic };
    std::atomic<AppMode> app_mode_{AppMode::kHome};
    std::atomic<uint32_t> page_generation_{0};
    std::unique_ptr<SdMusicPlayer> music_player_ = std::make_unique<SdMusicPlayer>();
    std::unique_ptr<SdMusicScreen> music_screen_;
    lv_obj_t *home_overlay_ = nullptr, *preview_canvas_ = nullptr, *photo_hint_ = nullptr;
    std::shared_ptr<LvglFont> photo_font_;
    std::string photo_hint_text_, photo_hint_fallback_;
    uint8_t* preview_image_buf_ = nullptr;
    size_t preview_image_size_ = 0;
    lv_img_dsc_t preview_img_dsc_ = {};
    Display display;
    Display* display_ = &display;
    AudioCodec codec;
    int homes = 0;
    AudioCodec* GetAudioCodec() { return &codec; }
    void StopPreview() {}
    void ShowHomeScreen() { ++homes; }
    void ShowMusicScreenLocked() { home_overlay_ = lv_obj_create(lv_layer_top()); }
'''


TESTS = r'''
int main() {
    std::cout << std::unitbuf;
    lv_init();
    auto* display = lv_display_create(320, 240);
    static uint8_t buffer[320 * 40 * 4];
    lv_display_set_color_format(display, LV_COLOR_FORMAT_XRGB8888);
    lv_display_set_buffers(display, buffer, nullptr, sizeof(buffer), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, [](lv_display_t* d, const lv_area_t*, uint8_t*) {
        ++flushes;
        lv_display_flush_ready(d);
    });
    Tick();
    const auto initial_timers = TimerCount();
    auto* root = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, 320, 240);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    live_root = root;
    SdMusicPlayer player;
    int back = 0, previous = 0, next = 0, rescans = 0, toggles = 0, volume = 55;
    SdMusicScreen::Actions actions;
    actions.back = [&] { ++back; };
    actions.previous = [&] { ++previous; };
    actions.next = [&] { ++next; };
    actions.rescan = [&] { ++rescans; };
    actions.toggle = [&] { ++toggles; };
    actions.volume = [&](int delta) { volume += delta; };
    actions.get_volume = [&] { return volume; };
    auto screen = std::make_unique<SdMusicScreen>(root, player, std::move(actions));
    screen->SetFont(std::make_shared<LvglBuiltInFont>(&font_noto_basic_20_4));
    Tick();
    assert(Find(root, "MP3 / PCM WAV") && Find(root, "Preparing..."));
    for (const auto state : {SdMusicPlayer::State::kNoCard, SdMusicPlayer::State::kEmpty,
                             SdMusicPlayer::State::kError, SdMusicPlayer::State::kScanning}) {
        player.snapshot.state = state;
        Tick();
        auto* play = Find(root, "Play");
        assert(play && lv_obj_has_state(lv_obj_get_parent(play), LV_STATE_DISABLED));
        AssertBounds(root);
    }
    std::cout << "PASS: real 320x240 LVGL layout; no-card/empty/error/scanning controls\n";

    player.snapshot = {SdMusicPlayer::State::kPlaying, "Moonlight", 1, 3, 65, "Playing"};
    Tick();
    assert(Find(root, "Moonlight") && (Find(root, "Playing") || Find(root, "播放中")) && Find(root, "2 / 3     01:05"));
    Click(root, "Pause", "暂停"); Click(root, "Previous", "上一首"); Click(root, "Next", "下一首");
    Click(root, "Rescan", "重扫"); Click(root, "+"); Click(root, "-");
    assert(toggles == 1 && previous == 1 && next == 1 && rescans == 1 && volume == 55);
    player.snapshot.state = SdMusicPlayer::State::kPaused;
    Tick();
    assert(Find(root, "Paused") || Find(root, "已暂停")); Click(root, "Play", "播放");
    assert(toggles == 2);
    player.snapshot.title = "中文歌曲";
    Tick();
    assert(Find(root, "Track 2"));
    std::cout << "PASS: pause/resume and all controls; English titles and missing-glyph fallback\n";

    int destroyed_a = 0, destroyed_b = 0;
    auto a = std::make_shared<OwnedFont>(destroyed_a);
    screen->SetFont(a);
    a.reset();
    Tick();
    assert(Find(root, "中文歌曲") && Find(root, "已暂停") && Find(root, "音量 55%"));
    AssertBounds(root);
    auto b = std::make_shared<OwnedFont>(destroyed_b);
    screen->SetFont(b);
    assert(destroyed_a == 1);
    b.reset();
    for (auto state : {SdMusicPlayer::State::kNoCard, SdMusicPlayer::State::kEmpty,
                       SdMusicPlayer::State::kError, SdMusicPlayer::State::kScanning}) {
        player.snapshot.state = state;
        Tick();
        AssertBounds(root);
    }
    Click(root, "返回"); assert(back == 1);
    std::cout << "PASS: dynamic CJK font updates live labels and retains old fonts until replacement\n";

    // Execute the production board teardown; the screen's dynamic font must
    // survive LVGL's synchronous delete events for every label in the tree.
    LichuangDevBoardML307 board;
    board.home_overlay_ = root;
    board.music_screen_ = std::move(screen);
    lv_obj_add_event_cb(root, [](lv_event_t*) { live_root = nullptr; }, LV_EVENT_DELETE, nullptr);
    board.DeleteOverlayLocked();
    assert(destroyed_b == 1 && board.music_screen_ == nullptr && board.home_overlay_ == nullptr);
    Tick(); Tick();
    assert(TimerCount() == initial_timers);
    assert(flushes > 0);
    std::cout << "PASS: production overlay teardown removes timers and preserves font lifetime\n";

    // Exercise production entry/return generation checks with a queued cleanup.
    auto* trigger = lv_obj_create(lv_layer_top());
    lv_obj_add_event_cb(trigger, LichuangDevBoardML307::HomeScreenMusicClicked, LV_EVENT_CLICKED, &board);
    lv_obj_send_event(trigger, LV_EVENT_CLICKED, nullptr);
    assert(board.app_mode_ == LichuangDevBoardML307::AppMode::kMusic);
    assert(board.music_player_->starts == 0);
    board.ReturnHome();
    Application::GetInstance().Drain();
    assert(board.music_player_->starts == 0 && board.music_player_->stops == 1);
    assert(Application::GetInstance().audio.wake && board.homes == 1);
    lv_obj_send_event(trigger, LV_EVENT_CLICKED, nullptr);
    Application::GetInstance().Drain();
    assert(board.music_player_->starts == 1);
    board.ReturnHome(); Application::GetInstance().Drain();
    auto& app = Application::GetInstance();

    // A pending Back click invalidates Start before the application queue runs.
    board.EnterMusic();
    auto bound = board.BoundActions();
    bound.back();
    app.Drain();
    assert(board.music_player_->starts == 1);

    board.EnterMusic(); app.Drain();
    bound.volume(5);
    board.ReturnHome(); app.Drain();
    assert(board.codec.volume == 50); // Old-page volume action was cancelled.

    board.SetNetworkEventCallback([&](NetworkEvent event, const std::string&) {
        if (event == NetworkEvent::WifiConfigModeEnter) app.state = kDeviceStateWifiConfiguring;
    });
    board.EnterMusic();
    const int stops_before_config = board.music_player_->stops;
    board.network_callback(NetworkEvent::WifiConfigModeEnter, "");
    app.Drain();
    assert(board.app_mode_ == LichuangDevBoardML307::AppMode::kMusic);
    assert(board.home_overlay_ && board.music_player_->stops == stops_before_config);
    const int homes_before_config_return = board.homes;
    app.audio.wake = false;
    board.ReturnHome(); app.Drain();
    assert(board.homes == homes_before_config_return && board.home_overlay_ == nullptr);
    assert(board.display.notifications == 1 && !app.audio.wake);

    // A queued provisioning-page cleanup must preserve a newer offline music page.
    board.network_callback(NetworkEvent::WifiConfigModeEnter, "");
    board.EnterMusic();
    app.Drain();
    assert(board.home_overlay_ && board.app_mode_ == LichuangDevBoardML307::AppMode::kMusic);
    board.ReturnHome(); app.Drain();
    assert(lock_depth == 0);
    lv_obj_delete(trigger);
    lv_display_delete(display);
    lv_deinit();
    std::cout << "PASS: queued Start/Back/volume generation cancellation and offline provisioning lifecycle\n";
}
'''


def main():
    player = (BOARD / "sd_music_player.h").read_text(encoding="utf-8")
    state = re.search(r"enum class State\s*\{[^}]+\};", player).group()
    snapshot = re.search(r"struct Snapshot\s*\{[\s\S]*?\n    \};", player).group()
    source = (BOARD / "lichuang_dev_board_ml307.cc").read_text(encoding="utf-8")
    methods = "\n".join(definition(source, name) for name in (
        "DeleteOverlayLocked", "HomeScreenMusicClicked", "EnterMusic", "ReturnHome",
        "SetNetworkEventCallback"))
    actions = definition(source, "ShowMusicScreenLocked")
    back = actions[actions.index("actions.back ="):actions.index("actions.previous =")]
    volume = actions[actions.index("actions.volume ="):actions.index("actions.get_volume =")]
    methods += ("\nSdMusicScreen::Actions BoundActions() { SdMusicScreen::Actions actions;\n"
                + back + volume + "\nreturn actions; }\n")
    with tempfile.TemporaryDirectory(prefix="sd-music-screen-") as temporary:
        directory = Path(temporary)
        (directory / "lv_conf.h").write_text("""
#define LV_CONF_H
#define LV_COLOR_DEPTH 32
#define LV_USE_LOG 0
#define LV_USE_OS LV_OS_NONE
#define LV_USE_STDLIB_MALLOC LV_STDLIB_CLIB
#define LV_USE_STDLIB_STRING LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_CLIB
#define LV_USE_FONT_PLACEHOLDER 0
#define LV_FONT_FMT_TXT_LARGE 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14
#define LV_USE_DEMO_WIDGETS 0
""", encoding="utf-8")
        (directory / "sd_music_player.h").write_text(
            "#pragma once\n#include <string>\n#include <cstdint>\n"
            "class SdMusicPlayer { public:\n" + state + "\n" + snapshot +
            "\nSnapshot snapshot; int starts = 0, stops = 0;\n"
            "Snapshot GetSnapshot() const { return snapshot; }\n"
            "void Start() { ++starts; } void Stop() { ++stops; }\n};\n", encoding="utf-8")
        for name in ("sd_music_screen.h", "sd_music_screen.cc"):
            shutil.copyfile(BOARD / name, directory / name)
        harness = directory / "test.cc"
        harness.write_text(HARNESS + BOARD_STUB + methods + "\n};\n" + TESTS, encoding="utf-8")
        common = ["-O0", "-g", "-fsanitize=address,undefined", "-DLV_CONF_INCLUDE_SIMPLE",
                  "-DLV_LVGL_H_INCLUDE_SIMPLE", "-I" + str(directory), "-I" + str(LVGL),
                  "-I" + str(ROOT / "main/display/lvgl_display")]
        cxx_objects = []
        for path in (harness, directory / "sd_music_screen.cc"):
            obj = path.with_suffix(".o")
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra",
                            "-Werror", *common, "-c", str(path), "-o", str(obj)], check=True)
            cxx_objects.append(str(obj))
        sources = sorted((LVGL / "src").rglob("*.c"))
        sources.append(ROOT / "managed_components/78__xiaozhi-fonts/src/font_noto_basic_20_4.c")
        # Cache the unmodified LVGL host library by its exact input contents;
        # screen/board production code is always rebuilt above on every run.
        digest = hashlib.sha256(b"lvgl-host-screen-v1:-O0:-g:address,undefined:c11")
        digest.update(subprocess.check_output([os.environ.get("CC", "gcc"), "--version"]))
        digest.update((directory / "lv_conf.h").read_bytes())
        for path in sorted(set(sources) | set(LVGL.rglob("*.h"))):
            digest.update(str(path).encode())
            digest.update(path.read_bytes())
        cache = Path(tempfile.gettempdir()) / "codex-lvgl-screen-test" / digest.hexdigest()
        cache.mkdir(parents=True, exist_ok=True)
        print(f"Preparing {len(sources)} cached LVGL host C units...", flush=True)

        def compile_one(item):
            index, path = item
            obj = cache / f"unit-{index}.o"
            if obj.exists():
                return str(obj)
            pending = directory / f"unit-{index}.o"
            subprocess.run([os.environ.get("CC", "gcc"), "-std=c11", *common,
                            "-c", str(path), "-o", str(pending)], check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            pending.replace(obj)
            return str(obj)

        with ThreadPoolExecutor(max_workers=6) as pool:
            objects = list(pool.map(compile_one, enumerate(sources)))
        binary = directory / "test"
        subprocess.run([os.environ.get("CXX", "g++"), *common, *cxx_objects,
                        *objects, "-lm", "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        if error.stderr:
            print(error.stderr.decode(errors="replace"), file=sys.stderr)
        print(f"Host test command failed (exit {error.returncode}).", file=sys.stderr)
        sys.exit(1)

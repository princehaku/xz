#!/usr/bin/env python3
"""Exercise camera hint updates with production methods and real LVGL font data.

Requires Python 3, gcc and g++ (run inside WSL on Windows). Only a temporary
host executable is built; this does not configure or build ESP-IDF firmware.
"""

from pathlib import Path
import os
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
LVGL = ROOT / "managed_components/lvgl__lvgl"
BOARD = ROOT / "main/boards/lichuang-dev-ml307/lichuang_dev_board_ml307.cc"


def definition(source, name):
    """Extract a real definition, skipping declarations and braces in strings."""
    for match in re.finditer(r"\b" + re.escape(name) + r"\s*\(", source):
        start = source.rfind("\n", 0, match.start()) + 1
        brace = source.find("{", match.end())
        if brace < 0 or ";" in source[match.end():brace]:
            continue
        depth = 0
        for token in re.finditer(
            r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]',
            source[brace:],
        ):
            if token.group() == "{":
                depth += 1
            elif token.group() == "}":
                depth -= 1
                if depth == 0:
                    return source[start:brace + token.end()]
    raise AssertionError(f"Missing production definition: {name}")


def font_runtime():
    fmt = (LVGL / "src/font/lv_font_fmt_txt.c").read_text(encoding="utf-8")
    font = (LVGL / "src/font/lv_font.c").read_text(encoding="utf-8")
    text = (LVGL / "src/misc/lv_text.c").read_text(encoding="utf-8")
    utils = (LVGL / "src/misc/lv_utils.c").read_text(encoding="utf-8")
    pieces = [r'''
#include "lvgl.h"
#include "src/misc/lv_text_private.h"
#include "src/misc/lv_utils.h"
#include <string.h>
typedef struct { uint32_t gid_left, gid_right; } kern_pair_ref_t;
static uint32_t get_glyph_dsc_id(const lv_font_t*, uint32_t);
static int8_t get_kern_value(const lv_font_t*, uint32_t, uint32_t);
static int unicode_list_compare(const void*, const void*);
static int kern_pair_8_compare(const void*, const void*);
static int kern_pair_16_compare(const void*, const void*);
/* Bitmap drawing is outside this font-selection/lifetime regression. */
const void * lv_font_get_bitmap_fmt_txt(lv_font_glyph_dsc_t *g, lv_draw_buf_t *b) {
    (void)g; (void)b; return NULL;
}
void lv_memset(void *dst, uint8_t v, size_t n) { memset(dst, v, n); }
''']
    pieces.extend(re.findall(r"^#define LV_IS_[^\n]+", text, re.MULTILINE))
    for name in ("get_glyph_dsc_id", "get_kern_value", "unicode_list_compare",
                 "kern_pair_8_compare", "kern_pair_16_compare",
                 "lv_font_get_glyph_dsc_fmt_txt"):
        pieces.append(definition(fmt, name))
    pieces.append(definition(utils, "lv_utils_bsearch"))
    pieces.append(definition(font, "lv_font_get_glyph_dsc"))
    pieces.append(definition(text, "lv_text_utf8_next"))
    pieces.append("uint32_t (*const lv_text_encoded_next)(const char *, uint32_t *) = lv_text_utf8_next;")
    return "\n".join(pieces)


HARNESS = r'''
#include "lvgl.h"
#include "src/misc/lv_text_private.h"
#include "lvgl_font.h"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
LV_FONT_DECLARE(font_noto_basic_20_4);
#define ESP_LOGI(...) ((void)0)
static int lock_depth, label_updates, style_updates;
static bool lock_available = true;
struct Label {
    const lv_font_t* font = &lv_font_montserrat_14;
    std::string text;
    bool alive = true;
};
static Label* visible_label;
static std::set<const lv_font_t*> retired_fonts;
static lv_obj_t* as_obj(Label& label) { return reinterpret_cast<lv_obj_t*>(&label); }
static Label& label_of(lv_obj_t* obj) {
    auto& label = *reinterpret_cast<Label*>(obj);
    assert(label.alive);
    assert(!retired_fonts.contains(label.font));
    return label;
}
extern "C" void lv_obj_set_style_text_font(lv_obj_t* obj, const lv_font_t* font,
                                            lv_style_selector_t) {
    assert(lock_depth > 0);
    auto& label = label_of(obj); // Old font must survive the style replacement.
    assert(font && !retired_fonts.contains(font));
    label.font = font;
    ++style_updates;
}
extern "C" void lv_label_set_text(lv_obj_t* obj, const char* text) {
    assert(lock_depth > 0);
    auto& label = label_of(obj);
    label.text = text;
    ++label_updates;
}
extern "C" void lv_obj_delete(lv_obj_t*) {
    assert(lock_depth > 0);
    if (visible_label) {
        assert(!retired_fonts.contains(visible_label->font));
        visible_label->alive = false;
        visible_label = nullptr;
    }
}
bool lvgl_port_lock(uint32_t) {
    if (!lock_available) return false;
    ++lock_depth;
    return true;
}
void lvgl_port_unlock() { assert(lock_depth > 0); --lock_depth; }
void heap_caps_free(void* ptr) { free(ptr); }
struct Theme { virtual ~Theme() = default; };
struct LvglTheme : Theme {
    std::shared_ptr<LvglFont> text;
    mutable int font_reads = 0;
    std::shared_ptr<LvglFont> text_font() const { ++font_reads; return text; }
};
struct SpiLcdDisplay {
    Theme* theme = nullptr;
    virtual ~SpiLcdDisplay() = default;
    virtual void SetTheme(Theme* value) { assert(lock_depth > 0); theme = value; }
    Theme* GetTheme() const { return theme; }
};
struct DisplayLockGuard {
    explicit DisplayLockGuard(SpiLcdDisplay*) { ++lock_depth; }
    ~DisplayLockGuard() { --lock_depth; }
};
'''


TESTS = r'''
static std::vector<uint32_t> queried_codepoints;
struct OwnedFont : LvglFont {
    lv_font_t value = font_noto_basic_20_4;
    int* destroyed;
    explicit OwnedFont(int& counter) : destroyed(&counter) {
        value.get_glyph_dsc = [](const lv_font_t*, lv_font_glyph_dsc_t* out,
                                 uint32_t codepoint, uint32_t) {
            out->is_placeholder = false;
            out->adv_w = 20;
            queried_codepoints.push_back(codepoint);
            return codepoint != 0;
        };
        retired_fonts.erase(&value);
    }
    const lv_font_t* font() const override { return &value; }
    ~OwnedFont() override {
        assert(!visible_label || !visible_label->alive || visible_label->font != &value);
        retired_fonts.insert(&value);
        ++*destroyed;
    }
};

int main() {
    BoardLcdDisplay display;
    LvglTheme boot;
    boot.text = std::make_shared<LvglBuiltInFont>(&font_noto_basic_20_4);
    LichuangDevBoardML307 board;
    board.display_ = &display;
    display.on_theme_changed = [&] { board.RefreshPhotoFontLocked(); };
    display.SetTheme(&boot); // Theme updates must be safe before a camera page exists.
    assert(!board.photo_font_ && label_updates == 0);

    assert(board.FontSupportsText(&font_noto_basic_20_4, "Ready / Exit\nNext"));
    assert(!board.FontSupportsText(&font_noto_basic_20_4, "按键拍照 / 双击返回"));
    assert(!board.FontSupportsText(&font_noto_basic_20_4, "\xff"));
    assert(!board.FontSupportsText(&font_noto_basic_20_4, "A\xe4\xb8"));
    assert(!board.FontSupportsText(&font_noto_basic_20_4, "A\xf0\x9f\x98"));
    assert(!board.FontSupportsText(nullptr, "Ready"));
    for (const auto* fallback : production_fallbacks) {
        assert(board.FontSupportsText(&lv_font_montserrat_14, fallback));
    }
    std::cout << "PASS: real boot font lacks camera hint glyphs; ASCII/newlines work\n";

    int destroyed_a = 0, destroyed_b = 0;
    auto full_a = std::make_shared<OwnedFont>(destroyed_a);
    queried_codepoints.clear();
    assert(board.FontSupportsText(full_a->font(), "é中😀"));
    assert((queried_codepoints == std::vector<uint32_t>{0xe9, 0x4e2d, 0x1f600}));
    boot.text = full_a; // Assets has changed the theme, but has not applied it yet.
    const int reads_before_page = boot.font_reads;
    Label hint;
    visible_label = &hint;
    board.photo_hint_ = as_obj(hint);
    board.home_overlay_ = as_obj(hint);
    board.app_mode_ = LichuangDevBoardML307::AppMode::kAiPhoto;
    assert(lvgl_port_lock(100));
    board.RefreshPhotoFontLocked();
    lvgl_port_unlock();
    assert(boot.font_reads == reads_before_page); // Page creation uses only the applied snapshot.
    assert(board.photo_font_->font() == &font_noto_basic_20_4);
    board.SetPhotoHint("按键拍照 / 双击返回", "Press to capture / Double-click to exit");
    assert(hint.text == "Press to capture / Double-click to exit");
    assert(hint.font == &lv_font_montserrat_14);
    board.SetPhotoHint("识图进行中 / 双击返回", "Recognizing... / Double-click to exit");
    std::cout << "PASS: page creation keeps the old applied snapshot while asset changes are pending\n";

    display.SetTheme(&boot);
    assert(board.photo_hint_ == as_obj(hint));
    assert(hint.text == "识图进行中 / 双击返回");
    assert(hint.font == full_a->font());
    std::cout << "PASS: asset font updates the existing page and preserves its busy hint\n";

    const std::string result = "这是一只柴犬。\n点画面继续 / 双击返回";
    board.SetPhotoHint(result.c_str(), "Done. Tap to continue / Double-click to exit");
    assert(hint.text == result);
    const int writes_before_repeat = label_updates;
    display.SetTheme(&boot);
    assert(label_updates == writes_before_repeat); // No unnecessary repeated layout work.

    full_a.reset();
    auto full_b = std::make_shared<OwnedFont>(destroyed_b);
    LvglTheme next_theme;
    next_theme.text = full_b;
    boot.text.reset(); // Only the page and applied display snapshot retain the old font.
    assert(destroyed_a == 0);
    display.SetTheme(&next_theme);
    assert(destroyed_a == 1 && hint.font == full_b->font());
    assert(hint.text == result && board.photo_hint_text_ == result);
    boot.text = std::make_shared<LvglBuiltInFont>(&font_noto_basic_20_4);
    display.SetTheme(&boot);
    assert(hint.text == "Done. Tap to continue / Double-click to exit");
    assert(board.photo_hint_text_ == result);
    display.SetTheme(&next_theme);
    assert(hint.text == result && hint.font == full_b->font());
    std::cout << "PASS: theme replacement retains old font until labels change; result is preserved\n";

    board.app_mode_ = LichuangDevBoardML307::AppMode::kHome;
    board.SetPhotoHint("按键拍照 / 双击返回", "wrong page");
    assert(hint.text == result);
    board.app_mode_ = LichuangDevBoardML307::AppMode::kAiPhoto;
    lock_available = false;
    board.SetPhotoHint("按键拍照 / 双击返回", "lock failed");
    lock_available = true;
    assert(hint.text == result && lock_depth == 0);

    full_b.reset();
    next_theme.text.reset();
    assert(destroyed_b == 0);
    assert(lvgl_port_lock(100));
    board.DeleteOverlayLocked();
    lvgl_port_unlock();
    assert(destroyed_b == 0 && !hint.alive && !board.photo_font_); // Display still owns its snapshot.
    assert(board.photo_hint_text_.empty() && board.photo_hint_fallback_.empty());
    const int writes_after_close = label_updates;
    display.SetTheme(&boot);
    assert(destroyed_b == 1 && label_updates == writes_after_close && lock_depth == 0);
    std::cout << "PASS: closing page destroys labels before fonts; later theme updates do nothing\n";
}
'''


def main():
    source = BOARD.read_text(encoding="utf-8")
    fallbacks = set(re.findall(
        r'SetPhotoHint\([\s\S]*?,\s*("(?:\\.|[^"\\])*")\s*\)', source))
    fallbacks.update(re.findall(r'photo_hint_fallback_\s*=\s*("(?:\\.|[^"\\])*")', source))
    assert len(fallbacks) >= 5, "The production fixed camera hints must be included"
    assert all(text.isascii() for text in fallbacks), "Fallback hints must stay ASCII"
    production_fallbacks = "\nconst char* production_fallbacks[] = {\n" + ",\n".join(sorted(fallbacks)) + "\n};\n"
    display_start = source.index("class BoardLcdDisplay")
    display_end = source.index("\n};", display_start) + 3
    production_display = source[display_start:display_end]
    production_methods = "\n".join(definition(source, name) for name in (
        "FontSupportsText", "UpdatePhotoHintLocked", "RefreshPhotoFontLocked",
        "SetPhotoHint", "DeleteOverlayLocked"))
    board_stub = r'''
class LichuangDevBoardML307 {
public:
    enum class AppMode { kHome, kAiPhoto };
    std::atomic<AppMode> app_mode_{AppMode::kHome};
    SpiLcdDisplay* display_ = nullptr;
    lv_obj_t *photo_hint_ = nullptr, *preview_canvas_ = nullptr, *home_overlay_ = nullptr;
    std::shared_ptr<LvglFont> photo_font_;
    std::string photo_hint_text_, photo_hint_fallback_;
    uint8_t* preview_image_buf_ = nullptr;
    size_t preview_image_size_ = 0;
    lv_img_dsc_t preview_img_dsc_ = {};
'''
    with tempfile.TemporaryDirectory(prefix="camera-font-test-") as temporary:
        directory = Path(temporary)
        (directory / "lv_conf.h").write_text("""
#define LV_CONF_H
#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_MALLOC 0
#define LV_USE_FONT_PLACEHOLDER 0
#define LV_FONT_FMT_TXT_LARGE 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14
""", encoding="utf-8")
        runtime = directory / "font_runtime.c"
        runtime.write_text(font_runtime(), encoding="utf-8")
        harness = directory / "test.cc"
        harness.write_text(HARNESS + production_display + board_stub +
                           production_methods + "\n};\n" + production_fallbacks + TESTS,
                           encoding="utf-8")
        common = ["-g", "-fsanitize=address,undefined", "-DLV_CONF_INCLUDE_SIMPLE",
                  "-DLV_LVGL_H_INCLUDE_SIMPLE", "-I" + str(directory), "-I" + str(LVGL)]
        objects = []
        for index, path in enumerate((runtime,
                ROOT / "managed_components/78__xiaozhi-fonts/src/font_noto_basic_20_4.c",
                LVGL / "src/font/lv_font_montserrat_14.c")):
            obj = directory / f"font-{index}.o"
            subprocess.run([os.environ.get("CC", "gcc"), "-std=c11", *common,
                            "-c", str(path), "-o", str(obj)], check=True)
            objects.append(str(obj))
        binary = directory / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++20", "-Wall", "-Wextra",
                        "-Werror", *common,
                        "-I" + str(ROOT / "main/display/lvgl_display"),
                        str(harness), *objects, "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    main()

# Agents

## Cursor Cloud specific instructions

### Project overview

This is the **XiaoZhi (小智) AI Chatbot** ESP32 firmware project. It is an embedded C/C++ codebase built with ESP-IDF, targeting ESP32-S3, ESP32-C3, ESP32-P4, and other ESP32 variants. It cannot be "run" on a regular VM — only compiled.

### Development environment

- **ESP-IDF v5.5.2** is installed at `/opt/esp-idf`. Source the environment with:
  ```bash
  export IDF_PATH=/opt/esp-idf && source /opt/esp-idf/export.sh
  ```
  This is already added to `~/.bashrc` and should be available automatically in new shells.

- The workspace rule (`.agents/rules/dev.md`) instructs: focus changes on `bread-compact-wifi-s3cam` and `sdkconfig`; do not build from agent code directly unless needed.

### Build commands

Standard build for the `bread-compact-wifi-s3cam` board (ESP32-S3 target):

```bash
cd /workspace
idf.py set-target esp32s3
# Append board selection to sdkconfig:
echo "CONFIG_BOARD_TYPE_BREAD_COMPACT_WIFI_CAM=y" >> sdkconfig
echo "CONFIG_LCD_ST7789_240X320=y" >> sdkconfig
# Build:
idf.py -DBOARD_NAME=bread-compact-wifi-s3cam -DBOARD_TYPE=bread-compact-wifi-s3cam build
```

For CI-style multi-board builds, see `scripts/release.py`.

### Linting

- **Code style**: Google C++ style, enforced via `.clang-format` (requires clang-format 19+).
- Run lint check: `clang-format --dry-run <file>` or `clang-format -i <file>` to auto-fix.
- `clang-format-19` is installed and symlinked as `clang-format`.

### Testing

There are no automated unit tests in this repository. Validation is done by successful compilation (`idf.py build`). End-to-end testing requires physical ESP32 hardware.

### Key gotchas

- After `idf.py set-target`, ESP-IDF Component Manager downloads ~40+ managed components into `/workspace/managed_components/`. This directory is gitignored and auto-populated.
- The `build/` directory is gitignored. A clean build after target change takes ~2 minutes.
- The `.agents/rules/dev.md` rule says "你别自己build" (don't build yourself) and "改动尽量只动 bread-compact-wifi-s3cam 和 sdkconfig" (changes should focus on bread-compact-wifi-s3cam and sdkconfig). Respect this when making code changes.

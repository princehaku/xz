# SD 卡 AVI 视频

适用板型：`lichuang-dev-ml307`。开发板连接 Wi-Fi 后可自行从 HTTP(S) 地址下载视频并写入 TF 卡；无需将卡拔到读卡器。

## 操作

1. 开发板主页进入 **SD Music**，点击 **Download AVI**，查看局域网页面地址。
2. 同一网络的手机或电脑打开 `http://开发板IP:8080/`，输入可以直接下载的 AVI 文件地址。
3. 下载完成后自动播放；网页可查看进度、暂停/继续、停止或重新扫描。屏幕也提供返回、上一段、暂停和下一段。
4. 停止后文件保存在 `/video/test.avi`，以后可以离线播放。音乐原有 MP3/PCM WAV 功能继续可用。

播放器支持单 RIFF 的 MJPG/baseline JPEG AVI，最大 320×240、1–30 fps、单帧最多 256 KiB。首版只播放画面，AVI 音轨会跳过；进入本地播放会暂停 AI 语音和唤醒，返回主页恢复语音入口。下载要求 HTTP 200 和有效 Content-Length，最大 16 MiB；暂不支持需要登录或重定向的下载页。

下载按 4 KiB 分块写入临时文件 `/video/test.part`，完整检查 AVI 结构、帧数及各帧 JPEG 格式，并验证首帧可解码后才提交。覆盖已有测试视频时先保留 `test-backup.avi`，重命名失败会尝试恢复。其它用户文件不参与替换，挂载失败不会格式化卡。

断电遗留的 `test.part` 或 `test-backup.avi` 会保留并阻止对应替换操作，避免误覆盖；需要检查后清理。停止会立即取消界面及帧发布，底层 DNS/网络连接结束后才会释放下载任务和 SD 资源，因此不可达网址可能延迟重扫或下一次操作。

局域网页面在 Wi-Fi 连接后启动，断开后关闭。写操作需要本次服务的随机 token，页面自动携带；未开放跨域访问。可以通过设备 MCP 工具 `self.sd_video.download` 提交 `url`，通过 `self.sd_video.status` 查看结果。网页的 202/queued 只说明请求已排队，下载完成需要查看状态或串口日志。

## 测试样片

本次准备的 `tmp/video-test/big_buck_bunny_320x240_10fps_mjpeg_pcm.avi` 为 12 秒、120 帧、320×240、10 fps，约 1.65 MiB。

来源为 Blender Foundation 的 [Big Buck Bunny](https://peach.blender.org/about/)，采用 [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) 许可。署名：(c) copyright 2008, Blender Foundation / www.bigbuckbunny.org。取原片 00:30–00:42，降低帧率、保留宽高比加黑边、转 MJPEG；详细来源和 SHA-256 见样片目录 `README.txt`。

## 验证

- `python3 scripts/tests/test_avi_reader.py`：真实 120 帧样片，以及 RIFF/流选择/尺寸/帧数/坏 JPEG/截断/扫描预算等边界。
- `python3 scripts/tests/test_sd_video.py`：真实 AVI 解析器与生产下载/播放代码，验证网络错误、坏文件、写盘错误、旧文件回滚、取消及视频帧生命周期。网络、JPEG 解码和硬件为宿主 stub。
- `python3 scripts/tests/test_sd_music.py`：现有音频功能回归。
- `python3 scripts/tests/test_sd_music_screen.py`：真实 LVGL 的下载进度、RGB565 帧替换与暂停、页面退出及原有主页/相机回归。

宿主回归使用 WSL 的 g++、ASan 和 UBSan，AVI 读取器共 653 个用例通过。

2026-09-14 已完成实板验证：ESP32-S3 v0.2、8 MiB PSRAM，通过 Wi-Fi 下载 1,727,884 字节样片到 TF 卡，完整校验后自动循环播放。用户确认屏幕有画面；串口记录每轮 120 帧、约 12.23 秒、9.81 fps。暂停期间帧数和进度保持不变，继续和返回有效。关闭电脑样片服务、更新固件并重启后，仍可从卡中重新播放。局域网控制接口拒绝未携带 token 的 POST（403）。当前仍为无声播放。

烧录只更新 `ota_0` 的应用地址 `0x20000`，保留原分区表、NVS 配网信息及资源分区。原设备应用和测试日志保留在本机忽略目录 `tmp/video-test/`，不进入版本库。

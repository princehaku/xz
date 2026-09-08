# SD 卡音乐

适用板型：`lichuang-dev-ml307`，当前使用 Wi-Fi，ML307 暂停使用。

## 使用

1. 使用 FAT/FAT32 格式的 SD 卡，将歌曲放在根目录或 `music` 文件夹中。`music` 内还可分两层子目录，文件名支持中文和长文件名。
2. 插入板载 TF 卡槽，点击主页底部 **SD Music**。扫描完成后自动播放，按文件路径排序并循环下一首。
3. 屏幕提供上一首、播放/暂停、下一首、音量 ±5 和返回。实体 BOOT 单击播放/暂停，双击返回。
4. 无卡、空目录或读取失败时会显示提示。插卡或更换歌曲后点“重扫”。更换卡片前先返回退出播放。
5. 未联网也能播放。Wi-Fi 配网页按 BOOT 进入 SD 音乐；返回后继续显示配网说明，自动进入配网不会中断已开始的音乐。

支持 MP3，以及 **16 bit PCM WAV**（单声道/双声道）。不支持浮点 WAV、压缩 WAV、FLAC 或 AAC。歌曲按现有音频输出参数转换为 **16 kHz、单声道**。MP3 标题采用文件名，封面不会显示；读取时跳过 ID3v2 封面标签。

扫描最多 256 首、4096 个目录项，路径最长 512 字节。字库尚未加载时，界面使用可读英文提示；中文标题暂以曲目编号显示，字体就绪后自动更新。

音乐仅从卡内读取、在设备上解码。进入音乐会结束 AI 通话，并暂停麦克风上传和唤醒；返回后恢复语音入口。卡片挂载失败不会自动格式化，不创建或修改卡内音乐文件。

## 接入与验证

- 板载 SDMMC 采用 1-bit：CLK GPIO47、CMD GPIO48、D0 GPIO21。GPIO48 原先配置的状态灯已关闭，避免抢占 CMD 信号。[官方板卡读卡教程](https://wiki.lckfb.com/zh-hans/szpi-esp32s3/beginner/sd-card.html)
- 本板配置启用 `CONFIG_FATFS_LFN_HEAP`、`CONFIG_FATFS_MAX_LFN=255`、`CONFIG_FATFS_API_ENCODING_UTF_8`。可复现的发布选项同时记录在板级 `config.json`。
- SD 读取、解码和重采样在独立工作任务执行，UI 不执行读卡。PCM 以最多 20 ms 的小块进入现有音频输出队列，暂停保留队列，切歌及返回通过会话编号拒绝迟到数据。已提交到 I2S DMA 的短尾音仍可能完成播放。
- 解码器只注册 MP3、PCM 和 WAV；缓存有上限，坏文件会跳过，连续一整轮都失败后停止。ID3v2.4 标签长度和 footer 处理按 [ID3 结构规范](https://id3.org/id3v2.4.0-structure)校验。
- 宿主回归：`scripts/tests/test_sd_music.py`、`scripts/tests/test_local_audio.py`、`scripts/tests/test_sd_music_screen.py`；原有音频与相机字体回归继续保留。宿主使用真实文件操作、生产代码及真实 LVGL，SD 驱动、解码库、重采样库和 I2S 由 stub 模拟。硬件上的声音、卡片兼容性和连续播放效果需要烧录后验证。

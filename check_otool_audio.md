# otool_audio 静态审查报告

审查日期：2026-05-14  
审查对象：`components/otool_esp_component/otool_audio` 及其在 `main` / UI 中的调用  
审查方式：源码静态审查 + 本地 `managed_components/espressif__esp-sr` 头文件对照 + 官方 ESP-SR AFE/AEC 文档对照。未在板端做录音、播放、AEC 实测，也未执行本次编译。

## 1. 总体结论

当前 `otool_audio` 的 **codec 管理、PCM 播放、批处理录音、通道拆分测试** 已经形成了较完整的工程结构；`audio_tools` 作为总控对象，惰性创建 `audio_playback` / `audio_recorder` / `audio_sr_afe` 子对象，代码职责基本清晰。

但是，如果把当前实现理解为“完整 ESP-SR AFE/SR 功能”，结论是：**尚不符合官方 AFE/SR 示例流程**。当前 `audio_sr_afe` 实际上是一个 **低层 `esp_afe_aec` AEC wrapper**，并没有实现官方 ESP-SR AFE 的 `afe_config_init()`、`esp_afe_handle_from_config()`、`create_from_config()`、`feed()`、`fetch()` 流程，也没有真正接入 WakeNet、MultiNet、VAD、NS、BSS/SE、AGC 等功能。

更关键的是，ES7210 “TDM 模式”当前存在明显逻辑矛盾：`es7210_init(..., ES7210_TDM_ENABLED)` 会把状态标记为 TDM/4 mic，但底层 `i2s_rx_init()` 又明确把 RX 强制配置为 **standard I2S**，并把 `rx_tdm_slot_count` 清零。这会直接影响四麦克风录音、MIC3 参考通道、流式 AEC，以及 UI 录音功能的可信度。

综合风险等级：**High / 高风险**。建议先修正 I2S RX/TDM 与 AEC 参考通道，再决定是否把 `audio_sr_afe` 升级为完整 ESP-SR AFE。

## 2. 官方流程对照

### 2.1 官方 ESP-SR AFE 流程

本项目依赖 `espressif/esp-sr: ^2.4.4`。本地 `CHANGELOG.md` 显示：

- `2.4.3` 增加 ESP32-S3 / ESP32-P4 Full-Duplex AEC and AFE。
- `2.0.1` 增加 `afe_aec_create`、`afe_aec_destroy`、`afe_aec_process` 低层接口。
- `2.0.0` 重构 AFE interface，V2.0 与旧接口不兼容。

官方 AFE 文档与 `esp_afe_sr_iface.h` / `esp_afe_config.h` 的关键流程是：

1. 用 `input_format` 字符串描述输入通道排列：`M` = microphone，`R` = playback reference，`N` = unused/unknown。
2. 输入数据必须是 **channel-interleaved**。
3. AFE feed 输入为 **signed 16-bit / 16 kHz**。
4. 使用 `afe_config_init(input_format, models, afe_type, afe_mode)` 初始化配置。
5. 使用 `esp_afe_handle_from_config(afe_config)` 获取 `esp_afe_sr_iface_t`。
6. 使用 `afe_handle->create_from_config(afe_config)` 创建实例。
7. 使用 `get_feed_chunksize()` / `get_feed_channel_num()` 获取 feed 帧长和通道数。
8. 持续 `feed(afe_data, feed_buff)`。
9. 持续 `fetch()` 或 `fetch_with_delay()` 获取单通道增强结果、VAD/WakeNet 状态、raw data 等。
10. 用 `destroy()` 释放。

官方 AFE 适用于同时需要 AEC、NS、VAD、WakeNet、BSS/SE、AGC 等前端能力的场景。

### 2.2 官方 AEC 低层流程

官方 AEC 文档提供两种集成方式：

- 直接调用 AEC API：`aec_create()` / `aec_get_chunksize()` / `aec_process()` / `aec_destroy()`。
- 通过 AFE 模块使用 AEC。

本地 `esp_afe_aec.h` 又提供 AFE 包装版低层接口：

- `afe_aec_create(input_format, filter_length, type, mode)`
- `afe_aec_get_chunksize(handle)`
- `afe_aec_process(handle, indata, outdata)`
- `afe_aec_destroy(handle)`

该接口说明有两个重要约束：

- 当前只支持 **1 microphone channel + 1 playback reference channel**；如果输入有多路 mic/ref，只会选择第一路。
- `input_format` 仍使用 `M/R/N`，例如 `MMNR`。

当前 `audio_sr_afe` 使用的正是这一类低层 AEC API，而不是完整 AFE/SR pipeline。

## 3. 驱动与初始化层评估

### 3.1 `audio_tools::audio_system_init()`

优点：

- 只做音频系统基础初始化，codec 初始化由 `es8311_init()` / `es7210_init()` 显式完成，职责边界较清楚。
- 创建 TX/RX I2S channel，并创建共享 `audio_codec_new_i2s_data()`，减少多 codec 重复创建 data interface 的冲突。
- `audio_system_deinit()` 会统一 deinit ES8311/ES7210、删除共享 data interface，并尝试释放 I2S channel。

注意点：

- `audio_system_init()` 会同时创建 TX/RX channel，但具体 std/tdm mode 在 codec 初始化时再设置，这是可行的，但要求后续 `i2s_tx_init()` / `i2s_rx_init()` 与 codec 的 `esp_codec_dev_open()` 参数完全一致。
- 当前最大风险不在这里，而在 `i2s_rx_init()` 对 TDM 的处理。

### 3.2 ES8311 播放链路

整体评价：**基本合理**。

已观察到的正向设计：

- `es8311_init()` 根据 `ES8311_MODE_DAC` / `ADC` / `DAC_AND_ADC` 选择工作模式。
- 使用 `esp_codec_dev` 抽象设备，并通过共享 I2S data interface 接入。
- `set_volume()` 会在播放设备就绪后即时应用音量。
- `audio_playback` 支持嵌入 PCM、buffer 播放、阻塞/异步播放、停止异步播放、写静音清管线。

潜在问题：

- `es8311_sleep()` 关闭设备后没有看到对应的显式 wake/reopen API。若后续需要从 sleep 恢复播放，建议补齐状态恢复路径。
- `clear_audio_pipeline()` 会写静音并短暂修改音量；当前实现可用于降低残留噪声，但仍建议在硬件上验证是否产生尾音或音量跳变。

### 3.3 ES7210 录音链路与 TDM 矛盾

这是本次审查中最关键的问题。

`es7210_init()` 中：

- 当 `use_tdm == ES7210_TDM_ENABLED` 时，设置 `es7210_use_tdm = true`。
- TDM 下 `effective_mask = AUDIO_MIC_CHANNEL_ALL`，即实际初始化四路 MIC。
- 设置 `rx_tdm_slot_count = 4`。
- `effective_channels` 被设置为 `AUDIO_CHANNELS_STEREO`。

但 `audio_tools::i2s_rx_init()` 中：

- 即使 `es7210_use_tdm == true`，也打印：`es7210_use_tdm=TRUE but RX is forced to standard I2S configuration`。
- 实际调用的是 `i2s_channel_init_std_mode()`。
- 随后把 `rx_tdm_slot_count = 0`。

这意味着：

- 代码状态声称“ES7210 TDM / 4CH”，但硬件 I2S RX 实际是 standard I2S stereo。
- 录音拆分逻辑和 AEC streaming 又会根据 `es7210_use_tdm` 走 TDM packing 假设。
- `main.cpp` 当前初始化调用为 `es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_ALL, ES7210_TDM_ENABLED)`，正好触发该矛盾路径。

影响：

- `record_all_channel_to_files()`、`record_and_play_test_with_channel_select()`、`aec_stream_task()` 中的四通道拆分结果不可靠。
- UI 中 `aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3, ...)` 假设 MIC3 是硬件回采参考；在当前 RX 未真正 TDM 的情况下，MIC3 数据可能不是预期参考信号。
- AEC 效果可能完全失效，甚至把错误通道作为 reference，产生更差输出。

建议：

- 如果 ES7210 确实应以 TDM 输出 4 mic，则 `i2s_rx_init()` 必须在 `es7210_use_tdm` 时使用 ESP-IDF 的 TDM RX API，例如 `i2s_channel_init_tdm_mode()`，并明确 slot width、slot mask、ws/bclk/mclk 配置。
- 如果当前硬件实际上是“standard I2S stereo + 32-bit packed 4 mic”这种非标准约定，则不要命名为 TDM；应把状态变量和拆分函数命名为 packed/stereo-pack，并用实测波形确认 MIC1~MIC4 的真实映射。
- `esp_codec_dev_open()` 的 `fs.channel`、`rx_channels`、`rx_tdm_slot_count`、`split_recorded_channels()` 的 frame 计算必须统一。

### 3.4 ES7210 mute/gain 寄存器映射疑点

`es7210_set_mic_channel_mute()` 中注释和代码为：

- `0x15` 控制 ADC1/ADC2，也就是 MIC1/MIC2。
- `0x14` 控制 ADC3/ADC4，也就是 MIC3/MIC4。

但 `audio_tools::es7210_set_mic_channel_gain()` 在写增益后清 mute bit 时使用：

- `ch < 2 ? 0x14 : 0x15`

这与 `es7210_set_mic_channel_mute()` 的映射相反。结果可能是：

- 调 MIC1/MIC2 增益时，实际清的是 MIC3/MIC4 mute bit。
- 调 MIC3/MIC4 增益时，实际清的是 MIC1/MIC2 mute bit。

建议优先修正为与 `es7210_set_mic_channel_mute()` 一致：MIC1/MIC2 -> `0x15`，MIC3/MIC4 -> `0x14`。修复后应实测四路 MIC mute/gain 是否独立生效。

## 4. 播放功能评估

整体评价：**结构较好，可继续使用**。

正向点：

- `audio_playback.cpp` 用 `AUDIO_FILE_TABLE` 管理嵌入 PCM 文件元数据。
- CMake 通过 `AUDIO_FILE_CONFIGS` 生成 `USE_AUDIO_*` 宏并嵌入文件，结构清晰。
- `play_audio_file_impl()` / `play_audio_buffer_impl()` 会调用 `remix_convert_pcm_to_format()`，把输入音频转成当前系统采样率、声道数、位深后播放。
- 异步播放会复制 buffer，避免调用者释放原始 buffer 后悬垂。
- `stop_async_playback()` 使用 task notify 停止后台任务，逻辑基本完整。

风险点：

- 播放写入过程本身没有全程持有 `audio_mutex`；`clear_audio_pipeline()` 和 `stop_async_playback()` 会加锁，但播放 task 写数据期间与其他音频控制操作仍有交错可能。当前 UI/测试场景通常可接受，但如果后续增加多任务音频操作，建议定义更严格的音频状态机。
- AEC streaming 与异步播放是设计上允许共存的，但需要可靠的 playback reference。仅“同时播放”并不能保证 AEC 有正确参考通道。

## 5. 录音功能评估

整体评价：**批处理录音可用，流式录音 API 未完成**。

正向点：

- `record_to_file()` 对文件参数、音频路径、chunk 对齐、写入异常有基本检查。
- `record_all_channel_to_files()` 能批量录制并拆分所有启用 MIC，且有 RMS、峰值、zero、clip 等质量指标。
- `split_recorded_channels()` 同时支持 standard I2S 和当前假设的 TDM/packed 拆分。
- 录音与 AEC streaming 有互斥检查，避免两个录音消费者同时读 `record_dev`。

关键问题：

- `record_session_start()` / `record_session_read()` / `record_session_stop()` 目前仍是 TODO stub，分别返回 `ESP_ERR_NOT_SUPPORTED` 或 `-1`。
- 头文件中已经暴露了流式录音 API，并在注释里描述了使用方式，但实现未完成，会误导调用者。
- UI 录音当前没有使用 `audio_recorder::record_session_*`，而是直接使用 `audio_sr_afe::aec_session_*`。这意味着 UI 的“REC”按钮实际上依赖 AEC session，即使用户只是想普通录音。

建议：

- 若需要普通流式录音，应补齐 `record_session_*`，并复用 `split_recorded_channels()` 输出 16-bit mono PCM 到 RingBuffer。
- 若短期不实现，应从 public API 或文档中明确标注“不支持”，避免上层误用。
- 普通录音与 AEC 录音建议拆成两个清晰入口：`record_session_*` 与 `aec_session_*`，UI 根据场景选择。

## 6. ESP-SR / AEC 功能评估

### 6.1 当前实现到底是什么

`audio_sr_afe.h` include 了：

- `esp_afe_sr_iface.h`
- `esp_afe_sr_models.h`
- `esp_afe_aec.h`

但 `audio_sr_afe.cpp` 实际只使用：

- `afe_aec_create()`
- `afe_aec_get_chunksize()`
- `afe_aec_process()`
- `afe_aec_destroy()`

没有使用完整 AFE/SR 的：

- `srmodel_list_t *models = esp_srmodel_init(...)`
- `afe_config_init(...)`
- `afe_config_check(...)`
- `esp_afe_handle_from_config(...)`
- `afe_handle->create_from_config(...)`
- `afe_handle->feed(...)`
- `afe_handle->fetch(...)`
- WakeNet / MultiNet / VAD / NS / BSS / AGC 相关接口

因此建议把当前模块定义为：**低层 AEC 测试/流式包装模块**，而不是完整的 “ESP-SR AFE/Speech Recognition” 模块。

### 6.2 当前低层 AEC 使用是否符合官方低层流程

部分符合。

符合点：

- 使用了 `afe_aec_create("MR", filter_length, AFE_TYPE_SR, aec_mode)`。
- 使用了 `afe_aec_get_chunksize()` 获取帧长。
- 使用了 `afe_aec_process()` 按帧处理。
- 输出 buffer 使用了 16-byte aligned allocation。
- streaming 模式要求系统采样率为 16 kHz，这与官方 AEC 16 kHz 约束一致。
- 当前只选一个 mic 和一个 reference，符合 `esp_afe_aec.h` “only support 1 mic + 1 playback channel” 的约束。

不完整或有风险的点：

- AEC reference 必须是 playback reference，即扬声器播放的 far-end 信号。当前代码把某个 ES7210 MIC 通道（默认 MIC3）当作 reference，但没有验证它确实是硬件回采/播放参考。
- 在 TDM/RX 配置不一致的情况下，MIC3 reference 很可能不是预期信号。
- `aec_init(..., AUDIO_MIC_NONE, ...)` 允许 silent reference，此时不会创建 `afe_aec_handle_t`，但 `aec_stream_task()` 启动后无条件调用 `afe_aec_get_chunksize(self->aec_ctx_.handle)`，存在 null handle 崩溃风险。
- `aec_test()` 注释写“自动完成初始化->测试->清理”，实际只调用 `aec_init()` + `aec_test_loopback()`，没有调用 `aec_deinit()`。
- batch 模式对非 16 kHz 输入用线性重采样到 16 kHz，再处理后重采回原采样率。作为测试工具可以接受，但不是官方高质量 AFE 流程；若用于语音识别，建议统一硬件采样率为 16 kHz。
- 当前硬编码 `AFE_TYPE_SR`，没有暴露官方新增的 `AFE_TYPE_FD` full-duplex 场景，也没有配置 `aec_nlp_level`。如果目标是全双工对话/通话，应考虑 `AFE_TYPE_FD` 或官方直接 AEC 的 FD mode。

### 6.3 与完整 ESP-SR AFE/SR 功能的差距

当前未实现：

- WakeNet 唤醒词。
- MultiNet 离线命令词识别。
- VAD/VADNet 语音活动检测。
- NS/NSNET 降噪。
- BSS/SE 双麦增强。
- AGC 自动增益。
- AFE pipeline 的 `feed/fetch` 任务模型。
- AFE fetch result 中的 `vad_state`、`wakeup_state`、`wake_word_index`、`trigger_channel_id` 等状态输出。

所以如果产品需求写的是“esp_sr 相关功能，包括但不限于回声消除”，当前只覆盖了“回声消除的一部分”，没有覆盖 “SR/AFE 完整功能”。

## 7. 上层调用影响

### 7.1 `main.cpp`

当前主程序音频初始化：

- 初始化系统为 `AUDIO_SAMPLE_RATE_16K` + `I2S_DATA_BIT_WIDTH_32BIT`。
- ES8311 使用 DAC。
- ES7210 调用 `es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_ALL, ES7210_TDM_ENABLED)`。
- 注释测试中使用 `aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3, 4, AFE_MODE_LOW_COST)`。

这说明上层已经假设：MIC1 是近端语音，MIC3 是硬件回采参考。但底层 TDM/RX 逻辑目前不能可靠支撑这个假设。

### 7.2 `ui_mian_sync.cpp`

UI 录音按钮状态机中：

- 进入 recording 时，如果 AEC 未初始化，则调用 `aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3, 4, AFE_MODE_LOW_COST)`。
- 启动 `aec_session_start()`。
- 后台 accumulator task 持续 `aec_session_read()`，把 16 kHz / 16-bit / mono 数据累积到 SPIRAM。
- 暂停或退出时停止 AEC session。
- 播放时将累积的 mono buffer 送给 playback。

风险：UI 的普通 REC 功能被 AEC session 强绑定。一旦 AEC reference 或 TDM 拆分不正确，UI 录音也会受影响。若只需要普通录音，应改用独立 `record_session_*`；若需要“消回声录音”，则必须先解决 TDM/ref 通道可信度。

## 8. 问题清单

### Critical

1. **ES7210 TDM 状态与 I2S RX 实际配置矛盾**  
	`es7210_use_tdm=true`，但 `i2s_rx_init()` 强制 standard I2S 并清零 `rx_tdm_slot_count`。这会直接破坏四麦克风拆分和 AEC reference。

2. **当前 `audio_sr_afe` 不是完整 ESP-SR AFE/SR 流程**  
	未实现官方 `afe_config_init` / `esp_afe_handle_from_config` / `feed` / `fetch`，也没有 WakeNet/MultiNet/VAD/NS/BSS/AGC。

### High

3. **AEC streaming silent reference 会空指针调用**  
	`aec_init(..., AUDIO_MIC_NONE, ...)` 不创建 handle，但 `aec_stream_task()` 无条件 `afe_aec_get_chunksize(handle)`。

4. **`record_session_*` public API 未实现**  
	头文件暴露并描述了流式录音 API，但 cpp 返回 `ESP_ERR_NOT_SUPPORTED` / `-1`。

5. **`es7210_set_mic_channel_gain()` 清 mute 寄存器映射疑似反了**  
	与 `es7210_set_mic_channel_mute()` 的 MIC1/2 -> `0x15`、MIC3/4 -> `0x14` 不一致。

6. **AEC reference 通道缺少硬件/信号验证**  
	当前默认 MIC3 为参考，但代码没有证明 MIC3 是 playback far-end 回采；若只是普通麦克风，AEC 不会有效。

### Medium

7. **`aec_test()` 注释与实现不一致**  
	注释写自动清理，实际未 `aec_deinit()`。

8. **batch AEC 的线性重采样仅适合测试，不适合作为高质量 SR 前端**  
	官方 AEC/AFE 输入应统一为 16 kHz / int16。

9. **播放、清管线、停止任务之间仍有状态交错可能**  
	当前足够测试使用，但若多任务并发播放/录音/停止，需要更明确的音频状态机。

10. **TDM 下 `esp_codec_dev_open()` 的 channel 参数与四通道语义不完全一致**  
	 当前 TDM 时仍按 1/2 channel 打开设备，是否符合 ES7210 实际输出格式需要硬件验证。

### Low

11. **命名容易误导**  
	 `audio_sr_afe` 容易让人以为已经接入完整 ESP-SR AFE/SR。建议更名或在 README/API 注释中明确“当前仅封装低层 AEC”。

12. **废弃测试函数仍占较多代码**  
	 `record_test`、`record_and_play_test` 等标注 deprecated，但仍是主要测试逻辑。建议迁移到明确的 test utility 或示例代码中。

## 9. 建议修复顺序

### P0：先修硬件音频链路

1. 明确 ES7210 到 ESP32-P4 的真实数据格式：真正 TDM 4 slot，还是 standard stereo packed。
2. 修正 `i2s_rx_init()`：
	- 真 TDM：使用 TDM RX API，并保持 `rx_tdm_slot_count=4`。
	- 非真 TDM packed：重命名状态，避免 `TDM` 误导，并文档化 packed layout。
3. 修正 `esp_codec_dev_open()` 的 channel/slot 配置，使其与真实 I2S RX 数据格式一致。
4. 保存四路 raw PCM，确认 MIC1/MIC2/MIC3/MIC4 与硬件丝印/原理图对应关系。

### P1：修 AEC 基础安全性

1. 修复 `aec_stream_task()` 对 null handle 的处理：silent reference 应使用固定 16 kHz frame size 透传，或禁止 silent reference 启动 streaming。
2. 修复 `aec_test()` 注释或实现：要么真正 finally `aec_deinit()`，要么说明 handle 会保留。
3. 对 `aec_init()` 增加 reference 通道有效性验证/提示：要求 reference 必须是 playback loopback。
4. 增加 `AFE_TYPE_FD` 或直接 AEC FD mode 的选择入口，用于 full-duplex 场景。

### P2：补齐录音 API

1. 实现 `record_session_start/read/stop()`，输出 16-bit mono PCM。
2. UI 普通录音改用 `record_session_*`；需要消回声时才使用 `aec_session_*`。
3. 明确普通录音、AEC 录音、播放之间的互斥关系和状态转移。

### P3：决定是否升级完整 ESP-SR AFE

如果目标是完整 ESP-SR：

1. 新增完整 AFE path：
	- `esp_srmodel_init("model")`
	- `afe_config_init("MNR"/"MMNR", models, AFE_TYPE_SR 或 AFE_TYPE_FD, AFE_MODE_*)`
	- 根据需求开启/关闭 `aec_init`、`vad_init`、`wakenet_init`、`ns_init`、`se_init`、`agc_init`
	- `esp_afe_handle_from_config()`
	- `create_from_config()`
	- feed task + fetch task
2. 保留当前低层 AEC wrapper，但建议改名为 `audio_aec` 或 `audio_lowlevel_aec`，避免和完整 AFE 混淆。
3. 在 fetch 结果里向 UI/应用层输出 `vad_state`、`wakeup_state`、`wake_word_index` 等状态。

## 10. 建议验证用例

### 基础录音验证

1. 16 kHz / 32-bit slot，录制 10 秒所有通道。
2. 对 MIC1~MIC4 分别敲击/吹气，确认只有对应通道 RMS/峰值变化。
3. 分别 mute/unmute 四个 MIC，确认 mute 寄存器映射正确。
4. 分别设置四个 MIC gain，确认增益只影响目标通道。

### 播放验证

1. 播放 16 kHz stereo PCM，确认无爆音/尾音。
2. 播放 44.1 kHz / mono PCM，确认重采样和声道转换正常。
3. 异步播放中途 stop，确认任务退出且后续还能播放。

### AEC 验证

1. 播放已知 1 kHz sine 或语音文件，同时录 MIC1 和 reference 通道。
2. 确认 reference 通道与播放信号高度相关，而不是普通环境麦克风。
3. 对比 AEC 前后：播放信号频段应显著下降，近端讲话不应被过度削弱。
4. 测试 `AUDIO_MIC_NONE` silent reference 的 streaming，不允许崩溃。
5. 长时间运行 `aec_session_start()` + async playback + `aec_session_read()`，观察 ringbuffer overflow、内存泄漏和任务退出。

### 完整 ESP-SR AFE 验证（若实现）

1. feed/fetch 跑通，fetch 输出单通道数据。
2. VAD 状态能随讲话变化。
3. WakeNet 能输出 `wakeup_state` / `wake_word_index`。
4. MultiNet 能在唤醒后识别命令词。
5. AEC/NS/VAD/WakeNet 同时启用时 CPU、PSRAM、延迟可接受。

## 11. 最终判断

- **驱动层**：ES8311 播放链路基本合理；ES7210 初始化和通道控制有可修复问题；TDM/RX 是当前最大阻塞点。
- **播放层**：总体可用，建议后续加强并发状态机。
- **录音层**：批处理可用，流式普通录音未实现。
- **ESP-SR/AEC 层**：低层 AEC wrapper 部分符合官方低层 AEC 思路，但不等同于完整 ESP-SR AFE/SR；当前还不能声称支持 WakeNet/MultiNet/VAD/NS 等官方 AFE 流程。

建议下一步先完成 **ES7210 RX 数据格式实测与 TDM/packed 逻辑修正**。这是后续普通录音、AEC、UI REC、完整 ESP-SR 的共同地基；地基稳了，上层才不会像在果冻上盖楼。

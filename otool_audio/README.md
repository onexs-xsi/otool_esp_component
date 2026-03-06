# otool_audio — 音频子组件

> **平台**：ESP-IDF 5.5.x · ESP32-P4  
> **硬件**：ES8311 (DAC) + ES7210 (4-ch ADC/TDM) · I2S  
> **依赖**：`esp_codec_dev` · `esp-sr` · `esp-dsp`

---

## 目录

- [目录结构](#目录结构)
- [快速开始](#快速开始)
  - [前置条件](#前置条件)
  - [最小初始化](#最小初始化)
- [核心 API — audio\_tools](#核心-api--audio_tools)
  - [系统生命周期](#系统生命周期)
  - [ES8311 DAC 管理](#es8311-dac-管理)
  - [ES7210 ADC 管理](#es7210-adc-管理)
  - [I2S 通道管理（底层）](#i2s-通道管理底层)
  - [播放](#播放)
  - [录音与测试](#录音与测试)
  - [多通道拆分与质量分析](#多通道拆分与质量分析)
  - [音量与增益控制](#音量与增益控制)
  - [配置 Getter / Setter](#配置-getter--setter)
  - [设备句柄与 PCM 素材查询](#设备句柄与-pcm-素材查询)
- [AEC 回声消除 — audio\_sr\_afe](#aec-回声消除--audio_sr_afe)
  - [批处理 AEC 测试](#批处理-aec-测试)
  - [流式 AEC 会话](#流式-aec-会话)
- [音频格式转换 — audio\_remix\_tools](#音频格式转换--audio_remix_tools)
- [嵌入 PCM 素材管理](#嵌入-pcm-素材管理)
- [使用案例集](#使用案例集)
  - [案例 1：最小播放 Demo](#案例-1最小播放-demo)
  - [案例 2：异步播放 + 手动停止](#案例-2异步播放--手动停止)
  - [案例 3：播放内存 PCM 缓冲](#案例-3播放内存-pcm-缓冲)
  - [案例 4：录音并立即播放（单通道选择）](#案例-4录音并立即播放单通道选择)
  - [案例 5：录音并保存到 SD 卡（全通道）](#案例-5录音并保存到-sd-卡全通道)
  - [案例 6：录音保存单文件（原始 I2S 流）](#案例-6录音保存单文件原始-i2s-流)
  - [案例 7：麦克风增益与静音精调](#案例-7麦克风增益与静音精调)
  - [案例 8：AEC 一键回环测试](#案例-8aec-一键回环测试)
  - [案例 9：AEC 测试结果保存到 SD 卡](#案例-9aec-测试结果保存到-sd-卡)
  - [案例 10：流式 AEC — 边播放边录制消回声](#案例-10流式-aec--边播放边录制消回声)
  - [案例 11：流式 AEC 录制 → SPIRAM 累积 → 播放](#案例-11流式-aec-录制--spiram-累积--播放)
  - [案例 12：PCM 格式转换（重采样 + 声道 + 位深）](#案例-12pcm-格式转换重采样--声道--位深)
  - [案例 13：低功耗场景 — 休眠与唤醒](#案例-13低功耗场景--休眠与唤醒)
- [硬件连接参考](#硬件连接参考)
- [常见问题](#常见问题)

---

## 目录结构

```
otool_audio/
├── include/
│   ├── audio_tools.h          # 主类 audio_tools 定义（播放/录音/I2S/ES 芯片管理）
│   ├── audio_sr_afe.h         # AEC 回声消除类 audio_sr_afe 定义
│   ├── audio_remix_tools.h    # PCM 格式转换工具（重采样/声道/位深）
│   └── audio_types.h          # 公共枚举（声道、麦克风通道）
├── audio_chip_device/
│   ├── audio_es_es8311/       # ES8311 DAC 驱动（含寄存器定义）
│   └── audio_es_es7210/       # ES7210 4ch ADC 驱动
├── audio_playback_material/   # 嵌入式 PCM 素材文件
├── audio_tools.cpp            # audio_tools 实现
├── audio_sr_afe.cpp           # audio_sr_afe 实现（AEC 批处理 + 流式会话）
├── audio_remix_tools.cpp      # 格式转换实现
├── CMakeLists.txt             # 子组件构建配置
├── sine_pcm_build.py          # 正弦波 PCM 生成脚本
├── check_and_update_material.py  # PCM 素材 ↔ 代码枚举同步脚本
├── report.md                  # 开发分析报告（BUG 追踪/架构决策）
└── README.md                  # ← 本文件
```

---

## 快速开始

### 前置条件

1. **电源就绪** — 在调用任何音频 API 前，必须先开启音频芯片电源和功放电源：
   ```cpp
   corep4.set_status(StatusParam::POWER_AUDIO_CHIP, true);  // ES8311 + ES7210 供电
   corep4.set_status(StatusParam::POWER_AUDIO_PA, true);    // 功放供电
   ```
2. **I2C 总线已初始化** — `audio_system_init()` 需要有效的 `i2c_master_bus_handle_t`
3. **I2S 引脚已配置** — 若使用非默认引脚，需先调用 `set_i2s_pin_config()`

### 最小初始化

```cpp
#include "audio_tools.h"

// 假设 corep4.otool_tools.audio 已经是组件内全局对象
auto& audio = corep4.otool_tools.audio;

// 1) 配置 I2S 引脚（与硬件原理图匹配）
audio.set_i2s_pin_config(
    Pin_P4_I2S_BCLK_PIN,    // BCK
    Pin_P4_I2S_MCLK_PIN,    // MCK
    Pin_P4_I2S_DATA_IN_PIN,  // DIN (ES7210 → ESP32)
    Pin_P4_I2S_DATA_OUT_PIN, // DOUT (ESP32 → ES8311)
    Pin_P4_I2S_DATA_WS_PIN,  // WS/LRCK
    Pin_P4_I2S_PWR_EN        // PA 使能
);

// 2) 初始化音频系统（I2C 总线 + I2S 端口 + 采样率 + 位深）
esp_err_t ret = audio.audio_system_init(
    i2c_bus_handle,       // 已初始化的 I2C 主总线句柄
    I2S_NUM_0,            // I2S 端口号
    AUDIO_SAMPLE_RATE_16K, // 16kHz（AEC 要求此采样率）
    I2S_DATA_BIT_WIDTH_32BIT  // 32-bit（TDM 模式必须）
);

// 3) 初始化 DAC（播放）
audio.es8311_init(AUDIO_CHANNELS_STEREO, ES8311_MODE_DAC);

// 4) 初始化 ADC（录音，启用 TDM 4 通道）
audio.es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_ALL, ES7210_TDM_ENABLED);

// 5) 设置音量和录音增益
audio.set_volume(60.0);       // 播放音量 0~100
audio.set_record_gain(30.0);  // 录音增益 0~100
```

> **⚠️ TDM 模式说明**：M5Stack CoreP4 使用 ES7210 的 TDM 模式，4 路麦克风数据在 I2S 双槽中交织传输。`I2S_DATA_BIT_WIDTH_32BIT` 是 TDM 模式的硬性要求。

---

## 核心 API — audio_tools

`audio_tools` 是音频子系统的主入口类，管理 ES8311/ES7210 芯片生命周期、I2S 通道、播放、录音等。

### 系统生命周期

| 方法 | 说明 |
|------|------|
| `audio_system_init(bus, port, rate, bits)` | 初始化完整音频系统（I2C + I2S + 芯片发现） |
| `audio_system_deinit()` | 反初始化所有音频资源（I2S + 芯片） |
| `audio_system_sleep()` | 使所有芯片进入低功耗休眠模式 |
| `is_system_initialized()` | 查询系统是否已初始化 |

### ES8311 DAC 管理

| 方法 | 说明 |
|------|------|
| `es8311_init(channels, mode)` | 初始化 ES8311（`mode`: DAC / ADC / DAC_AND_ADC） |
| `es8311_deinit()` | 反初始化 ES8311 |
| `es8311_sleep()` | ES8311 进入休眠 |
| `is_es8311_initialized()` | 查询是否已初始化 |
| `is_es8311_sleeping()` | 查询是否在休眠 |
| `es8311_has_dac_path()` | 是否启用了播放路径 |
| `es8311_has_adc_path()` | 是否启用了录音路径 |

**mode 枚举**：
```cpp
ES8311_MODE_DAC          // 仅播放
ES8311_MODE_ADC          // 仅录音
ES8311_MODE_DAC_AND_ADC  // 同时播放 + 录音
```

### ES7210 ADC 管理

| 方法 | 说明 |
|------|------|
| `es7210_init(channels, mic_channels, tdm)` | 初始化 ES7210（指定启用的麦克风通道和 TDM 模式） |
| `es7210_deinit()` | 反初始化 ES7210 |
| `es7210_sleep()` | ES7210 进入休眠 |
| `es7210_set_mic_channel_mute(ch, mute)` | 设置单个麦克风通道静音 |
| `es7210_set_mic_channel_gain(ch, gain)` | 设置单个/多个麦克风通道增益 |
| `is_es7210_initialized()` | 查询是否已初始化 |
| `is_es7210_tdm_mode()` | 查询是否为 TDM 模式 |

**增益枚举**（`es7210_mic_gain_t`，0dB ~ 37.5dB，共 15 档）：
```cpp
ES7210_MIC_GAIN_0DB     // 0 dB
ES7210_MIC_GAIN_3DB     // 3 dB
// ... 每 3dB 一档 ...
ES7210_MIC_GAIN_30DB    // 30 dB（常用值）
ES7210_MIC_GAIN_33DB    // 33 dB
ES7210_MIC_GAIN_37_5DB  // 37.5 dB（最大）
```

### I2S 通道管理（底层）

> 通常不需要直接调用，`audio_system_init()` 会自动管理。仅在需要手动控制 TX/RX 生命周期时使用。

| 方法 | 说明 |
|------|------|
| `i2s_channel_init()` | 创建 I2S TX+RX 通道 |
| `i2s_channel_deinit()` | 销毁 I2S 通道 |
| `i2s_tx_init()` | 仅初始化 TX 通道 |
| `i2s_tx_deinit()` | 仅销毁 TX |
| `i2s_rx_init()` | 仅初始化 RX 通道 |
| `i2s_rx_deinit()` | 仅销毁 RX |

### 播放

| 方法 | 说明 |
|------|------|
| `play_audio_file(type, mode, limit)` | 播放嵌入 PCM 素材（阻塞/异步） |
| `play_audio_buffer(buf, size, rate, ch, bits, mode, limit)` | 播放内存 PCM 缓冲（支持任意格式参数） |
| `is_async_playback_running()` | 查询异步播放是否进行中 |
| `stop_async_playback()` | 停止异步播放任务 |
| `clear_audio_pipeline(ms)` | 清理音频管道残留（发送静音数据） |

**播放模式**：
| 模式 | 行为 |
|------|------|
| `AUDIO_PLAYBACK_BLOCKING` | 阻塞调用，函数返回时播放已结束 |
| `AUDIO_PLAYBACK_ASYNC` | 立即返回，后台任务播放，用 `stop_async_playback()` 停止 |

### 录音与测试

| 方法 | 说明 | 前置条件 |
|------|------|---------|
| `record_test(duration_ms)` | 纯录音测试（不播放，仅打印统计） | ES7210 已 init |
| `record_and_play_test(duration_s)` | 录音后播放全通道混合 | ES8311(DAC) + ES7210 已 init |
| `record_and_play_test_with_channel_select(s, ch, analysis)` | 录音后选择单通道播放，可选仅分析模式 | 同上 |
| `record_and_playback_test(s, loop, ch)` | 录音 → 单通道提取 → 循环播放 | 同上 |
| `record_all_channel_to_files(s, dir, prefix)` | 录音 → 拆分 4 通道 → 保存为独立文件 | 同上 + SD 卡已挂载 |
| `record_to_file(path, s, chunk)` | 录制原始 I2S 流保存到文件 | ES7210 已 init + SD 卡已挂载 |

> **⚠️ 互斥约束**：当流式 AEC 会话运行时（`aec_session_is_running() == true`），上述录音函数会返回 `ESP_ERR_INVALID_STATE`。

### 多通道拆分与质量分析

| 静态方法 | 说明 |
|---------|------|
| `split_recorded_channels(buf, bytes, fs, is_tdm, mic_mask)` | 将原始 I2S 缓冲拆分为最多 4 路 16-bit 单声道缓冲 |
| `free_channel_split_result(result)` | 释放拆分结果内存 |
| `compute_split_channel_quality(split, quality[4])` | 计算各通道音频质量指标（RMS、零值率、削波率等） |

**`channel_split_result_t` 结构**：
```cpp
typedef struct {
    esp_err_t status;               // 拆分操作结果
    size_t samples_per_channel;     // 每通道样本数
    size_t bytes_per_sample;        // 原始样本字节数
    bool is_tdm_mode;               // 是否 TDM 模式拆分
    audio_mic_channel_t enabled_mask; // 参与拆分的通道掩码
    int16_t* mic_buffers[4];        // 各通道 16-bit 缓冲指针
} channel_split_result_t;
```

**`mic_channel_quality_t` 结构**：
```cpp
typedef struct {
    bool available;              // 通道是否有效
    size_t sample_count;         // 样本总数
    int16_t min_value, max_value; // 幅度极值
    int32_t average_abs_amplitude; // 平均绝对幅度
    double rms_db;               // RMS 电平 (dB)
    double zero_percent;         // 零值占比 (%)
    double clipped_percent;      // 削波占比 (%)
} mic_channel_quality_t;
```

### 音量与增益控制

| 方法 | 范围 | 说明 |
|------|------|------|
| `set_volume(volume)` | 0.0 ~ 100.0 | 设置 ES8311 DAC 输出音量 |
| `get_volume()` | — | 获取当前音量 |
| `set_record_gain(gain)` | 0.0 ~ 100.0 | 设置 ES8311 ADC 输入增益 |
| `get_record_gain()` | — | 获取当前增益 |
| `set_audio_levels(vol, gain)` | — | 同时设置音量 + 增益 |
| `es7210_set_mic_channel_gain(ch, gain)` | 0~37.5dB | 设置 ES7210 具体通道硬件增益 |

### 配置 Getter / Setter

| 方法 | 说明 |
|------|------|
| `set_i2s_pin_config(bck, mck, din, dout, ws, pa)` | 批量设置 I2S 引脚 |
| `set_i2c_bus(bus)` | 设置 I2C 总线句柄 |
| `set_i2s_cross_data_pins(cross)` | 设置是否交叉 DIN/DOUT 映射（硬件走线补偿） |
| `set_tx_channels(ch)` / `get_tx_channels()` | TX 声道配置 |
| `set_rx_channels(ch)` / `get_rx_channels()` | RX 声道配置 |
| `set_sample_rate(rate)` / `get_sample_rate()` | 采样率 |
| `set_bits_per_sample(bits)` / `get_bits_per_sample()` | 位深 |
| `set_i2s_port(port)` / `get_i2s_port()` | I2S 端口号 |
| `set_pins_high_z_on_sleep(enable)` | 休眠时是否将引脚设为高阻态 |
| `get_mic_channels()` / `count_selected_mics()` | 查询当前启用的麦克风通道 |
| `get_mic_channels_description(ch)` | 获取通道枚举的可读字符串 |
| `is_mic_channels_valid(ch)` | 校验通道组合有效性 |

> **⚠️ 已废弃 API**：`set_audio_channels()` / `get_audio_channels()` — 请改用独立的 `set_tx_channels()` + `set_rx_channels()`

### 设备句柄与 PCM 素材查询

| 方法 | 说明 |
|------|------|
| `get_play_device_handle()` | 获取播放设备句柄（`esp_codec_dev_handle_t`） |
| `get_record_device_handle()` | 获取录音设备句柄 |
| `get_available_pcm_count()` | 查询已嵌入的 PCM 素材数量 |
| `get_audio_file_name(type)` | 获取 PCM 素材文件名 |
| `is_audio_file_available(type)` | 检查某个素材是否已编译进固件 |
| `get_sr_afe()` | 获取/创建 `audio_sr_afe` 子对象（AEC 功能入口） |

---

## AEC 回声消除 — audio_sr_afe

`audio_sr_afe` 封装 ESP-SR AFE 的 AEC（Acoustic Echo Cancellation）模块，通过 `audio.get_sr_afe()` 获取实例。

**硬件基础**：M5Stack CoreP4 的 MIC3 通道被硬件走线连接到 ES8311 扬声器输出端，作为 AEC 的物理参考信号（Hardware Loopback Reference）。

**架构层次**：
- `aec_init()` / `aec_deinit()` — 管理 AEC 滤波器生命周期（自适应系数）
- `aec_session_start()` / `aec_session_stop()` — 管理 I2S 采集任务（可反复开关，系数跨会话保留）

### 批处理 AEC 测试

适合快速验证 AEC 效果，一次性录制 + 处理 + 播放。

| 方法 | 说明 |
|------|------|
| `aec_init(mic, ref, filter_len, mode)` | 初始化 AEC 算法句柄 |
| `aec_deinit()` | 销毁 AEC 句柄 |
| `aec_test_loopback(s, play_orig, play_proc)` | 录制后批量 AEC 处理并播放对比 |
| `aec_test_loopback_to_files(s, mount, prefix)` | 录制后保存原始/参考/净化结果到 SD 卡 |
| `aec_test(s, mic, ref, len, mode, orig, proc)` | 一键便捷测试（自动 init → test → 不 deinit） |
| `is_initialized()` | 查询 AEC 是否已初始化 |

### 流式 AEC 会话

适合实时应用（如边播放边录制消回声）。后台任务持续从 I2S 读取 mic+ref → AEC 处理 → 写入 RingBuffer。

| 方法 | 说明 |
|------|------|
| `aec_session_start(buf_size, priority, stack)` | 启动后台 AEC 采集任务 |
| `aec_session_stop()` | 停止后台任务，释放 RingBuffer |
| `aec_session_read(buf, len, timeout_ms)` | 从 RingBuffer 读取净化后 PCM（16kHz/16bit/mono） |
| `aec_session_is_running()` | 查询会话是否运行中 |

**约束**：
- 流式 AEC 要求系统采样率必须为 **16kHz**
- 流式 AEC 会话与 `record_*()` 系列函数互斥（共享 I2S RX）
- AEC 默认参数：`mic=MIC1`, `ref=MIC3`, `filter_length=4`, `mode=AFE_MODE_LOW_COST`

---

## 音频格式转换 — audio_remix_tools

独立工具函数，不依赖 `audio_tools` 实例。

| 函数 | 说明 |
|------|------|
| `remix_convert_pcm_to_format(...)` | 通用 PCM 格式转换（组合重采样 + 声道转换 + 位深转换） |
| `resample_linear(in, samples, ch, in_rate, out_rate, out, out_samples)` | 线性插值重采样 |
| `bits_to_audio_data_type(bits)` | 位深数字 → `audio_data_type_t` 枚举 |

**支持的数据类型**（`audio_data_type_t`）：
| 枚举 | 值 | 说明 |
|------|-----|------|
| `AUDIO_TYPE_INT8` | 8 | 8-bit 有符号 PCM |
| `AUDIO_TYPE_INT16` | 16 | 16-bit 有符号 PCM（最常用） |
| `AUDIO_TYPE_INT32` | 32 | 32-bit 有符号 PCM |
| `AUDIO_TYPE_FLOAT32` | 132 | 32-bit 浮点 [-1.0, 1.0] |

---

## 嵌入 PCM 素材管理

PCM 音频文件通过 CMake 的 `EMBED_FILES` 机制编译进固件。在根 `CMakeLists.txt` 中配置：

```cmake
set(AUDIO_FILE_CONFIGS
    # ID : 文件名 : 默认ON/OFF : 采样率 : 声道 : 位深 : 描述
    "AUDIO_CANDY_WIND_2CH_16K_16BIT_9S:candy_wind_pcm_2ch_16k_16bit_9s.pcm:ON:16000:2:16:candy wind 2ch"
    "AUDIO_STARTUP_2CH_16K_16BIT_4S:startup_pcm_2ch_16k_16bit_4s.pcm:ON:16000:2:16:startup 2ch"
    # ... 更多条目
)
```

- 默认 `OFF` 的素材不会编译进固件，减小 Flash 占用
- 使用 `is_audio_file_available(type)` 运行时检查素材是否可用
- 使用 `check_and_update_material.py` 脚本可自动同步 PCM 文件 ↔ 代码枚举
- 使用 `sine_pcm_build.py` 可生成自定义正弦波测试音频

**可用素材枚举**（`audio_file_type_t`）：

| 枚举 | 格式 | 默认 |
|------|------|------|
| `AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S` | 1ch 16kHz 16bit 9s | OFF |
| `AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S` | 1ch 44.1kHz 16bit 45s | OFF |
| `AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S` | 2ch 16kHz 16bit 9s | **ON** |
| `AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S` | 2ch 44.1kHz 16bit 45s | OFF |
| `AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S` | 2ch 16kHz 16bit 10s | OFF |
| `AUDIO_FILE_STARTUP_1CH_16K_16B_4S` | 1ch 16kHz 16bit 4s | OFF |
| `AUDIO_FILE_STARTUP_2CH_16K_16B_4S` | 2ch 16kHz 16bit 4s | **ON** |

---

## 使用案例集

> 以下所有案例假设音频系统已按 [最小初始化](#最小初始化) 完成配置。
> 通过 `corep4.otool_tools.audio` 访问 `audio_tools` 对象（简写为 `audio`）。

### 案例 1：最小播放 Demo

**前置条件**：ES8311 DAC 已初始化、对应 PCM 素材已编译（默认 ON）

```cpp
auto& audio = corep4.otool_tools.audio;

// 检查素材是否可用后播放（阻塞模式，播放完才返回）
if (audio.is_audio_file_available(AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S)) {
    audio.play_audio_file(AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S);
    // 此处播放已结束
}
```

### 案例 2：异步播放 + 手动停止

**前置条件**：同案例 1

```cpp
auto& audio = corep4.otool_tools.audio;

// 异步播放，立即返回
audio.play_audio_file(AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S, AUDIO_PLAYBACK_ASYNC);

// 做其他事情...
vTaskDelay(pdMS_TO_TICKS(3000));

// 检查是否还在播放
if (audio.is_async_playback_running()) {
    ESP_LOGI(TAG, "Still playing, stopping now...");
    audio.stop_async_playback();
}
```

### 案例 3：播放内存 PCM 缓冲

**前置条件**：ES8311 DAC 已初始化

```cpp
auto& audio = corep4.otool_tools.audio;

// 假设 pcm_data 是一段外部获取的 PCM 数据（如网络下载、AEC 输出等）
const uint8_t* pcm_data = /* ... */;
size_t pcm_size = /* ... */;

// 格式参数必须与实际数据匹配
esp_err_t ret = audio.play_audio_buffer(
    pcm_data, pcm_size,
    16000,                    // 采样率
    AUDIO_CHANNELS_MONO,      // 声道
    I2S_DATA_BIT_WIDTH_16BIT, // 位深
    AUDIO_PLAYBACK_ASYNC,     // 异步播放
    5.0f                      // 最多播放 5 秒（0 = 播完全部）
);
```

### 案例 4：录音并立即播放（单通道选择）

**前置条件**：ES8311(DAC) + ES7210(ADC) 均已初始化

```cpp
auto& audio = corep4.otool_tools.audio;

// 录制 5 秒，提取 MIC1 单通道播放
audio.record_and_play_test_with_channel_select(
    5,                     // 录音时长（秒）
    AUDIO_MIC_CHANNEL_1,   // 要播放的麦克风通道
    false                  // false=播放, true=仅分析不播放
);

// 仅分析模式（不播放，自动打印各通道统计）
audio.record_and_play_test_with_channel_select(3, AUDIO_MIC_CHANNEL_1, true);
```

### 案例 5：录音并保存到 SD 卡（全通道）

**前置条件**：ES7210 已初始化 + SD 卡已挂载到 `/sdcard`

```cpp
auto& audio = corep4.otool_tools.audio;

// 录制 10 秒，将 4 路麦克风分别保存为独立文件
// 输出：/sdcard/mic_test/micdump_mic1.pcm, _mic2.pcm, _mic3.pcm, _mic4.pcm（16bit/16kHz/mono）
esp_err_t ret = audio.record_all_channel_to_files(
    10,                    // 录音时长（秒）
    "/sdcard/mic_test",    // 输出目录（自动创建）
    "micdump"              // 文件前缀
);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "All channels saved to SD card");
}
```

### 案例 6：录音保存单文件（原始 I2S 流）

**前置条件**：ES7210 已初始化 + SD 卡已挂载

```cpp
auto& audio = corep4.otool_tools.audio;

// 将原始 I2S RX 数据流式写入文件（未拆通道，适合后期处理）
audio.record_to_file("/sdcard/raw_recording.pcm", 10);
```

> **注意**：原始文件格式取决于当前 I2S 配置（TDM 32-bit 双槽 → 每帧 8 字节含 4 ch × 16bit）。

### 案例 7：麦克风增益与静音精调

**前置条件**：ES7210 已初始化

```cpp
auto& audio = corep4.otool_tools.audio;

// 为不同麦克风设置不同增益
audio.es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_1, ES7210_MIC_GAIN_27DB);  // 近场语音
audio.es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_2, ES7210_MIC_GAIN_27DB);
audio.es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_3, ES7210_MIC_GAIN_33DB);  // AEC 参考通道（远场）
audio.es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_4, ES7210_MIC_GAIN_27DB);

// 静音/取消静音某个通道
audio.es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_2, true);   // 静音 MIC2
audio.es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_2, false);  // 取消静音
```

### 案例 8：AEC 一键回环测试

**前置条件**：
- ES8311(DAC) + ES7210(TDM) 均已初始化
- 采样率 16kHz（AEC 硬性要求；非 16kHz 时批处理接口会自动重采样，但流式接口不支持）

```cpp
auto& audio = corep4.otool_tools.audio;

// 一键测试：自动 init → 录音 5 秒 → AEC 处理 → 播放原始/净化音频对比
auto* afe = audio.get_sr_afe();
if (afe) {
    afe->aec_test(
        5,                      // 录音 5 秒
        AUDIO_MIC_CHANNEL_1,    // MIC1 = 期望信号
        AUDIO_MIC_CHANNEL_3,    // MIC3 = 硬件回采参考
        4,                      // 滤波器长度（ESP32-P4 推荐 4）
        AFE_MODE_LOW_COST,      // 低功耗模式
        true,                   // 播放原始音频
        true                    // 播放 AEC 处理后音频
    );
}
```

### 案例 9：AEC 测试结果保存到 SD 卡

**前置条件**：同案例 8 + SD 卡已挂载

```cpp
auto* afe = corep4.otool_tools.audio.get_sr_afe();
if (afe) {
    // 先初始化 AEC
    afe->aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3);

    // 录制 + AEC 处理 → 保存 3 个文件到 SD 卡
    // /sdcard/aec_test_mic.pcm   — 原始麦克风信号
    // /sdcard/aec_test_ref.pcm   — 参考信号
    // /sdcard/aec_test_out.pcm   — AEC 处理后输出
    afe->aec_test_loopback_to_files(5, "/sdcard", "aec_test");

    // 不调用 aec_deinit()，保留滤波器系数给后续测试
}
```

### 案例 10：流式 AEC — 边播放边录制消回声

**前置条件**：
- ES8311(DAC) + ES7210(TDM) 均已初始化
- **系统采样率必须为 16kHz**（`AUDIO_SAMPLE_RATE_16K`）
- 不能有其他 `record_*()` 函数正在运行

```cpp
auto& audio = corep4.otool_tools.audio;
auto* afe = audio.get_sr_afe();
if (!afe) return;

// 1) 初始化 AEC（MIC1=语音, MIC3=硬件回采参考）
afe->aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3, 4, AFE_MODE_LOW_COST);

// 2) 启动流式 AEC 后台任务
afe->aec_session_start(64 * 1024);  // 64KB RingBuffer

// 3) 同时异步播放音频（AEC 会实时消除喇叭回声）
if (audio.is_audio_file_available(AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S)) {
    audio.play_audio_file(AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S, AUDIO_PLAYBACK_ASYNC);
}

// 4) 从 RingBuffer 读取 AEC 净化后的 PCM（16kHz/16bit/mono）
const size_t buf_size = 16000 * 2;  // 1 秒数据量
int16_t* aec_buf = (int16_t*)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
if (aec_buf) {
    for (int i = 0; i < 10; ++i) {  // 读取约 10 秒
        int bytes = afe->aec_session_read(aec_buf, buf_size, 1000);
        if (bytes > 0) {
            ESP_LOGI(TAG, "AEC clean audio: %d bytes (iter %d)", bytes, i);
            // 可将 aec_buf 送给语音识别、保存文件、累积缓冲等
        }
    }
    heap_caps_free(aec_buf);
}

// 5) 清理
audio.stop_async_playback();
afe->aec_session_stop();
// afe->aec_deinit();  // 仅在完全退出时调用，平时保留系数
```

### 案例 11：流式 AEC 录制 → SPIRAM 累积 → 播放

**前置条件**：同案例 10

```cpp
auto& audio = corep4.otool_tools.audio;
auto* afe = audio.get_sr_afe();

// —— 录制阶段 ——
afe->aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3);
afe->aec_session_start();

// 分配 2MB SPIRAM 缓冲（约 64 秒 @ 16kHz/16bit/mono）
const size_t MAX_BUF = 2 * 1024 * 1024;
int16_t* rec_buf = (int16_t*)heap_caps_malloc(MAX_BUF, MALLOC_CAP_SPIRAM);
size_t rec_used = 0;

// 录制 5 秒
int64_t start = esp_timer_get_time();
int16_t chunk[320];  // 20ms 一帧
while ((esp_timer_get_time() - start) < 5000000LL) {
    int bytes = afe->aec_session_read(chunk, sizeof(chunk), 100);
    if (bytes > 0 && rec_used + bytes <= MAX_BUF) {
        memcpy((uint8_t*)rec_buf + rec_used, chunk, bytes);
        rec_used += bytes;
    }
}
afe->aec_session_stop();

ESP_LOGI(TAG, "Recorded %u bytes (%.1f s)", (unsigned)rec_used, rec_used / 32000.0f);

// —— 播放阶段 ——
if (rec_used > 0) {
    audio.play_audio_buffer(
        (const uint8_t*)rec_buf, rec_used,
        16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT,
        AUDIO_PLAYBACK_BLOCKING
    );
}
heap_caps_free(rec_buf);
```

### 案例 12：PCM 格式转换（重采样 + 声道 + 位深）

**前置条件**：无（纯 CPU 计算，不依赖硬件）

```cpp
#include "audio_remix_tools.h"

// 将 48kHz/立体声/16bit PCM 转换为 16kHz/单声道/16bit
uint8_t* output = nullptr;
size_t output_size = 0;

esp_err_t ret = remix_convert_pcm_to_format(
    input_pcm_data, input_pcm_size,   // 输入
    48000, 2, AUDIO_TYPE_INT16,        // 输入：48kHz, stereo, int16
    16000, 1, AUDIO_TYPE_INT16,        // 输出：16kHz, mono, int16
    &output, &output_size              // 输出缓冲（需要调用方 heap_caps_free）
);

if (ret == ESP_OK && output) {
    // 使用 output（已转换为 16kHz/mono/16bit）
    // ...
    heap_caps_free(output);
}
```

**仅重采样**（float 数据）：
```cpp
#include "audio_remix_tools.h"

float* resampled = nullptr;
size_t resampled_samples = 0;

esp_err_t ret = resample_linear(
    float_input, input_samples, 1,  // 输入 float 数据, 单声道
    44100, 16000,                   // 44.1kHz → 16kHz
    &resampled, &resampled_samples
);

if (ret == ESP_OK && resampled) {
    // 使用 resampled
    heap_caps_free(resampled);
}
```

### 案例 13：低功耗场景 — 休眠与唤醒

```cpp
auto& audio = corep4.otool_tools.audio;

// 进入休眠（ES8311 + ES7210 均进入低功耗模式）
audio.audio_system_sleep();

// ... 系统休眠一段时间 ...

// 唤醒：重新初始化
audio.audio_system_deinit();
audio.audio_system_init(i2c_bus, I2S_NUM_0, AUDIO_SAMPLE_RATE_16K, I2S_DATA_BIT_WIDTH_32BIT);
audio.es8311_init(AUDIO_CHANNELS_STEREO, ES8311_MODE_DAC);
audio.es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_ALL, ES7210_TDM_ENABLED);
```

---

## 硬件连接参考

### I2S 总线（M5Stack CoreP4 默认引脚）

| 信号 | GPIO | 说明 |
|------|------|------|
| BCLK | 见 `m5_corep4_pinmap.h` | I2S 位时钟 |
| MCLK | 见 `m5_corep4_pinmap.h` | I2S 主时钟 |
| WS/LRCK | 见 `m5_corep4_pinmap.h` | 帧同步/左右声道选择 |
| DATA_IN | 见 `m5_corep4_pinmap.h` | ES7210 ADC → ESP32 (录音) |
| DATA_OUT | 见 `m5_corep4_pinmap.h` | ESP32 → ES8311 DAC (播放) |
| PA_EN | 见 `m5_corep4_pinmap.h` | 功放使能 |

### I2C 地址

| 芯片 | 地址 | 类型 |
|------|------|------|
| ES8311 | 0x18 | DAC + ADC（低功耗） |
| ES7210 | 0x40 | 4 通道 ADC |

### ES7210 TDM 通道映射（I2S 32-bit 双槽模式）

```
┌─────────────────────┬─────────────────────┐
│    Left Slot (32b)  │   Right Slot (32b)  │
│ [High 16b] [Low 16b] │ [High 16b] [Low 16b] │
│   MIC1       MIC3   │   MIC2       MIC4   │
└─────────────────────┴─────────────────────┘
```

### AEC 硬件回采链路

```
ES8311 SPK OUT ──(PCB走线)──→ ES7210 MIC3 IN
```

MIC3 物理连接到扬声器输出端，因此 `AUDIO_MIC_CHANNEL_3` 始终作为 AEC 参考通道。

---

## 常见问题

### Q: AEC 效果不好 / 前几秒消音很差？

AEC 使用自适应滤波器，需要 2~5 秒的收敛时间。使用 **流式 AEC 会话**（`aec_session_start`）时系数会跨 session 保留，收敛一次后效果持续改善。批处理接口每次都从零开始，前几秒效果差属正常现象。

### Q: 录音函数返回 ESP_ERR_INVALID_STATE？

流式 AEC 会话和 `record_*()` 系列函数共享 I2S RX，同时只能运行一种模式。先调用 `aec_session_stop()` 再使用录音函数。

### Q: 流式 AEC 报错"系统采样率不是 16kHz"？

ESP-SR AEC 算法硬性要求 16kHz。初始化时用 `AUDIO_SAMPLE_RATE_16K`。若系统运行在其他采样率（如 44.1kHz），请使用批处理接口 `aec_test_loopback()`（内置自动重采样）。

### Q: `play_audio_file()` 返回错误？

检查 PCM 素材是否已编译进固件：调用 `is_audio_file_available(type)` 确认。默认只有 `CANDY_WIND_2CH_16K` 和 `STARTUP_2CH_16K` 开启。在 `CMakeLists.txt` 中将对应 `ENABLE_xxx` 改为 `ON` 后重新编译。

### Q: ES7210 TDM 模式下为什么必须用 32-bit？

ES7210 在 TDM 模式下，一个 LRCK 周期内传输 4 路 16-bit 数据，ESP32 使用 `I2S_SLOT_MODE_STEREO`（32bit × 2 槽）接收。左槽的高 16 位 = MIC1、低 16 位 = MIC3；右槽的高 16 位 = MIC2、低 16 位 = MIC4。

### Q: `remix_convert_pcm_to_format()` 输出 output_data 为 nullptr？

当输入格式和目标格式完全一致时，函数返回 `ESP_OK` 但 `output_data = nullptr`，表示零拷贝，不需要转换。

# otool_audio API 重构计划 v3

> **日期**：2026-03-06  
> **范围**：`components/otool_esp_component/otool_audio/` 全部公开 API  
> **目标**：子对象 class 拆分 + 流式/批处理对称 + 脱耦测试生态 + 废弃 API 迁移列表

---

## 一、现状问题分析

### 1.1 能力不对称

| 特性 | 普通录音 | AEC 录音 |
|------|---------|---------|
| 批处理（录完再处理） | ✅ 6 个函数 | ✅ `capture_aec_buffers` + test 接口 |
| 流式会话（后台读 RingBuffer） | ❌ **缺失** | ✅ `aec_session_start/stop/read` |
| 对外部已有 PCM 缓冲的处理 | N/A | ❌ **缺失**（必须走 I2S 录制） |

### 1.2 God Class 问题

`audio_tools` 单类 1091 行头文件 + 3598 行实现，承担 6 种职责：
- 硬件基础层（I2S / 编解码器生命周期）
- 播放（阻塞/异步）
- 录音（6 种测试 + file 保存 + 通道拆分）
- 配置管理（~20 getters/setters）
- AEC 子对象代理
- 嵌入 PCM 素材管理

### 1.3 测试函数与 audio_tools 强耦合

4 个录音测试函数是 `audio_tools` 成员方法，依赖 `this->record_dev`、`this->play_dev` 等私有状态，无法独立复用。

---

## 二、设计目标

1. **职责分离**：参考 `audio_sr_afe` 子对象模式，将播放、录音拆为独立 class
2. **对称性**：普通录音和 AEC 都有「批处理」和「流式会话」两种模式
3. **输入解耦**：AEC 支持外部 PCM 缓冲处理
4. **测试生态**：`test_` 前缀脱耦自由函数 + `audio_test_context_t`
5. **向后兼容**：旧 API 通过 `[[deprecated]]` 转发保留

---

## 三、类拆分设计

### 3.1 目标架构一览

```
audio_tools (核心协调器 / facade)
  │
  ├── audio_playback* playback_   ← 新 class，子对象
  │     play_audio_file / play_audio_buffer
  │     async playback / stop / clear_pipeline
  │     audio file info (get_available_pcm_count...)
  │
  ├── audio_recorder* recorder_   ← 新 class，子对象
  │     record_session_start / stop / read / is_running   (新增)
  │     record_to_file / record_all_channel_to_files
  │     split_recorded_channels / compute_quality (static)
  │     deprecated: record_test / record_and_play_test / ...
  │
  ├── audio_sr_afe* sr_afe_       ← 已有，不变
  │     AEC init/deinit / session / process_buffer   (process_buffer 新增)
  │
  └── [audio_test_utils.h]        ← 自由函数，不是子对象
        test_record_quality / test_record_and_play / ...
```

### 3.2 文件结构

```
include/
  audio_tools.h          ← 核心 class（瘦身后 ~600 行）
  audio_playback.h       ← 新增 class（~200 行）
  audio_recorder.h       ← 新增 class（~300 行）
  audio_sr_afe.h         ← 不变
  audio_types.h          ← 不变
  audio_remix_tools.h    ← 不变
  audio_test_utils.h     ← 新增（脱耦测试自由函数）

audio_tools.cpp          ← 核心实现（~900 行）
audio_playback.cpp       ← 新增（~700 行）
audio_recorder.cpp       ← 新增（~1900 行）
audio_sr_afe.cpp         ← 基本不变 + aec_process_buffer
audio_remix_tools.cpp    ← 不变
audio_test_utils.cpp     ← 新增（~500 行）

audio_chip_device/
  audio_es_es8311/...    ← 不变
  audio_es_es7210/...    ← 不变
```

### 3.3 依赖关系与 #include 策略

```
audio_types.h   (独立，无依赖)
     ↑
audio_playback.h   (前置声明 class audio_tools;)
audio_recorder.h   (前置声明 class audio_tools;)
audio_sr_afe.h     (前置声明 class audio_tools;  ← 已有)
     ↑
audio_tools.h      (#include 上面三个子对象头文件)
     ↑
audio_test_utils.h (#include audio_tools.h，用于 make_test_context)
```

每个子对象的 `.cpp` 文件 `#include "audio_tools.h"` 以获取完整 `audio_tools` 定义，从而通过 `parent_->xxx` 访问核心状态。与 `audio_sr_afe.cpp` 现有模式完全一致。

### 3.4 状态所有权划分

#### audio_tools（核心）保有的状态

| 成员 | 说明 | 被谁访问 |
|------|------|---------|
| `play_dev` | ES8311 播放设备句柄 | playback_, recorder_(deprecated tests) |
| `record_dev` | ES7210/ES8311 录音设备句柄 | recorder_, sr_afe_ |
| `tx_handle` / `rx_handle` | I2S TX/RX 通道句柄 | 核心自身 |
| `i2c_bus_handle` | I2C 总线句柄 | 核心自身（codec init） |
| `es8311_initialized` / `es7210_initialized` / `system_initialized` | 初始化标志 | 所有子对象（只读） |
| `es8311_sleeping` / `es7210_sleeping` | 睡眠标志 | 核心自身 |
| `es8311_dev_handle` / `es8311_work_mode` | ES8311 状态 | 核心自身 |
| `i2s_bck_pin` ... `pa_pin` | I2S 引脚配置 | 核心自身 |
| `i2s_user_count` / `tx_configured` / `rx_configured` | I2S 引用计数 | 核心自身 |
| `rx_tdm_slot_count` / `i2s_cross_data_pins` / `pins_high_z_on_sleep` | I2S 配置 | 核心自身 + recorder_(只读) |
| `i2s_port_num` / `tx_channels` / `rx_channels` | 通道配置 | 所有子对象（只读） |
| `sample_rate` / `bits_per_sample` | 采样格式 | 所有子对象（只读） |
| `volume` / `record_gain` | 音量增益 | 核心自身（set 方法） |
| `mic_channels` / `es7210_use_tdm` | 麦克风配置 | recorder_, sr_afe_（只读） |
| `shared_i2s_data_if` | 共享 I2S 数据接口 | 核心自身 |
| `suppress_release` | I2S 释放抑制标志 | 核心自身 |

#### audio_playback（播放子对象）独占的状态

| 成员 | 说明 |
|------|------|
| `parent_` | 指向 audio_tools（访问 play_dev, sample_rate 等） |
| `playback_task_handle_` | 异步播放任务句柄 |
| `audio_mutex_` | 播放操作互斥锁 |

> 注：`audio_mutex` 从 audio_tools 迁移到 audio_playback，因为它仅保护播放操作。

#### audio_recorder（录音子对象）独占的状态

| 成员 | 说明 |
|------|------|
| `parent_` | 指向 audio_tools |
| `record_session_task_handle_` | 流式录音后台任务句柄 |
| `record_session_ringbuf_` | 流式录音输出 RingBuffer |
| `record_session_stop_flag_` | 任务停止标志 |
| `record_session_config_` | 当前会话配置 |

#### audio_sr_afe（AEC 子对象）— 不变

已有 `aec_ctx_`、`session_task_handle_`、`session_ringbuf_`、`session_stop_flag_`。

---

## 四、各 class 详细 API 设计

### 4.1 audio_tools（核心协调器）

```cpp
class audio_tools {
private:
    // ===== 硬件句柄 =====
    esp_codec_dev_handle_t play_dev;
    esp_codec_dev_handle_t record_dev;
    i2s_chan_handle_t tx_handle, rx_handle;
    i2c_master_bus_handle_t i2c_bus_handle;
    // ... (所有现有硬件状态成员, 见 3.4 节表格)

    // ===== 子对象指针 =====
    audio_playback* playback_ = nullptr;
    audio_recorder* recorder_ = nullptr;
    audio_sr_afe* sr_afe_ = nullptr;

    // ===== I2S 基础设施 (不变) =====
    esp_err_t ensure_i2s_channel();
    void incr_i2s_user();
    void decr_i2s_user();
    void try_release_i2s();

public:
    // 声明所有子对象为友元
    friend class audio_playback;
    friend class audio_recorder;
    friend class audio_sr_afe;

    audio_tools();
    audio_tools(gpio_num_t bck_pin, ...);
    ~audio_tools();

    // ===== 系统生命周期 =====
    esp_err_t audio_system_init(i2c_master_bus_handle_t, i2s_port_t, audio_sample_rate_t, i2s_data_bit_width_t);
    esp_err_t audio_system_deinit();
    esp_err_t audio_system_sleep();

    // ===== ES8311 生命周期 =====
    esp_err_t es8311_init(audio_channels_t, es8311_path_mode_t = ES8311_MODE_DAC);
    esp_err_t es8311_deinit();
    esp_err_t es8311_sleep();

    // ===== ES7210 生命周期 =====
    esp_err_t es7210_init(audio_channels_t, audio_mic_channel_t, es7210_tdm_mode_t = ES7210_TDM_DISABLED);
    esp_err_t es7210_deinit();
    esp_err_t es7210_sleep();
    esp_err_t es7210_set_mic_channel_mute(audio_mic_channel_t, bool);
    esp_err_t es7210_set_mic_channel_gain(audio_mic_channel_t, es7210_mic_gain_t);

    // ===== I2S 底层 =====
    esp_err_t i2s_channel_init();
    esp_err_t i2s_channel_deinit();
    esp_err_t i2s_tx_init();
    esp_err_t i2s_tx_deinit();
    esp_err_t i2s_rx_init();
    esp_err_t i2s_rx_deinit();

    // ===== 配置 getters/setters =====
    void set_i2s_pin_config(gpio_num_t bck, gpio_num_t mck, gpio_num_t din, gpio_num_t dout, gpio_num_t ws, gpio_num_t pa);
    void set_i2c_bus(i2c_master_bus_handle_t bus);
    void set_i2s_cross_data_pins(bool cross);
    void set_pins_high_z_on_sleep(bool enable);
    bool get_pins_high_z_on_sleep() const;
    void set_tx_channels(audio_channels_t);
    audio_channels_t get_tx_channels() const;
    void set_rx_channels(audio_channels_t);
    audio_channels_t get_rx_channels() const;
    void set_i2s_port(i2s_port_t);
    i2s_port_t get_i2s_port() const;
    void set_sample_rate(audio_sample_rate_t);
    audio_sample_rate_t get_sample_rate() const;
    void set_bits_per_sample(i2s_data_bit_width_t);
    i2s_data_bit_width_t get_bits_per_sample() const;
    esp_err_t set_volume(float);
    float get_volume() const;
    esp_err_t set_record_gain(float);
    float get_record_gain() const;
    esp_err_t set_audio_levels(float volume, float gain);
    bool es8311_has_dac_path() const;
    bool es8311_has_adc_path() const;
    audio_mic_channel_t get_mic_channels() const;
    const char* get_mic_channels_description(audio_mic_channel_t) const;
    bool is_mic_channels_valid(audio_mic_channel_t) const;
    int count_selected_mics() const;
    bool is_es7210_tdm_mode() const;

    // ===== 状态查询 =====
    bool is_es8311_initialized() const;
    bool is_es7210_initialized() const;
    bool is_system_initialized() const;
    bool is_es8311_sleeping() const;
    bool is_es7210_sleeping() const;
    esp_codec_dev_handle_t get_play_device_handle() const;
    esp_codec_dev_handle_t get_record_device_handle() const;

    // ===== 子对象访问 =====
    audio_playback* get_playback();    ///< 首次调用时创建
    audio_recorder* get_recorder();    ///< 首次调用时创建
    audio_sr_afe* get_sr_afe();        ///< 已有，保持不变

    // ===== 废弃的兼容转发 (见附录 A) =====
    [[deprecated("Use get_playback()->play_audio_file()")]]
    esp_err_t play_audio_file(audio_file_type_t, audio_playback_mode_t = AUDIO_PLAYBACK_BLOCKING, float = 0.0f);
    // ... 其余转发见 TASK-4
};
```

**audio_tools 保留的实现**（估算 ~900 行）：
- 构造/析构 + 子对象生命周期管理
- I2S 基础设施（ensure_i2s_channel, channel_init/deinit, tx/rx_init/deinit, user counting）
- system_init/deinit/sleep
- 配置 get/set 方法
- 废弃转发 wrappers（thin，一行调用子对象）

### 4.2 audio_playback（播放子对象）

```cpp
// include/audio_playback.h
#pragma once
#include "audio_types.h"
#include "esp_codec_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class audio_tools; // 前置声明

class audio_playback {
private:
    audio_tools* parent_;
    TaskHandle_t playback_task_handle_ = nullptr;
    SemaphoreHandle_t audio_mutex_ = nullptr;

    struct playback_task_args;
    struct buffer_playback_task_args;

    esp_err_t play_audio_file_impl(audio_file_type_t, bool check_stop, float duration_limit);
    esp_err_t play_audio_buffer_impl(const uint8_t*, size_t, uint32_t, audio_channels_t,
                                     i2s_data_bit_width_t, bool check_stop, float duration_limit);
    static void playback_task_entry(void* param);
    static void buffer_playback_task_entry(void* param);
    esp_err_t get_pcm_data_and_format(audio_file_type_t, const uint8_t*&, size_t&,
                                      uint32_t&, audio_channels_t&, i2s_data_bit_width_t&);

public:
    explicit audio_playback(audio_tools* parent);
    ~audio_playback();

    // ===== 播放 =====
    esp_err_t play_audio_file(audio_file_type_t audio_type,
                              audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING,
                              float duration_limit_seconds = 0.0f);

    esp_err_t play_audio_buffer(const uint8_t* buffer, size_t buffer_size,
                                uint32_t sample_rate_hz, audio_channels_t channels,
                                i2s_data_bit_width_t bits,
                                audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING,
                                float duration_limit_seconds = 0.0f);

    bool is_async_playback_running() const { return playback_task_handle_ != nullptr; }
    esp_err_t stop_async_playback();
    esp_err_t clear_audio_pipeline(uint32_t silence_duration_ms = 100);

    // ===== 音频文件信息 =====
    int get_available_pcm_count() const;
    const char* get_audio_file_name(audio_file_type_t) const;
    bool is_audio_file_available(audio_file_type_t) const;
};
```

**audio_playback 访问 parent_ 的成员**：
- `parent_->play_dev`（写入播放数据）
- `parent_->sample_rate`、`parent_->bits_per_sample`、`parent_->tx_channels`（格式转换参考）
- `parent_->es8311_initialized`、`parent_->es8311_has_dac_path()`（前置检查）

### 4.3 audio_recorder（录音子对象）

```cpp
// include/audio_recorder.h
#pragma once
#include "audio_types.h"
#include "esp_codec_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

class audio_tools; // 前置声明

// 类型定义（从 audio_tools.h 迁移过来）
struct channel_split_result_t { ... };  // 同现有
struct mic_channel_quality_t  { ... };  // 同现有

/**
 * @brief 录音流式会话配置
 */
struct record_session_config_t {
    audio_mic_channel_t target_channel = AUDIO_MIC_CHANNEL_1;
    size_t output_ringbuf_size = 64 * 1024;
    UBaseType_t task_priority = configMAX_PRIORITIES - 3;
    uint32_t task_stack_size = 4096;
};

class audio_recorder {
private:
    audio_tools* parent_;

    // ===== 流式录音会话状态 =====
    TaskHandle_t record_session_task_handle_ = nullptr;
    RingbufHandle_t record_session_ringbuf_ = nullptr;
    volatile bool record_session_stop_flag_ = false;
    record_session_config_t record_session_config_ = {};

    static void record_stream_task(void* param);

public:
    explicit audio_recorder(audio_tools* parent);
    ~audio_recorder();

    // ===== 流式录音会话 (新增 API) =====
    esp_err_t record_session_start(const record_session_config_t& config = {});
    int record_session_read(void* buf, size_t len, uint32_t timeout_ms = 100);
    esp_err_t record_session_stop();
    bool record_session_is_running() const;

    // ===== 批处理录音 =====
    esp_err_t record_to_file(const char* filepath, uint32_t record_duration_seconds,
                             size_t chunk_size = 4096);
    esp_err_t record_all_channel_to_files(uint32_t record_duration_seconds,
                                          const char* output_directory,
                                          const char* file_prefix = nullptr);

    // ===== 通道处理工具 (static) =====
    static channel_split_result_t split_recorded_channels(const uint8_t* buffer, size_t bytes,
                                                          const esp_codec_dev_sample_info_t& fs,
                                                          bool is_tdm, audio_mic_channel_t mask);
    static void free_channel_split_result(channel_split_result_t& result);
    static void compute_split_channel_quality(const channel_split_result_t& result,
                                              mic_channel_quality_t quality[4]);

    // ===== 废弃测试函数 (代码体保留，标注 deprecated) =====
    [[deprecated("Use test_record_quality() from audio_test_utils.h")]]
    esp_err_t record_test(uint32_t record_duration_ms = 3000);

    [[deprecated("Use test_record_and_play() from audio_test_utils.h")]]
    esp_err_t record_and_play_test(uint32_t record_duration_seconds = 3);

    [[deprecated("Use test_record_channel_select() from audio_test_utils.h")]]
    esp_err_t record_and_play_test_with_channel_select(uint32_t record_duration_seconds = 3,
                                                        audio_mic_channel_t target = AUDIO_MIC_CHANNEL_1,
                                                        bool analysis_only = false);

    [[deprecated("Use test_record_and_playback_loop() from audio_test_utils.h")]]
    esp_err_t record_and_playback_test(uint32_t record_duration_seconds = 5,
                                       bool loop_playback = false,
                                       audio_mic_channel_t target = AUDIO_MIC_NONE);
};
```

**audio_recorder 访问 parent_ 的成员**：
- `parent_->record_dev`（I2S 录音读取）
- `parent_->sample_rate`、`parent_->bits_per_sample`、`parent_->rx_channels`（格式参考）
- `parent_->mic_channels`、`parent_->es7210_use_tdm`、`parent_->rx_tdm_slot_count`（TDM 配置）
- `parent_->es7210_initialized`、`parent_->es8311_initialized`（前置检查）
- `parent_->get_playback()->play_audio_buffer()`（废弃测试函数中的播放回调）
- `parent_->sr_afe_->aec_session_is_running()`（互斥检查）

### 4.4 audio_sr_afe — 变更最小

仅新增 `aec_process_buffer()` 方法，其余不变。

```cpp
// 新增方法声明（加入现有 audio_sr_afe.h）
esp_err_t aec_process_buffer(const int16_t* mic_data,
                             const int16_t* ref_data,
                             size_t sample_count,
                             int16_t** output_data,
                             size_t* output_samples);
```

同时更新互斥检查：`aec_session_start()` 入口增加 `parent_->get_recorder()->record_session_is_running()` 检查。

### 4.5 audio_test_utils（脱耦测试自由函数）

与 plan v2 设计一致，不做改动。摘要：

```cpp
struct audio_test_context_t {
    esp_codec_dev_handle_t record_dev;
    esp_codec_dev_handle_t play_dev;
    uint32_t sample_rate;
    uint8_t rx_channels;
    uint8_t bits_per_sample;
    bool is_tdm_mode;
    audio_mic_channel_t mic_mask;
    uint8_t rx_tdm_slot_count;
};

audio_test_context_t make_test_context(const audio_tools& audio);

esp_err_t test_record_quality(const audio_test_context_t& ctx, uint32_t duration_ms = 3000);
esp_err_t test_record_and_play(const audio_test_context_t& ctx, uint32_t duration_s = 3);
esp_err_t test_record_channel_select(const audio_test_context_t& ctx,
                                     audio_mic_channel_t target = AUDIO_MIC_CHANNEL_1,
                                     uint32_t duration_s = 3, bool analysis_only = false);
esp_err_t test_record_and_playback_loop(const audio_test_context_t& ctx,
                                        audio_mic_channel_t target = AUDIO_MIC_CHANNEL_1,
                                        uint32_t duration_s = 5, bool loop = false);
esp_err_t test_aec_loopback(const audio_test_context_t& ctx, void* aec_handle,
                            audio_mic_channel_t mic, audio_mic_channel_t ref,
                            uint32_t duration_s = 5, bool play_orig = true, bool play_proc = true);
```

---

## 五、用户调用方式变化对比

### 播放
```cpp
// 旧
audio.play_audio_file(AUDIO_FILE_STARTUP_2CH_16K_16B_4S);

// 新
audio.get_playback()->play_audio_file(AUDIO_FILE_STARTUP_2CH_16K_16B_4S);

// 旧代码不改也能编译（deprecated 转发），但有 warning
```

### 录音
```cpp
// 旧
audio.record_to_file("/sdcard/test.pcm", 5);
audio.record_and_play_test(3);

// 新
audio.get_recorder()->record_to_file("/sdcard/test.pcm", 5);
audio.get_recorder()->record_and_play_test(3);  // deprecated warning

// 新增流式录音
audio.get_recorder()->record_session_start({.target_channel = AUDIO_MIC_CHANNEL_1});
int n = audio.get_recorder()->record_session_read(buf, len, 100);
audio.get_recorder()->record_session_stop();
```

### AEC（不变）
```cpp
audio.get_sr_afe()->aec_init(...);
audio.get_sr_afe()->aec_session_start();
int n = audio.get_sr_afe()->aec_session_read(buf, len, 100);
audio.get_sr_afe()->aec_session_stop();

// 新增：外部缓冲 AEC
audio.get_sr_afe()->aec_process_buffer(mic, ref, samples, &out, &out_n);
```

### 脱耦测试
```cpp
auto ctx = make_test_context(audio);
test_record_quality(ctx, 3000);
test_record_channel_select(ctx, AUDIO_MIC_CHANNEL_1, 5, false);
```

---

## 六、互斥模型

三种 I2S RX 消费者共享 `record_dev`，同时只能有一个运行：

```
┌──────────────────────┐
│   I2S RX (record_dev) │
└─────────┬────────────┘
          │  同一时刻只能有一个消费者
          ├── recorder_->record_session_*()    [录音流式会话]
          ├── sr_afe_->aec_session_*()         [AEC 流式会话]
          └── recorder_->record_*() 批处理     [record_to_file / deprecated tests]
              + test_*() 自由函数              [也使用 esp_codec_dev_read]
```

**跨子对象互斥检查**：

```cpp
// recorder_->record_session_start() 入口
if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) return ESP_ERR_INVALID_STATE;
if (record_session_is_running()) return ESP_ERR_INVALID_STATE;

// sr_afe_->aec_session_start() 入口
if (parent_->recorder_ && parent_->recorder_->record_session_is_running()) return ESP_ERR_INVALID_STATE;
if (aec_session_is_running()) return ESP_ERR_INVALID_STATE;

// recorder_ 批处理入口
if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) return ESP_ERR_INVALID_STATE;
if (record_session_is_running()) return ESP_ERR_INVALID_STATE;
```

`aec_process_buffer()` 不参与 I2S 互斥（纯算法），但与 `aec_session_*()` 互斥（共享 AEC handle）。

---

## 七、实现任务清单

### TASK-0: 类拆分（最高优先级）

**目标**：将 `audio_tools` 的播放/录音职责拆入子对象，保持编译通过。

**子任务**：

#### TASK-0A: 创建 audio_playback class

1. 新建 `include/audio_playback.h`
2. 新建 `audio_playback.cpp`
3. 从 `audio_tools.h` 移出以下声明到 `audio_playback.h`：
   - `play_audio_file()`, `play_audio_buffer()`
   - `is_async_playback_running()`, `stop_async_playback()`, `clear_audio_pipeline()`
   - `get_available_pcm_count()`, `get_audio_file_name()`, `is_audio_file_available()`
   - 内部：`playback_task_args`, `buffer_playback_task_args`, `play_audio_file_impl()`, `play_audio_buffer_impl()`, `playback_task_entry()`, `buffer_playback_task_entry()`, `get_pcm_data_and_format()`
4. 从 `audio_tools.cpp` 移出对应实现到 `audio_playback.cpp`（~700 行）
   - 行范围：2468-2544 (file info) + 2499-3158 (playback impl) + 3160-3218 (clear_pipeline)
5. `audio_mutex` 迁移到 `audio_playback`
6. 在 `audio_tools` 中：
   - 添加 `audio_playback* playback_ = nullptr;` 成员
   - 添加 `audio_playback* get_playback();`（首次调用时创建）
   - 添加 deprecated 转发 wrappers（`play_audio_file` 等，一行调用 `playback_->xxx()`）
   - `friend class audio_playback;`
7. 析构函数中 `delete playback_`

**预估**：~700 行移动 + ~100 行新增（头文件声明、转发 wrappers、构造析构）

#### TASK-0B: 创建 audio_recorder class

1. 新建 `include/audio_recorder.h`
2. 新建 `audio_recorder.cpp`
3. 从 `audio_tools.h` 移出以下声明到 `audio_recorder.h`：
   - `channel_split_result_t`, `mic_channel_quality_t` 类型定义
   - `record_to_file()`, `record_all_channel_to_files()`
   - `record_test()`, `record_and_play_test()`, `record_and_play_test_with_channel_select()`, `record_and_playback_test()`
   - static: `split_recorded_channels()`, `free_channel_split_result()`, `compute_split_channel_quality()`
4. 从 `audio_tools.cpp` 移出对应实现到 `audio_recorder.cpp`（~1900 行）
   - 行范围：197-610 (static split/quality) + 1153-2418 (test functions) + 3453-3583 (record_to_file) + 1794-2108 (record_all_channel_to_files)
5. 在 `audio_tools` 中：
   - 添加 `audio_recorder* recorder_ = nullptr;` 成员
   - 添加 `audio_recorder* get_recorder();`
   - 添加 deprecated 转发 wrappers（`record_to_file` 等）
   - `friend class audio_recorder;`
6. 所有 deprecated 录音测试函数在 recorder 中添加 `[[deprecated]]` 标注
7. 析构函数中 `delete recorder_`

**预估**：~1900 行移动 + ~150 行新增

#### TASK-0C: 更新 CMakeLists.txt

```cmake
list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/audio_tools.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_playback.cpp"       # 新增
    "${CMAKE_CURRENT_LIST_DIR}/audio_recorder.cpp"        # 新增
    "${CMAKE_CURRENT_LIST_DIR}/audio_chip_device/audio_es_es8311/audio_es_es8311.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_chip_device/audio_es_es7210/audio_es_es7210.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_sr_afe.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_remix_tools.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_test_utils.cpp"      # 新增
)
```

#### TASK-0D: 更新现有调用方

- `main.cpp` — 更新 `play_audio_file()` 等调用（或暂时用 deprecated 转发）
- `m5_corep4_ui_sync.cpp` — 更新调用（如有）
- `audio_sr_afe.cpp` — 更新 `parent_->play_audio_buffer()` → `parent_->get_playback()->play_audio_buffer()`

#### TASK-0E: 构建验证

- `idf.py build` 通过
- deprecated warning 在预期位置出现

### TASK-1: 录音流式会话（在 audio_recorder 中实现）

同 plan v2 设计，唯一变化：实现在 `audio_recorder.cpp` 中而非 `audio_tools.cpp`。

**新增方法**：
- `record_session_start()`、`record_session_read()`、`record_session_stop()`、`record_session_is_running()`

**后台任务 `record_stream_task`**：
```
loop:
  1. esp_codec_dev_read(parent_->record_dev, buf, chunksize)
  2. TDM: 从 32-bit 双槽提取 target_channel 的 16-bit 样本
  3. xRingbufferSend(ringbuf, output_16bit, len)
```

**预估**：~200 行

### TASK-2: AEC 批处理缓冲（在 audio_sr_afe 中实现）

同 plan v2 设计，new `aec_process_buffer()` method in `audio_sr_afe`。

**预估**：~120 行

### TASK-3: 脱耦测试工具 (audio_test_utils)

同 plan v2 设计。新建 `include/audio_test_utils.h` + `audio_test_utils.cpp`。

**关键签名**：
```cpp
audio_test_context_t make_test_context(const audio_tools& audio);
esp_err_t test_record_quality(const audio_test_context_t& ctx, uint32_t duration_ms);
esp_err_t test_record_and_play(const audio_test_context_t& ctx, uint32_t duration_s);
esp_err_t test_record_channel_select(const audio_test_context_t& ctx, ...);
esp_err_t test_record_and_playback_loop(const audio_test_context_t& ctx, ...);
esp_err_t test_aec_loopback(const audio_test_context_t& ctx, void* aec_handle, ...);
```

**预估**：~500 行

### TASK-4: 废弃标注 + 转发 wrappers

在 `audio_tools.h` 中为所有迁移到子对象的公开方法添加 deprecated 转发：

```cpp
// 播放转发（约 8 个方法）
[[deprecated("Use get_playback()->play_audio_file()")]]
esp_err_t play_audio_file(audio_file_type_t t, audio_playback_mode_t m = AUDIO_PLAYBACK_BLOCKING, 
                          float d = 0.0f) { return get_playback()->play_audio_file(t, m, d); }

// 录音转发（约 7 个方法 + 4 个 deprecated tests）
[[deprecated("Use get_recorder()->record_to_file()")]]
esp_err_t record_to_file(const char* fp, uint32_t s, size_t cs = 4096) {
    return get_recorder()->record_to_file(fp, s, cs); }
```

**预估**：~80 行 inline 转发

### TASK-5: 互斥保护

- `audio_recorder::record_session_start()` — 检查 sr_afe session
- `audio_sr_afe::aec_session_start()` — 检查 recorder session
- `audio_recorder` 所有批处理入口 — 检查 record_session + aec_session

**预估**：~20 行分散改动

### TASK-6: 构建验证

`idf.py build` 通过，无 error。

### TASK-7: README.md 更新

新增文档：
- 子对象访问模式（`get_playback()`、`get_recorder()`、`get_sr_afe()`）
- 流式录音用法示例
- AEC 批处理用法示例
- 脱耦测试工具用法示例
- 迁移指南（旧 → 新 API）

---

## 八、任务依赖与执行顺序

```
TASK-0 (类拆分) ← 最高优先级，所有后续任务的前置条件
  ├── TASK-0A (audio_playback)
  ├── TASK-0B (audio_recorder)
  ├── TASK-0C (CMakeLists)
  ├── TASK-0D (调用方更新)
  └── TASK-0E (构建验证)
         │
         ├── TASK-1 (record_session_*)
         ├── TASK-2 (aec_process_buffer)
         ├── TASK-3 (audio_test_utils)
         │        │
         │        └── TASK-4 (deprecated 标注)
         └── TASK-5 (互斥保护)
                │
                └── TASK-6 (最终构建验证)
                       │
                       └── TASK-7 (README 更新)
```

**执行顺序**：TASK-0A → 0B → 0C → 0D → 0E → 1 → 2 → 3 → 4 → 5 → 6 → 7

---

## 九、关键设计决策记录

### D1: 子对象生命周期管理

**决策**：`playback_` 和 `recorder_` 在首次 `get_playback()` / `get_recorder()` 调用时惰性创建（与 `get_sr_afe()` 一致）。

**理由**：
- 与既有 `get_sr_afe()` 模式一致
- 用户只创建 `audio_tools` 对象不会立即分配子对象内存
- 构造函数签名不变

### D2: 硬件句柄归属

**决策**：`play_dev` 和 `record_dev` 始终由 `audio_tools` 所有，子对象通过 `parent_->` 访问。

**理由**：
- ES8311/ES7210 init 创建这些句柄，init 逻辑留在 `audio_tools` 核心
- 多个子对象可能共享同一 `record_dev`（recorder 和 sr_afe 都读 I2S RX）
- `play_dev` 的生命周期与 ES8311 绑定，不由 playback 控制

### D3: audio_mutex 归属

**决策**：`audio_mutex` 迁移至 `audio_playback`，因为它仅保护播放操作（防止 play_audio_file 与异步 playback 冲突）。

**理由**：
- 搜索代码发现 mutex 仅在 play 相关方法中使用
- 录音互斥通过运行状态标志（`record_session_is_running()` / `aec_session_is_running()`）实现，无需独立 mutex

### D4: channel_split_result_t / mic_channel_quality_t 归属

**决策**：类型定义移入 `audio_recorder.h`，`audio_tools.h` 通过 `#include "audio_recorder.h"` 间接包含。

**理由**：
- 这些类型仅被录音相关代码使用
- `audio_test_utils.h` 也需要这些类型，通过 include chain 可用
- 保持类型定义与使用者在同一头文件

### D5: 废弃转发 vs 直接移除

**决策**：所有迁移到子对象的公开方法在 `audio_tools` 上保留 `[[deprecated]]` inline 转发。

**理由**：
- 项目中 `main.cpp`、`m5_corep4_ui_sync.cpp` 等调用方可渐进迁移
- `[[deprecated]]` warning 提醒用户但不阻塞编译
- 转发实现为一行 inline，零维护成本

### D6: record_session 输出格式

**决策**：统一输出 16-bit signed mono PCM（目标单通道），与 `aec_session_read()` 一致。

### D7: 测试工具脱耦方式

**决策**：`audio_test_context_t` 结构体 + 自由函数。`make_test_context(audio)` 为便捷桥梁。

---

## 十、验收标准

- [ ] `audio_playback` class 编译通过，`get_playback()->play_audio_file()` 可工作
- [ ] `audio_recorder` class 编译通过，`get_recorder()->record_to_file()` 可工作
- [ ] `record_session_start/read/stop/is_running` 四个 API 可工作
- [ ] `aec_process_buffer()` 编译通过
- [ ] `audio_test_utils.h` 中 5 个 `test_*` 函数编译通过
- [ ] `make_test_context(audio)` 正确构造上下文
- [ ] 三种 I2S RX 消费者互斥正确
- [ ] 旧 API deprecated 转发可编译
- [ ] `idf.py build` 无 error
- [ ] README.md 包含新架构文档

---

## 附录 A：废弃 API 迁移参考

> 此列表供跨项目代码迁移时快速查阅。所有废弃 API 以 `[[deprecated]]` inline 转发形式保留在 `audio_tools` 上。

### 播放相关 — 迁移至 audio_playback

| 废弃 API (audio_tools::) | 新 API (audio_playback::) | 说明 |
|--------------------------|--------------------------|------|
| `play_audio_file(t, m, d)` | `get_playback()->play_audio_file(t, m, d)` | 参数完全相同 |
| `play_audio_buffer(buf, sz, sr, ch, bits, m, d)` | `get_playback()->play_audio_buffer(buf, sz, sr, ch, bits, m, d)` | 参数完全相同 |
| `is_async_playback_running()` | `get_playback()->is_async_playback_running()` | |
| `stop_async_playback()` | `get_playback()->stop_async_playback()` | |
| `clear_audio_pipeline(ms)` | `get_playback()->clear_audio_pipeline(ms)` | |
| `get_available_pcm_count()` | `get_playback()->get_available_pcm_count()` | |
| `get_audio_file_name(t)` | `get_playback()->get_audio_file_name(t)` | |
| `is_audio_file_available(t)` | `get_playback()->is_audio_file_available(t)` | |

### 录音相关 — 迁移至 audio_recorder

| 废弃 API (audio_tools::) | 新 API (audio_recorder::) | 说明 |
|--------------------------|--------------------------|------|
| `record_to_file(fp, s, cs)` | `get_recorder()->record_to_file(fp, s, cs)` | 参数完全相同 |
| `record_all_channel_to_files(s, dir, prefix)` | `get_recorder()->record_all_channel_to_files(s, dir, prefix)` | 参数完全相同 |
| `split_recorded_channels(...)` | `audio_recorder::split_recorded_channels(...)` | 静态方法，类名变更 |
| `free_channel_split_result(r)` | `audio_recorder::free_channel_split_result(r)` | 静态方法 |
| `compute_split_channel_quality(r, q)` | `audio_recorder::compute_split_channel_quality(r, q)` | 静态方法 |

### 录音测试 — 迁移至 audio_test_utils 自由函数

| 废弃 API | 新 API (audio_test_utils.h) | 说明 |
|----------|---------------------------|------|
| `record_test(ms)` | `test_record_quality(ctx, ms)` | 需构造 `audio_test_context_t` |
| `record_and_play_test(s)` | `test_record_and_play(ctx, s)` | |
| `record_and_play_test_with_channel_select(s, ch, a)` | `test_record_channel_select(ctx, ch, s, a)` | 参数顺序调整 |
| `record_and_playback_test(s, loop, ch)` | `test_record_and_playback_loop(ctx, ch, s, loop)` | 参数顺序调整 |

### 其他已废弃（v1.0 遗留）

| 废弃 API | 版本 | 替代方案 |
|----------|------|---------|
| `set_audio_channels(ch)` | v1.0 | `set_tx_channels(ch)` + `set_rx_channels(ch)` |
| `get_audio_channels()` | v1.0 | `get_tx_channels()` / `get_rx_channels()` |
| `audio_sr_afe::test_aec_loopback(s, play_ch)` | v1.0 | `aec_init()` + `aec_test_loopback(s, play_orig, play_proc)` |

### 迁移快速指南

```cpp
// 步骤 1: 播放迁移（仅添加 get_playback()-> 前缀）
audio.get_playback()->play_audio_file(AUDIO_FILE_STARTUP_2CH_16K_16B_4S);

// 步骤 2: 录音迁移（添加 get_recorder()-> 前缀）
audio.get_recorder()->record_to_file("/sdcard/test.pcm", 5);

// 步骤 3: 测试函数迁移（构造上下文 + 使用自由函数）
#include "audio_test_utils.h"
auto ctx = make_test_context(audio);
test_record_quality(ctx, 3000);
test_record_channel_select(ctx, AUDIO_MIC_CHANNEL_1, 5, false);
```

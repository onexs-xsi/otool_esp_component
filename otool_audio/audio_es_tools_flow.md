# audio_es_tools 主要流程概览

> 本文档总结 `audio_es_tools.cpp` 中核心功能的调用/状态流程，便于阅读与维护。使用 Mermaid 绘制流程图（VSCode 安装 Mermaid 预览插件可直接查看）。

## 1. 音频系统初始化整体流程

```mermaid
flowchart TD
    A["调用 audio_system_init"] --> B["检查是否已初始化或被占用"]
    B --> C["配置并保存 I2S 引脚"]
    C --> D["创建 I2S TX 通道"]
    C --> E["创建 I2S RX 通道"]
    D --> F["创建播放 codec_dev - ES8311"]
    E --> G["创建录音 codec_dev - ES7210"]
    F --> H["设置播放句柄与状态"]
    G --> H
    H --> I["设置内部格式: 采样率/位宽/声道"]
    I --> J["标记 system_initialized = true"]
```

## 2. ES8311 / ES7210 初始化（典型顺序）

```mermaid
sequenceDiagram
    participant APP as 应用层
    participant TOOL as audio_es_tools
    participant I2S as I2S 驱动
    participant ES8311 as ES8311(DAC)
    participant ES7210 as ES7210(ADC)

    APP->>TOOL: set_i2s_pin_config()
    APP->>TOOL: audio_system_init()
    TOOL->>I2S: 创建/配置 TX + RX
    APP->>TOOL: es8311_init()
    TOOL->>ES8311: 硬件复位/寄存器配置
    TOOL->>TOOL: 创建播放设备 play_dev
    APP->>TOOL: set_mic_channels()
    APP->>TOOL: es7210_init()
    TOOL->>ES7210: 硬件复位/寄存器配置
    TOOL->>TOOL: 创建录音设备 record_dev
    APP->>TOOL: set_volume() / set_record_gain()
```

## 3. record_and_playback_test(record_duration_seconds)

```mermaid
flowchart TD
    A[开始 record_and_playback_test] --> B{ES7210 与 ES8311 已初始化?}
    B -- 否 --> X1[返回 INVALID_STATE]
    B -- 是 --> C{record_dev 与 play_dev 有效?}
    C -- 否 --> X2[返回 INVALID_STATE]
    C -- 是 --> D[计算录音字节数]
    D --> E{内存分配成功?}
    E -- 否 --> X3[记录内存不足并退出]
    E -- 是 --> F[执行录音读取]
    F --> G{read_result < 0?}
    G -- 是 --> X4[记录读取失败并退出]
    G -- 否 --> H{读取字节数为 0?}
    H -- 是 --> X5[无数据 退出]
    H -- 否 --> I{需要下混为单声道?}
    I -- 是 --> J[执行下混]
    I -- 否 --> K[直接使用原始缓冲]
    J --> L[写入播放]
    K --> L
    L --> M{写入成功?}
    M -- 否 --> X6[播放失败]
    M -- 是 --> N[估算播放时长]
    N --> O[清理音频管线]
    O --> P[释放缓冲]
    P --> Q[返回 ESP_OK]
```

### 关键点说明

- read 返回值是“实际读取字节数”或“负数错误码”；必须与旧逻辑区分。
- 下混：当录音声道数 > 播放声道（常见：多麦录音 -> 单声道播放）时，对所有声道求平均或取首通道。
- 估算播放时长：`play_time_ms = play_size * 1000 / (sample_rate * 播放声道 * bytes_per_sample)`，用于 `vTaskDelay()` 等待数据真正输出完毕。
- `clear_audio_pipeline(timeout_ms)`：用于清除底层剩余缓冲，避免“听起来像播放两次”或尾部残留。

## 4. play_audio_file(audio_file_type)

```mermaid
flowchart LR
    A[开始 play_audio_file] --> B{播放设备就绪?}
    B -- 否 --> X1[返回错误]
    B -- 是 --> C{是否自动模式?}
    C -- 是 --> D[选择首个可用PCM]
    C -- 否 --> E[使用指定类型]
    D --> F
    E --> F[检查可用性]
    F --> G{文件可用?}
    G -- 否 --> X2[返回 NOT_FOUND]
    G -- 是 --> H[获取PCM数据]
    H --> I[记录数据长度]
    I --> J[写入播放设备]
    J --> K{写入成功?}
    K -- 否 --> X3[记录失败]
    K -- 是 --> L[记录成功并返回]
```

## 5. record_test(record_duration_ms)

```mermaid
flowchart TD
    A[开始 record_test] --> B{ES7210 已初始化?}
    B -- 否 --> X1[返回 INVALID_STATE]
    B -- 是 --> C{record_dev 有效?}
    C -- 否 --> X2[返回 INVALID_STATE]
    C -- 是 --> D[计算缓冲区大小]
    D --> E{分配内存成功?}
    E -- 否 --> X3[返回 NO_MEM]
    E -- 是 --> F[读取录音数据]
    F --> G{bytes < 0?}
    G -- 是 --> X4[记录失败]
    G -- 否 --> H[统计/分析幅度]
    H --> I[释放缓冲]
    I --> J[返回 ESP_OK]
```

## 6. 典型调用顺序（应用侧）

```mermaid
sequenceDiagram
    participant APP
    participant TOOL as audio_es_tools
    APP->>TOOL: set_i2s_pin_config()
    APP->>TOOL: audio_system_init()
    APP->>TOOL: es8311_init()
    APP->>TOOL: set_mic_channels()
    APP->>TOOL: es7210_init()
    APP->>TOOL: set_volume(%) / set_record_gain(dB)
    APP->>TOOL: record_and_playback_test(5)
```

## 7. 可能的“录一次听两次”现象排查提示

| 可能原因                         | 排查方向                                   | 解决建议                           |
| -------------------------------- | ------------------------------------------ | ---------------------------------- |
| 播放等待时间估算不足             | 日志对比实际时间                           | 增加尾部延时或查询底层缓冲状态     |
| 未清空底层 I2S / codec FIFO      | `clear_audio_pipeline` 返回值            | 在播放前后都调用一次清理           |
| 录放使用同一物理总线且有硬件回环 | 硬件原理图/codec 配置寄存器                | 关闭回采(loopback)或静音功放再录音 |
| 多声道 -> 单声道下混逻辑错误     | 下混后字节数 vs 估算时间                   | 使用 frames 计算校验               |
| 播放调用两份不同版本文件         | 工程里存在 otool_audio / otools_audio 两份 | 删除旧目录或统一引用路径           |

## 8. 后续可改进点

- 增加“播放完成”回调而非估算时间 + delay
- 使用环形缓冲 + 分段录放（降低一次性大 malloc）
- 通过 `esp_codec_dev_*` 查询底层 FIFO 状态（若驱动支持）
- 抽象通道/位宽自适应工具函数

---

如需再生成状态机版本或导出 PNG，可继续说明。

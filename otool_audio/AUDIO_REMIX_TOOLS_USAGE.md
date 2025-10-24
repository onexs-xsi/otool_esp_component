# Audio Remix Tools 使用指南

## 概述

`audio_remix_tools` 提供了统一的音频格式转换API，支持：
- ✅ **重采样**：任意采样率转换（线性插值）
- ✅ **声道转换**：单声道↔立体声
- ✅ **数据类型转换**：int16 ↔ int32 ↔ float32

## 支持的数据类型

```cpp
typedef enum {
    AUDIO_TYPE_INT8 = 8,        // 8位有符号整数 PCM
    AUDIO_TYPE_INT16 = 16,      // 16位有符号整数 PCM (最常用)
    AUDIO_TYPE_INT32 = 32,      // 32位有符号整数 PCM
    AUDIO_TYPE_FLOAT32 = 132    // 32位浮点数 [-1.0, 1.0]
} audio_data_type_t;
```

### 数据范围说明

| 类型 | 位深 | 数值范围 | 归一化范围 |
|------|------|----------|-----------|
| `INT8` | 8-bit | -128 ~ 127 | -1.0 ~ 1.0 |
| `INT16` | 16-bit | -32768 ~ 32767 | -1.0 ~ 1.0 |
| `INT32` | 32-bit | -2147483648 ~ 2147483647 | -1.0 ~ 1.0 |
| `FLOAT32` | 32-bit | -1.0 ~ 1.0 | -1.0 ~ 1.0 |

## API 接口

### `bits_to_audio_data_type()`

将位深度数值转换为音频数据类型枚举的辅助函数：

```cpp
audio_data_type_t bits_to_audio_data_type(uint32_t bits);
```

**参数：**
- `bits`: 位深度 (8, 16, 32)

**返回值：**
- 对应的 `audio_data_type_t` 枚举值
- 如果位深度不支持，默认返回 `AUDIO_TYPE_INT16` 并输出警告

**示例：**
```cpp
uint32_t bit_depth = 16;
audio_data_type_t type = bits_to_audio_data_type(bit_depth);
// type == AUDIO_TYPE_INT16

uint32_t unknown_bits = 24;
audio_data_type_t type2 = bits_to_audio_data_type(unknown_bits);
// type2 == AUDIO_TYPE_INT16 (默认回退，并输出警告)
```

### `remix_convert_pcm_to_format()`

统一的音频格式转换函数，支持一次性完成多种转换：

```cpp
esp_err_t remix_convert_pcm_to_format(
    const uint8_t* input_data,      // 输入音频数据
    size_t input_size,               // 输入数据字节数
    uint32_t input_rate,             // 输入采样率 (Hz)
    uint32_t input_channels,         // 输入声道数 (1 or 2)
    audio_data_type_t input_type,   // 输入数据类型
    uint32_t target_rate,            // 目标采样率 (Hz)
    uint32_t target_channels,        // 目标声道数 (1 or 2)
    audio_data_type_t target_type,  // 目标数据类型
    uint8_t** output_data,           // [输出] 转换后的缓冲区指针
    size_t* output_size              // [输出] 转换后的字节数
);
```

**返回值：**
- `ESP_OK`: 成功（如果格式完全一致，`output_data` 将为 `nullptr`）
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_NOT_SUPPORTED`: 不支持的数据类型或声道数
- `ESP_ERR_NO_MEM`: 内存分配失败

**内存管理：**
- 输出缓冲区由函数自动分配（优先使用SPIRAM）
- 调用者负责使用 `heap_caps_free()` 释放

## 使用示例

### 示例 1：重采样（48kHz → 16kHz，单声道int16）

```cpp
#include "audio_remix_tools.h"

void example_resample() {
    int16_t input_audio[48000];  // 1秒 @ 48kHz
    // ... 填充输入数据 ...
    
    uint8_t* output = nullptr;
    size_t output_size = 0;
    
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)input_audio, sizeof(input_audio),
        48000, 1, AUDIO_TYPE_INT16,  // 输入：48kHz 单声道 int16
        16000, 1, AUDIO_TYPE_INT16,  // 输出：16kHz 单声道 int16
        &output, &output_size
    );
    
    if (ret == ESP_OK && output != nullptr) {
        ESP_LOGI("APP", "Resampled to %zu bytes", output_size);
        int16_t* resampled = (int16_t*)output;
        size_t samples = output_size / sizeof(int16_t);
        
        // 使用重采样后的数据...
        
        heap_caps_free(output);  // 记得释放
    }
}
```

### 示例 2：声道转换（单声道 → 立体声）

```cpp
void example_mono_to_stereo() {
    int16_t mono_audio[16000];  // 1秒 @ 16kHz 单声道
    // ... 填充输入数据 ...
    
    uint8_t* stereo_output = nullptr;
    size_t stereo_size = 0;
    
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)mono_audio, sizeof(mono_audio),
        16000, 1, AUDIO_TYPE_INT16,  // 输入：单声道
        16000, 2, AUDIO_TYPE_INT16,  // 输出：立体声
        &stereo_output, &stereo_size
    );
    
    if (ret == ESP_OK && stereo_output != nullptr) {
        ESP_LOGI("APP", "Converted to stereo: %zu bytes", stereo_size);
        heap_caps_free(stereo_output);
    }
}
```

### 示例 3：类型转换（int8 → int16）

```cpp
void example_int8_to_int16() {
    int8_t pcm_data[16000];  // 1秒 @ 16kHz, 8-bit
    // ... 填充PCM数据 ...
    
    uint8_t* output = nullptr;
    size_t output_size = 0;
    
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)pcm_data, sizeof(pcm_data),
        16000, 1, AUDIO_TYPE_INT8,     // 输入：8-bit
        16000, 1, AUDIO_TYPE_INT16,    // 输出：16-bit
        &output, &output_size
    );
    
    if (ret == ESP_OK && output != nullptr) {
        int16_t* int16_samples = (int16_t*)output;
        size_t num_samples = output_size / sizeof(int16_t);
        
        // 精度从8位提升到16位
        
        heap_caps_free(output);
    }
}
```

### 示例 4：类型转换（int16 → float32）

```cpp
void example_int_to_float() {
    int16_t pcm_data[16000];  // 1秒 @ 16kHz
    // ... 填充PCM数据 ...
    
    uint8_t* float_output = nullptr;
    size_t float_size = 0;
    
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)pcm_data, sizeof(pcm_data),
        16000, 1, AUDIO_TYPE_INT16,    // 输入：int16
        16000, 1, AUDIO_TYPE_FLOAT32,  // 输出：float32 [-1.0, 1.0]
        &float_output, &float_size
    );
    
    if (ret == ESP_OK && float_output != nullptr) {
        float* float_samples = (float*)float_output;
        size_t num_samples = float_size / sizeof(float);
        
        // float_samples[i] 范围为 [-1.0, 1.0]
        
        heap_caps_free(float_output);
    }
}
```

### 示例 5：组合转换（48kHz立体声int32 → 16kHz单声道float32）

```cpp
void example_complex_conversion() {
    int32_t input[48000 * 2];  // 1秒 @ 48kHz 立体声 int32
    // ... 填充输入数据 ...
    
    uint8_t* output = nullptr;
    size_t output_size = 0;
    
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)input, sizeof(input),
        48000, 2, AUDIO_TYPE_INT32,    // 48kHz 立体声 int32
        16000, 1, AUDIO_TYPE_FLOAT32,  // 16kHz 单声道 float32
        &output, &output_size
    );
    
    if (ret == ESP_OK && output != nullptr) {
        ESP_LOGI("APP", "Complex conversion: %zu bytes", output_size);
        // 输出是 float32 单声道 @ 16kHz
        heap_caps_free(output);
    }
}
```

### 示例 6：零拷贝检测

```cpp
void example_no_conversion_needed() {
    int16_t audio[16000];
    
    uint8_t* output = nullptr;
    size_t output_size = 0;
    
    // 输入输出格式完全一致
    esp_err_t ret = remix_convert_pcm_to_format(
        (uint8_t*)audio, sizeof(audio),
        16000, 1, AUDIO_TYPE_INT16,
        16000, 1, AUDIO_TYPE_INT16,  // 相同格式
        &output, &output_size
    );
    
    if (ret == ESP_OK) {
        if (output == nullptr) {
            ESP_LOGI("APP", "No conversion needed, use original buffer");
            // 直接使用 audio[]
        } else {
            // 这种情况不应该发生
            heap_caps_free(output);
        }
    }
}
```

## 实际应用：AEC重采样

在 `audio_esp_sr_afe.cpp` 中的实际用法：

```cpp
// AEC需要16kHz，但录音可能是48kHz
uint32_t system_sample_rate = 48000;
uint32_t aec_sample_rate = 16000;

int16_t *mic_data_system = ...;  // 录制的48kHz数据
size_t samples = ...;

// 重采样到16kHz供AEC处理
uint8_t *mic_data_16k_u8 = nullptr;
size_t output_size = 0;

esp_err_t ret = remix_convert_pcm_to_format(
    (uint8_t*)mic_data_system, samples * sizeof(int16_t),
    system_sample_rate, 1, AUDIO_TYPE_INT16,
    aec_sample_rate, 1, AUDIO_TYPE_INT16,
    &mic_data_16k_u8, &output_size
);

if (ret == ESP_OK) {
    int16_t *mic_data_16k = (int16_t*)mic_data_16k_u8;
    // 使用mic_data_16k进行AEC处理...
    
    heap_caps_free(mic_data_16k_u8);
}
```

## 性能特性

### 内存分配策略
- **优先使用SPIRAM**：节省内部RAM
- **自动回退**：SPIRAM不足时使用内部RAM

### 转换流程
1. **输入 → float32** (归一化)
2. **重采样** (线性插值)
3. **声道转换** (混音/复制)
4. **float32 → 输出类型**

### 优化建议
- 🚀 固定采样率可避免重采样开销
- 💾 使用int16可减少内存占用（相比float32）
- ⚡ 批量处理优于逐帧转换

## 注意事项

1. **浮点范围**：
   - `AUDIO_TYPE_FLOAT32` 的有效范围是 `[-1.0, 1.0]`
   - 超出范围会自动钳位

2. **声道转换**：
   - 单声道→立体声：复制到左右声道
   - 立体声→单声道：平均混音 `(L+R)/2`

3. **重采样质量**：
   - 使用线性插值（简单高效）
   - 适合语音和一般音频
   - 如需高质量，考虑使用专业重采样库

4. **错误处理**：
   - 始终检查返回值
   - 检查 `output_data` 是否为 `nullptr`（零拷贝情况）
   - 记得释放分配的缓冲区

## 常见问题

### Q: 为什么删除了 `remix_resample_linear_mono`？
A: 统一使用 `remix_convert_pcm_to_format`，API更简洁，功能更强大。

### Q: 性能会变差吗？
A: 固定采样率后不需要重采样，性能影响忽略不计。多次类型转换的开销被简化的代码维护抵消。

### Q: 如何获得最佳性能？
A: 保持输入输出格式一致，函数会检测并返回零拷贝（`output_data = nullptr`）。

### Q: 支持更多声道吗？
A: 当前仅支持单声道(1)和立体声(2)。多声道需求请提issue。

---

**最后更新**: 2025-10-24  
**版本**: 2.0 (统一API)

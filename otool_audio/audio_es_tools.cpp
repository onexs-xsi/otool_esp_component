/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_es_tools.h"
#include "audio_config.h"
#include "driver/i2s_std.h"
#if SOC_I2S_SUPPORTS_TDM
#include "driver/i2s_tdm.h"
#endif
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "audio_remix_tools.h"
#include "freertos/task.h"
#include "es7210_adc.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <math.h>

#ifdef USE_PCM_CANDY_WIND_1CH_16K_16B_9S
// candy_wind_pcm_1ch_16k_16bit_9s.pcm 可用时的处理逻辑
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_PCM_CANDY_WIND_1CH_44K_16B_45S
// candy_wind_pcm_1ch_44.1k_16bit_45.5s.pcm 可用时的处理逻辑
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45_5s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45_5s_pcm_end[];
#endif

#ifdef USE_PCM_CANDY_WIND_2CH_16K_16B_9S
// candy_wind_pcm_2ch_16k_16bit_9s.pcm 可用时的处理逻辑
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_PCM_CANDY_WIND_2CH_44K_16B_45S
// candy_wind_pcm_2ch_44.1k_16bit_45.5s.pcm 可用时的处理逻辑
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45_5s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45_5s_pcm_end[];
#endif

#ifdef USE_PCM_SINE_440HZ_2CH_16K_16B_10S
// sine_440Hz_pcm_2ch_16k_16bit_10s.pcm 可用时的处理逻辑
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start[];
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end[];
#endif

#ifdef USE_PCM_STARTUP_1CH_16K_16B_4S
// startup_pcm_1ch_16k_16bit_4s.pcm 可用时的处理逻辑
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end[];
#endif

#ifdef USE_PCM_STARTUP_2CH_16K_16B_4S
// startup_pcm_2ch_16k_16bit_4s.pcm 可用时的处理逻辑
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_end[];
#endif

static const char *TAG = "audio_es_tools";
static constexpr size_t SILENCE_CHUNK_CAPACITY = 1024;
static uint8_t g_silence_chunk[SILENCE_CHUNK_CAPACITY] = {0};

namespace {

static inline int16_t load_int16_le(const uint8_t* ptr)
{
    int16_t value = 0;
    memcpy(&value, ptr, sizeof(int16_t));
    return value;
}

static inline uint32_t load_uint32_le(const uint8_t* ptr)
{
    uint32_t value = 0;
    memcpy(&value, ptr, sizeof(uint32_t));
    return value;
}

} // namespace

void audio_es_tools::free_channel_split_result(channel_split_result_t& result)
{
    for (int i = 0; i < 4; ++i) {
        if (result.mic_buffers[i]) {
            free(result.mic_buffers[i]);
            result.mic_buffers[i] = nullptr;
        }
    }

    result.samples_per_channel = 0;
    result.bytes_per_sample = 0;
    result.status = ESP_OK;
}

channel_split_result_t audio_es_tools::split_recorded_channels(const uint8_t* record_buffer,
                                                              size_t bytes_read,
                                                              const esp_codec_dev_sample_info_t& fs,
                                                              bool is_tdm_mode,
                                                              audio_mic_channel_t mic_channels)
{
    channel_split_result_t result{};
    result.status = ESP_FAIL;
    result.enabled_mask = mic_channels;
    result.is_tdm_mode = is_tdm_mode;
    result.bytes_per_sample = (fs.bits_per_sample >> 3);

    if (!record_buffer || bytes_read == 0) {
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    if (result.bytes_per_sample == 0) {
        ESP_LOGE(TAG, "Invalid bits_per_sample: %u", fs.bits_per_sample);
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    uint8_t enabled_indices[4] = {0};
    uint8_t mic_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (mic_channels & (1 << i)) {
            enabled_indices[mic_count++] = static_cast<uint8_t>(i);
        }
    }

    if (mic_count == 0) {
        ESP_LOGE(TAG, "No microphone channels enabled for splitting (mask=0x%02X)", mic_channels);
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    size_t samples_per_channel = 0;

    if (is_tdm_mode) {
        const size_t stereo_frame_bytes = result.bytes_per_sample * 2;
        if (result.bytes_per_sample != sizeof(uint32_t)) {
            ESP_LOGW(TAG, "Unexpected slot width %zu bytes in TDM mode; assuming 32-bit packing",
                     result.bytes_per_sample);
        }
        if (stereo_frame_bytes == 0) {
            ESP_LOGE(TAG, "Invalid TDM frame size calculation (slot bytes=%zu)", result.bytes_per_sample);
            result.status = ESP_ERR_INVALID_STATE;
            return result;
        }
        if (bytes_read % stereo_frame_bytes != 0) {
            ESP_LOGW(TAG, "Recorded byte count (%zu) not aligned with TDM frame size (%zu); trailing data ignored",
                     bytes_read, stereo_frame_bytes);
        }
        samples_per_channel = bytes_read / stereo_frame_bytes;
    } else {
        size_t channel_count = fs.channel ? static_cast<size_t>(fs.channel) : 1;
        const size_t frame_bytes = result.bytes_per_sample * channel_count;
        if (frame_bytes == 0) {
            ESP_LOGE(TAG, "Invalid STD frame size calculation (slot bytes=%zu, channels=%u)",
                     result.bytes_per_sample, fs.channel);
            result.status = ESP_ERR_INVALID_STATE;
            return result;
        }
        samples_per_channel = bytes_read / frame_bytes;
    }

    if (samples_per_channel == 0) {
        ESP_LOGE(TAG, "Channel splitting failed: no samples detected (bytes=%zu)", bytes_read);
        result.status = ESP_ERR_INVALID_SIZE;
        return result;
    }

    result.samples_per_channel = samples_per_channel;

    for (int i = 0; i < 4; ++i) {
        if (mic_channels & (1 << i)) {
            result.mic_buffers[i] = static_cast<int16_t*>(malloc(samples_per_channel * sizeof(int16_t)));
            if (!result.mic_buffers[i]) {
                ESP_LOGE(TAG, "Failed to allocate channel buffer for MIC%u (%zu samples)",
                         i + 1, samples_per_channel);
                for (int j = 0; j < 4; ++j) {
                    if (result.mic_buffers[j]) {
                        free(result.mic_buffers[j]);
                        result.mic_buffers[j] = nullptr;
                    }
                }
                result.status = ESP_ERR_NO_MEM;
                return result;
            }
            memset(result.mic_buffers[i], 0, samples_per_channel * sizeof(int16_t));
        }
    }

    if (is_tdm_mode) {
        const size_t stereo_frame_bytes = result.bytes_per_sample * 2;
        const size_t total_frames = bytes_read / stereo_frame_bytes;

        if (result.bytes_per_sample != sizeof(uint32_t)) {
            ESP_LOGW(TAG, "TDM splitting operating without 32-bit slot width validation (slot=%zu)",
                     result.bytes_per_sample * 8);
        }

        for (size_t frame = 0; frame < total_frames; ++frame) {
            size_t base = frame * stereo_frame_bytes;
            int16_t left_high = 0;
            int16_t left_low = 0;
            int16_t right_high = 0;
            int16_t right_low = 0;

            if (result.bytes_per_sample == sizeof(uint32_t)) {
                uint32_t left_word = load_uint32_le(record_buffer + base);
                uint32_t right_word = load_uint32_le(record_buffer + base + result.bytes_per_sample);
                left_high = static_cast<int16_t>(left_word >> 16);
                left_low = static_cast<int16_t>(left_word & 0xFFFF);
                right_high = static_cast<int16_t>(right_word >> 16);
                right_low = static_cast<int16_t>(right_word & 0xFFFF);
            } else {
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = load_int16_le(record_buffer + base + result.bytes_per_sample);
                left_high = left_sample;
                left_low = left_sample;
                right_high = right_sample;
                right_low = right_sample;
            }

            if (result.mic_buffers[0]) {
                result.mic_buffers[0][frame] = left_high;
            }
            if (result.mic_buffers[2]) {
                result.mic_buffers[2][frame] = left_low;
            }
            if (result.mic_buffers[1]) {
                result.mic_buffers[1][frame] = right_high;
            }
            if (result.mic_buffers[3]) {
                result.mic_buffers[3][frame] = right_low;
            }
        }
    } else {
        const size_t channel_count = fs.channel ? static_cast<size_t>(fs.channel) : 1;
        const size_t frame_bytes = result.bytes_per_sample * channel_count;

        if (channel_count > 2) {
            ESP_LOGW(TAG, "STD mode detected with %u channels; using first two for MIC mapping", fs.channel);
        }

        if (channel_count == 1 && mic_count > 1) {
            ESP_LOGW(TAG, "Multiple microphones enabled with mono STD stream; duplicating mono data across channels");
        }

        if (mic_count == 1) {
            const uint8_t mic_index = enabled_indices[0];
            const bool is_left_channel = (mic_index == 0 || mic_index == 2);
            const uint8_t channel_offset = is_left_channel ? 0 : 1;

            if (channel_count == 1) {
                for (size_t i = 0; i < samples_per_channel; ++i) {
                    int16_t sample = load_int16_le(record_buffer + i * result.bytes_per_sample);
                    if (result.mic_buffers[mic_index]) {
                        result.mic_buffers[mic_index][i] = sample;
                    }
                }
            } else {
                for (size_t i = 0; i < samples_per_channel; ++i) {
                    size_t sample_pos = (i * channel_count + channel_offset) * result.bytes_per_sample;
                    int16_t sample = load_int16_le(record_buffer + sample_pos);
                    if (result.mic_buffers[mic_index]) {
                        result.mic_buffers[mic_index][i] = sample;
                    }
                }
            }
        } else if (mic_count == 2) {
            const uint8_t first_mic = enabled_indices[0];
            const uint8_t second_mic = enabled_indices[1];

            for (size_t i = 0; i < samples_per_channel; ++i) {
                size_t base = i * frame_bytes;
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = (channel_count >= 2)
                                           ? load_int16_le(record_buffer + base + result.bytes_per_sample)
                                           : left_sample;

                if (result.mic_buffers[first_mic]) {
                    result.mic_buffers[first_mic][i] = left_sample;
                }
                if (result.mic_buffers[second_mic]) {
                    result.mic_buffers[second_mic][i] = right_sample;
                }
            }
        } else {
            if (mic_count == 3) {
                ESP_LOGW(TAG, "STD mode with 3 microphones enabled; one slot may carry zero samples");
            } else {
                ESP_LOGW(TAG, "STD mode with 4 microphones enabled; consider using TDM for optimal layout");
            }

            for (size_t i = 0; i < samples_per_channel; ++i) {
                size_t base = i * frame_bytes;
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = (channel_count >= 2)
                                           ? load_int16_le(record_buffer + base + result.bytes_per_sample)
                                           : left_sample;

                if (result.mic_buffers[0]) {
                    result.mic_buffers[0][i] = left_sample;
                }
                if (result.mic_buffers[1]) {
                    result.mic_buffers[1][i] = right_sample;
                }
                if (result.mic_buffers[2]) {
                    result.mic_buffers[2][i] = left_sample;
                }
                if (result.mic_buffers[3]) {
                    result.mic_buffers[3][i] = right_sample;
                }
            }
        }
    }

    result.status = ESP_OK;
    return result;
}

void audio_es_tools::compute_split_channel_quality(const channel_split_result_t& split_result,
                                                   mic_channel_quality_t quality[4])
{
    const size_t samples = split_result.samples_per_channel;

    for (int i = 0; i < 4; ++i) {
        mic_channel_quality_t& out = quality[i];
        out.available = (split_result.mic_buffers[i] != nullptr);
        out.sample_count = out.available ? samples : 0;
        out.min_value = out.available ? INT16_MAX : 0;
        out.max_value = out.available ? INT16_MIN : 0;
        out.average_abs_amplitude = 0;
        out.rms_db = -96.0;
        out.zero_percent = 0.0;
        out.clipped_percent = 0.0;

        if (!out.available || samples == 0) {
            if (out.available) {
                out.min_value = 0;
                out.max_value = 0;
            }
            continue;
        }

        const int16_t* data = split_result.mic_buffers[i];
        int64_t sum_abs = 0;
        size_t zero_count = 0;
        size_t clipped_count = 0;
        long double sum_square = 0.0;

        for (size_t sample_index = 0; sample_index < samples; ++sample_index) {
            int16_t sample = data[sample_index];
            if (sample == 0) {
                zero_count++;
            }
            if (sample == INT16_MAX || sample == INT16_MIN) {
                clipped_count++;
            }
            sum_abs += llabs(static_cast<long long>(sample));
            if (sample < out.min_value) {
                out.min_value = sample;
            }
            if (sample > out.max_value) {
                out.max_value = sample;
            }
            long double v = static_cast<long double>(sample);
            sum_square += v * v;
        }

        out.average_abs_amplitude = static_cast<int32_t>(sum_abs / static_cast<int64_t>(samples));
        const long double mean_square = sum_square / static_cast<long double>(samples);
        const double mean_square_double = static_cast<double>(mean_square);
        const double rms_linear = (mean_square_double > 0.0) ? sqrt(mean_square_double) : 0.0;
        out.rms_db = (rms_linear > 0.0) ? (20.0 * log10(rms_linear / 32768.0)) : -96.0;
        out.zero_percent = (samples > 0) ? (static_cast<double>(zero_count) * 100.0 / static_cast<double>(samples)) : 0.0;
        out.clipped_percent = (samples > 0) ? (static_cast<double>(clipped_count) * 100.0 / static_cast<double>(samples)) : 0.0;
    }
}

// 确保输出文件的父目录存在，如果不存在则递归创建
static esp_err_t ensure_parent_directories(const char* filepath)
{
    if (!filepath) {
        return ESP_ERR_INVALID_ARG;
    }

    const char* last_slash = strrchr(filepath, '/');
    if (!last_slash) {
        return ESP_OK; // 没有目录部分
    }

    size_t dir_len = static_cast<size_t>(last_slash - filepath);
    if (dir_len == 0) {
        return ESP_OK; // 根目录，无需创建
    }

    std::string dir_path(filepath, dir_len);
    if (dir_path.empty() || dir_path == "." || dir_path == "/") {
        return ESP_OK;
    }

    std::string current_path;
    size_t index = 0;
    if (!dir_path.empty() && dir_path[0] == '/') {
        current_path = "/";
        index = 1;
    }

    while (index <= dir_path.size()) {
        size_t next_sep = dir_path.find('/', index);
        size_t fragment_len = (next_sep == std::string::npos) ? (dir_path.size() - index) : (next_sep - index);
        std::string fragment = dir_path.substr(index, fragment_len);

        if (!fragment.empty()) {
            if (!current_path.empty() && current_path.back() != '/') {
                current_path += '/';
            }
            current_path += fragment;

            struct stat st {};
            if (stat(current_path.c_str(), &st) == 0) {
                if (!S_ISDIR(st.st_mode)) {
                    ESP_LOGE(TAG, "%s exists but is not a directory", current_path.c_str());
                    return ESP_ERR_INVALID_STATE;
                }
            } else {
                if (mkdir(current_path.c_str(), 0775) != 0) {
                    if (errno == EEXIST) {
                        continue;
                    }

                    if (stat(current_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
                        continue;
                    }

                    ESP_LOGE(TAG, "Failed to create directory %s (errno=%d, %s)",
                             current_path.c_str(), errno, strerror(errno));
                    return ESP_FAIL;
                }

                ESP_LOGI(TAG, "Created directory: %s", current_path.c_str());
            }
        }

        if (next_sep == std::string::npos) {
            break;
        }
        index = next_sep + 1;
    }

    return ESP_OK;
}

/**
 * 音频系统架构说明：
 * 
 * 本音频工具类采用模块化设计，将功能拆分为以下几个部分：
 * 
 * 1. audio_es_tools.cpp (主文件)
 *    - 系统级初始化和配置管理
 *    - I2S通道创建和管理  
 *    - 通用音频播放和测试功能
 *    - 配置参数的getter/setter
 *    - 为未来接入更多设备提供统一接口
 * 
 * 2. audio_es_es8311.cpp 
 *    - ES8311 DAC专用初始化和去初始化
 *    - ES8311睡眠管理
 *    - ES8311特定的配置和控制
 * 
 * 3. audio_es_es7210.cpp
 *    - ES7210 ADC专用初始化和去初始化  
 *    - ES7210睡眠管理
 *    - ES7210特定的配置和控制
 * 
 * 未来扩展新音频设备时，只需：
 * 1. 创建对应的 audio_es_xxx.cpp 文件
 * 2. 在主类中添加相应的初始化接口
 * 3. 确保与现有I2S管理系统兼容
 * 
 * 这种设计提供了良好的模块隔离和扩展性。
 */

// 将指定 GPIO 配置为高阻态（输入，去掉上下拉）
static inline void _gpio_set_high_z(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC || pin == I2S_GPIO_UNUSED) {
        return;
    }
    // 复位到默认状态，再显式设置为输入并关闭上下拉，确保高阻态
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_pullup_dis(pin);
    gpio_pulldown_dis(pin);
}

// 构造函数（默认参数）
audio_es_tools::audio_es_tools()
{
    ESP_LOGI(TAG, "audio_es_tools object created with default parameters");
    play_dev = NULL;
    record_dev = NULL;
    tx_handle = NULL;
    rx_handle = NULL;
    i2c_bus_handle = NULL;
    i2s_user_count = 0;
    
    // 创建互斥锁
    audio_mutex = xSemaphoreCreateMutex();
    if (!audio_mutex) {
        ESP_LOGE(TAG, "Failed to create audio mutex!");
    }
    
    // 初始化状态标志
    es8311_initialized = false;
    es7210_initialized = false;
    system_initialized = false;
    es8311_sleeping = false;
    es7210_sleeping = false;
    
    // 初始化I2S配置状态
    tx_configured = false;
    rx_configured = false;
    
    // 设置默认引脚配置
    i2s_bck_pin = I2S_BCLK_PIN;
    i2s_mck_pin = I2S_MCLK_PIN;
    i2s_data_in_pin = I2S_DATA_IN_PIN;
    i2s_data_out_pin = I2S_DATA_OUT_PIN;
    i2s_ws_pin = I2S_DATA_WS_PIN;
    pa_pin = PA_PIN;
    
    // 初始化默认音频响度配置
    volume = 80.0;
    record_gain = 30.0;
    
    // 初始化默认麦克风通道配置
    mic_channels = AUDIO_MIC_CHANNEL_12;
    playback_task_handle = nullptr;
}

// 构造函数（带参数）
audio_es_tools::audio_es_tools(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                               gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin)
{
    ESP_LOGI(TAG, "audio_es_tools object created with custom parameters");
    play_dev = NULL;
    record_dev = NULL;
    tx_handle = NULL;
    rx_handle = NULL;
    i2c_bus_handle = NULL;
    i2s_user_count = 0;
    
    // 初始化状态标志
    es8311_initialized = false;
    es7210_initialized = false;
    system_initialized = false;
    es8311_sleeping = false;
    es7210_sleeping = false;
    
    // 初始化I2S配置状态
    tx_configured = false;
    rx_configured = false;
    
    // 设置自定义引脚配置
    i2s_bck_pin = bck_pin;
    i2s_mck_pin = mck_pin;
    i2s_data_in_pin = data_in_pin;
    i2s_data_out_pin = data_out_pin;
    i2s_ws_pin = ws_pin;
    this->pa_pin = pa_pin;
    
    // 初始化默认音频响度配置
    volume = 80.0;
    record_gain = 30.0;
    
    // 初始化默认麦克风通道配置
    mic_channels = AUDIO_MIC_CHANNEL_123;
    playback_task_handle = nullptr;
}

// 析构函数
audio_es_tools::~audio_es_tools()
{
    if (playback_task_handle) {
        ESP_LOGW(TAG, "Waiting for playback task to finish before destruction");
        while (playback_task_handle) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    ESP_LOGI(TAG, "audio_es_tools object destroyed");
    // 清理音频资源
    audio_system_deinit();
    
    // 销毁互斥锁
    if (audio_mutex) {
        vSemaphoreDelete(audio_mutex);
        audio_mutex = nullptr;
    }
}

// 内部辅助：确保 I2S 通道存在
esp_err_t audio_es_tools::ensure_i2s_channel()
{
    if (tx_handle && rx_handle) return ESP_OK;
    return i2s_channel_init();
}

void audio_es_tools::incr_i2s_user()
{
    i2s_user_count++;
    ESP_LOGD(TAG, "I2S user ++ => %d", i2s_user_count);
}

void audio_es_tools::decr_i2s_user()
{
    if (i2s_user_count > 0) {
        i2s_user_count--;
        ESP_LOGD(TAG, "I2S user -- => %d", i2s_user_count);
    }
    try_release_i2s();
}

void audio_es_tools::try_release_i2s()
{
    if (suppress_release) {
        ESP_LOGD(TAG, "Suppressing I2S release (system deinit in progress)");
        return;
    }
    if (i2s_user_count == 0) {
        ESP_LOGI(TAG, "No codec uses I2S anymore, releasing channels");
        if (tx_configured) { i2s_tx_deinit(); tx_configured = false; }
        if (rx_configured) { i2s_rx_deinit(); rx_configured = false; }
        i2s_channel_deinit();
    }
}

esp_err_t audio_es_tools::i2s_channel_init()
{
    if (tx_handle && rx_handle) {
        ESP_LOGW(TAG, "I2S channels already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2S channels (TX and RX) with port %d, %d Hz, %d bits...", 
             i2s_port_num, 
             (int)sample_rate, 
             (int)bits_per_sample);
    
    esp_err_t ret = ESP_OK;
    
    // 1. 创建 I2S 通道（同时创建TX和RX）
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port_num, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = 256;
    ret = i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channels: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S channels created successfully (channel config will be set individually)");
    return ESP_OK;
}

esp_err_t audio_es_tools::i2s_channel_deinit()
{
    if (!tx_handle && !rx_handle) {
        ESP_LOGW(TAG, "I2S channels not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing I2S channels...");
    
    // 禁用并删除通道
    if (tx_handle) {
        if (tx_configured) {
            esp_err_t ret = i2s_channel_disable(tx_handle);
            if (ret != ESP_OK) {
                if (ret == ESP_ERR_INVALID_STATE) {
                    ESP_LOGD(TAG, "I2S TX channel was already disabled");
                } else {
                    ESP_LOGW(TAG, "Failed to disable I2S TX channel: %s", esp_err_to_name(ret));
                }
            }
        }
        i2s_del_channel(tx_handle);
        tx_handle = NULL;
    }
    
    if (rx_handle) {
        if (rx_configured) {
            esp_err_t ret = i2s_channel_disable(rx_handle);
            if (ret != ESP_OK) {
                if (ret == ESP_ERR_INVALID_STATE) {
                    ESP_LOGD(TAG, "I2S RX channel was already disabled");
                } else {
                    ESP_LOGW(TAG, "Failed to disable I2S RX channel: %s", esp_err_to_name(ret));
                }
            }
        }
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
    }
    
    // 释放 I2S 通道后，将相关引脚全部置为高阻态，避免继续驱动总线
    _gpio_set_high_z(i2s_mck_pin);
    _gpio_set_high_z(i2s_bck_pin);
    _gpio_set_high_z(i2s_ws_pin);
    _gpio_set_high_z(i2s_data_out_pin);
    _gpio_set_high_z(i2s_data_in_pin);
    _gpio_set_high_z(pa_pin);
    ESP_LOGI(TAG, "I2S GPIOs set to high-Z (MCK:%d, BCK:%d, WS:%d, DOUT:%d, DIN:%d PA:%d)",
             i2s_mck_pin, i2s_bck_pin, i2s_ws_pin, i2s_data_out_pin, i2s_data_in_pin, pa_pin);
    
    ESP_LOGI(TAG, "I2S channels deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::i2s_tx_init()
{
    if (!tx_handle) {
        ESP_LOGE(TAG, "I2S TX handle not available, call i2s_channel_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Configuring I2S TX channel with %s, %d Hz, %d bits...", 
             (tx_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
             (int)sample_rate, 
             (int)bits_per_sample);
    
    esp_err_t ret = ESP_OK;
    
    // 根据声道数量设置slot模式
    i2s_slot_mode_t slot_mode = (tx_channels == AUDIO_CHANNELS_MONO) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    
    gpio_num_t tx_dout_pin = i2s_cross_data_pins ? i2s_data_in_pin : i2s_data_out_pin;
    gpio_num_t tx_din_pin  = i2s_cross_data_pins ? i2s_data_out_pin : I2S_GPIO_UNUSED;
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_sample, slot_mode),
        .gpio_cfg = {
            .mclk = i2s_mck_pin,
            .bclk = i2s_bck_pin,
            .ws = i2s_ws_pin,
            .dout = tx_dout_pin,
            .din = tx_din_pin,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    ret = i2s_channel_init_std_mode(tx_handle, &tx_std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S TX standard mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 启用 I2S TX 通道
    ret = i2s_channel_enable(tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S TX channel configured successfully");
    tx_configured = true;
    return ESP_OK;
}

esp_err_t audio_es_tools::i2s_tx_deinit()
{
    if (!tx_handle) {
        ESP_LOGW(TAG, "I2S TX not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disabling I2S TX channel...");
    
    if (tx_configured) {
        esp_err_t ret = i2s_channel_disable(tx_handle);
        if (ret != ESP_OK) {
            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG, "I2S TX channel was already disabled");
            } else {
                ESP_LOGW(TAG, "Failed to disable I2S TX channel: %s", esp_err_to_name(ret));
            }
        }
        tx_configured = false;
    }
    
    ESP_LOGI(TAG, "I2S TX channel disabled successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::i2s_rx_init()
{
    if (!rx_handle) {
        ESP_LOGE(TAG, "I2S RX handle not available, call i2s_channel_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t requested_channels = static_cast<uint8_t>(rx_channels);
    if (requested_channels == 0) {
        requested_channels = 1;
    }

    const bool use_tdm = es7210_use_tdm;
    if (use_tdm) {
        ESP_LOGW(TAG, "es7210_use_tdm=TRUE but RX is forced to standard I2S configuration");
    }

    const char *mode_desc = (rx_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO";

    ESP_LOGI(TAG, "Configuring I2S RX channel with %s (%u %s), %d Hz, %d bits...",
             mode_desc,
             requested_channels,
             (requested_channels > 1) ? "channels" : "channel",
             (int)sample_rate,
             (int)bits_per_sample);

    gpio_num_t rx_din_pin = i2s_cross_data_pins ? i2s_data_out_pin : i2s_data_in_pin;
    gpio_num_t rx_dout_pin = i2s_cross_data_pins ? i2s_data_in_pin : I2S_GPIO_UNUSED;

    esp_err_t ret = ESP_OK;

    // 标准 I2S 接收配置
    i2s_slot_mode_t slot_mode = (rx_channels == AUDIO_CHANNELS_MONO) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_sample, slot_mode),
        .gpio_cfg = {
            .mclk = i2s_mck_pin,
            .bclk = i2s_bck_pin,
            .ws = i2s_ws_pin,
            .dout = rx_dout_pin,
            .din = rx_din_pin,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(rx_handle, &rx_std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S RX standard mode: %s", esp_err_to_name(ret));
        return ret;
    }

    rx_tdm_slot_count = 0;

    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S RX channel configured successfully");
    rx_configured = true;
    return ESP_OK;
}

esp_err_t audio_es_tools::i2s_rx_deinit()
{
    if (!rx_handle) {
        ESP_LOGW(TAG, "I2S RX not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disabling I2S RX channel...");
    
    // 只有在通道已配置的情况下才尝试禁用
    if (rx_configured) {
        esp_err_t ret = i2s_channel_disable(rx_handle);
        if (ret != ESP_OK) {
            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG, "I2S RX channel was already disabled");
            } else {
                ESP_LOGW(TAG, "Failed to disable I2S RX channel: %s", esp_err_to_name(ret));
            }
        }
        rx_configured = false;
    }
    
    rx_tdm_slot_count = 0;

    ESP_LOGI(TAG, "I2S RX channel disabled successfully");
    return ESP_OK;
}

// ES8311和ES7210的具体实现已移至对应的独立源文件中
// ES8311相关函数实现在 audio_es_es8311.cpp
// ES7210相关函数实现在 audio_es_es7210.cpp

esp_err_t audio_es_tools::audio_system_init(i2c_master_bus_handle_t i2c_bus_handle, i2s_port_t i2s_port_num, audio_sample_rate_t sample_rate, i2s_data_bit_width_t bits_per_sample)
{
    if (system_initialized) {
        ESP_LOGW(TAG, "Audio system already initialized");
        return ESP_OK;
    }

    // 验证传入的I2C总线句柄
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle passed to audio_system_init");
        return ESP_ERR_INVALID_ARG;
    }

    // 存储I2C总线句柄和I2S配置参数（不包括声道配置）
    this->i2c_bus_handle = i2c_bus_handle;
    this->i2s_port_num = i2s_port_num;
    this->sample_rate = sample_rate;
    this->bits_per_sample = bits_per_sample;

    ESP_LOGI(TAG, "Initializing audio system with I2C bus handle, I2S port %d, %d Hz, %d bits...", 
             i2s_port_num, 
             (int)sample_rate, 
             (int)bits_per_sample);
    
    ESP_LOGI(TAG, "Note: Channel configuration will be set individually by es8311_init() and es7210_init()");
    
    // 根据需要初始化相应的模块
    esp_err_t ret = ESP_OK;

    // 按需只准备 I2S 通道，codec 由外部手动调用 init，以实现解耦
    ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 创建共享的 I2S 数据接口（避免重复创建导致冲突）
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.rx_handle = rx_handle;
    i2s_cfg.tx_handle = tx_handle;
    
    shared_i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!shared_i2s_data_if) {
        ESP_LOGE(TAG, "Failed to create shared I2S data interface");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Shared I2S data interface created successfully");
    
    system_initialized = true;
    
    ESP_LOGI(TAG, "Audio system initialized successfully with I2S port %d, %d Hz, %d bits", 
             i2s_port_num,
             (int)sample_rate,
             (int)bits_per_sample);
    return ESP_OK;
}

esp_err_t audio_es_tools::audio_system_deinit()
{
    // 获取互斥锁保护
    if (audio_mutex && xSemaphoreTake(audio_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire audio mutex for deinit, force deinit anyway");
        // 继续执行，但可能有风险
    }
    
    if (!system_initialized) {
        ESP_LOGW(TAG, "Audio system not initialized");
        if (audio_mutex) xSemaphoreGive(audio_mutex);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing audio system...");
    suppress_release = true;  // 避免在两个 codec 先后 deinit 间隙多次释放 I2S
    
    // 去初始化所有已初始化的模块（各自安全判断）
    if (es8311_initialized) es8311_deinit();
    if (es7210_initialized) es7210_deinit();
    
    // 释放共享的 I2S 数据接口
    if (shared_i2s_data_if) {
        audio_codec_delete_data_if(shared_i2s_data_if);
        shared_i2s_data_if = nullptr;
        ESP_LOGI(TAG, "Shared I2S data interface deleted");
    }
    
    // 现在统一处理 I2S 释放
    suppress_release = false;
    try_release_i2s();
    // 若两个都释放了会自动释放 I2S
    
    system_initialized = false;
    
    ESP_LOGI(TAG, "Audio system deinitialized successfully");
    
    // 释放互斥锁
    if (audio_mutex) xSemaphoreGive(audio_mutex);
    return ESP_OK;
}

// ES8311和ES7210的睡眠功能实现已移至对应的独立源文件中

esp_err_t audio_es_tools::audio_system_sleep()
{
    if (!system_initialized) {
        ESP_LOGE(TAG, "Audio system not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Putting audio system to sleep...");
    
    // 调用各个模块的睡眠函数（实现在对应的专用文件中）
    if (es8311_initialized) {
        es8311_sleep();  // 实现在 audio_es_es8311.cpp
    }
    
    if (es7210_initialized) {
        es7210_sleep();  // 实现在 audio_es_es7210.cpp  
    }
    
    ESP_LOGI(TAG, "Audio system sleep mode enabled");
    return ESP_OK;
}

esp_err_t audio_es_tools::record_test(uint32_t record_duration_ms)
{
    if (!es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized, cannot record");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting %lu ms record test...", record_duration_ms);

    // 分配录音缓冲区（动态参数）
    const size_t sr = (size_t)sample_rate;
    size_t channels = 0;
    uint8_t mic_mask = static_cast<uint8_t>(mic_channels);
    for (int i = 0; i < 4; ++i) if (mic_mask & (1 << i)) channels++;
    if (channels == 0) channels = 1;
    const size_t bps = (size_t)bits_per_sample;
    const size_t bytes_per_sample = bps / 8;
    const size_t buffer_size = (sr * channels * bytes_per_sample * record_duration_ms) / 1000;
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate record buffer (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Record buffer allocated: %zu bytes", buffer_size);
    ESP_LOGI(TAG, "Sample rate: %zu Hz, Channels: %zu, Bits: %zu", sr, channels, bps);

    // 使用官方推荐的固定块大小读取方式
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    const size_t BLOCK_SIZE = 512; // 使用固定块大小
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_ms + 300); // 适度超时余量
    
    while (bytes_read < buffer_size) {
        // 计算本次读取的块大小
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error: %s", esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 检查是否超时
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached (%lu ms), stop early", record_duration_ms);
            break;
        }
    }
    TickType_t end_time = xTaskGetTickCount();
    
    uint32_t actual_duration = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Record completed successfully");
    ESP_LOGI(TAG, "Requested duration: %lu ms, Actual duration: %lu ms", record_duration_ms, actual_duration);
    ESP_LOGI(TAG, "Expected bytes: %zu, Actual bytes read: %zu (%.1f%%)", buffer_size, bytes_read, (buffer_size? (bytes_read * 100.0 / buffer_size):0.0));
    
    // 简单的音频数据分析
    if (bytes_read > 0) {
    int16_t *samples = (int16_t *)record_buffer;
    size_t total_samples = bytes_read / sizeof(int16_t); // 所有通道样本点
    size_t frame_count = (channels > 0) ? total_samples / channels : total_samples;
        
        // 计算音频信号的基本统计信息
        int32_t sum = 0;
        int16_t min_val = INT16_MAX;
        int16_t max_val = INT16_MIN;
        
        for (size_t i = 0; i < total_samples; i++) {
            int16_t sample = samples[i];
            sum += abs(sample);
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        int32_t avg_amplitude = (total_samples > 0) ? (sum / (int32_t)total_samples) : 0;
        ESP_LOGI(TAG, "Audio analysis - Frames: %zu, Total samples: %zu, Avg amplitude: %ld, Range: [%d, %d]", 
                 frame_count, total_samples, avg_amplitude, min_val, max_val);
        
        // 检查是否捕获到有效音频信号
        if (avg_amplitude > 100) {
            ESP_LOGI(TAG, "Valid audio signal detected!");
        } else {
            ESP_LOGW(TAG, "Low audio signal detected - check microphone connection");
        }
    }

    // 清理资源
    free(record_buffer);
    
    ESP_LOGI(TAG, "Record test completed successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::record_and_play_test(uint32_t record_duration_seconds)
{
    if (!es7210_initialized || !es8311_initialized) {
        ESP_LOGE(TAG, "Audio devices not initialized (ES7210 or ES8311)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev || !play_dev) {
        ESP_LOGE(TAG, "Record or playback device not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Record and Play Test (Single Shot) ===");
    ESP_LOGI(TAG, "Duration: %lu seconds", record_duration_seconds);
    
    // 获取当前音频格式信息（使用录音设备的声道配置）
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)this->sample_rate;
    fs.channel = (this->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    fs.bits_per_sample = (uint32_t)this->bits_per_sample;
    
    ESP_LOGI(TAG, "Audio format: %lu Hz, %u channels, %u bits",
             fs.sample_rate, fs.channel, fs.bits_per_sample);
    
    // 计算缓冲区大小
    size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    size_t buffer_size = (size_t)fs.sample_rate * fs.channel * bytes_per_sample * record_duration_seconds;
    
    ESP_LOGI(TAG, "Allocating buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate memory for recording");
        ESP_LOGE(TAG, "Required: %zu bytes (%.2f KB), Free: %zu bytes (%.2f KB)",
                buffer_size, buffer_size / 1024.0f, free_heap, free_heap / 1024.0f);
        return ESP_ERR_NO_MEM;
    }
    
    memset(record_buffer, 0, buffer_size);
    
    // ========== Phase 1: Recording ==========
    ESP_LOGI(TAG, "Phase 1: Recording %lu seconds...", record_duration_seconds);
    
    const size_t BLOCK_SIZE = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    
    while (bytes_read < buffer_size) {
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu bytes: %s", bytes_read, esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 进度提示 (每25%打印一次)
        static size_t last_progress = 0;
        size_t progress = (bytes_read * 100) / buffer_size;
        if (progress >= last_progress + 25) {
            ESP_LOGI(TAG, "Recording progress: %zu%%", progress);
            last_progress = progress;
        }
        
        // 超时检查
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached, stopping early");
            break;
        }
    }
    
    TickType_t end_time = xTaskGetTickCount();
    uint32_t actual_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Recording completed!");
    ESP_LOGI(TAG, "Requested: %lu sec, Actual: %.2f sec",
             record_duration_seconds, actual_duration_ms / 1000.0f);
    ESP_LOGI(TAG, "Bytes read: %zu / %zu (%.1f%%)",
             bytes_read, buffer_size, (buffer_size > 0) ? (bytes_read * 100.0f / buffer_size) : 0.0f);
    
    // 简单音频分析
    if (bytes_read > 0) {
        int16_t *samples = (int16_t *)record_buffer;
        size_t sample_count = bytes_read / sizeof(int16_t);
        
        int32_t sum = 0;
        int16_t min_val = INT16_MAX;
        int16_t max_val = INT16_MIN;
        
        for (size_t i = 0; i < sample_count; i++) {
            int16_t sample = samples[i];
            sum += abs(sample);
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        
        int32_t avg_amplitude = (sample_count > 0) ? (sum / (int32_t)sample_count) : 0;
        
        ESP_LOGI(TAG, "Audio analysis - Samples: %zu, Avg amplitude: %ld, Range: [%d, %d]",
                 sample_count, avg_amplitude, min_val, max_val);
        
        if (avg_amplitude > 100) {
            ESP_LOGI(TAG, "Valid audio signal detected [OK]");
        } else {
            ESP_LOGW(TAG, "Low audio signal - check microphone");
        }
    }
    
    // 录音和播放之间短暂延迟
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // ========== Phase 2: Playback ==========
    ESP_LOGI(TAG, "Phase 2: Playing recorded audio...");
    
    start_time = xTaskGetTickCount();
    
    // 使用 play_audio_buffer 播放录音，支持自适应格式
    esp_err_t ret = play_audio_buffer(
        record_buffer,
        bytes_read,
        fs.sample_rate,
        (fs.channel == 1) ? AUDIO_CHANNELS_MONO : AUDIO_CHANNELS_STEREO,
        (i2s_data_bit_width_t)fs.bits_per_sample,
        AUDIO_PLAYBACK_BLOCKING
    );
    
    end_time = xTaskGetTickCount();
    uint32_t playback_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        free(record_buffer);
        return ret;
    }
    
    ESP_LOGI(TAG, "Playback completed!");
    ESP_LOGI(TAG, "Duration: %.2f sec, Bytes played: %zu", 
             playback_duration_ms / 1000.0f, bytes_read);
    
    // ========== Phase 3: 生成详细统计报告 ==========
    ESP_LOGI(TAG, "=== Analysis Results ===");
    
    if (bytes_read > 0 && fs.channel > 0) {
        int16_t *samples = (int16_t *)record_buffer;
        size_t total_samples = bytes_read / sizeof(int16_t);
        size_t frames = total_samples / fs.channel;
        
        // 为每个通道统计信息
        for (uint32_t ch = 0; ch < fs.channel; ch++) {
            // 通道标识
            const char* ch_name = (ch == 0) ? "Microphone" : "Loopback";
            ESP_LOGI(TAG, "=== CH%u (%s) Statistics ===", ch + 1, ch_name);
            
            // 统计变量
            int64_t sum_squares = 0;  // 用于计算RMS
            int16_t min_val = INT16_MAX;
            int16_t max_val = INT16_MIN;
            size_t zero_count = 0;
            size_t clipped_count = 0;
            
            // 遍历该通道的所有样本
            for (size_t frame = 0; frame < frames; frame++) {
                size_t idx = frame * fs.channel + ch;
                if (idx >= total_samples) break;
                
                int16_t sample = samples[idx];
                
                // RMS 计算
                int32_t sample_32 = (int32_t)sample;
                sum_squares += (int64_t)(sample_32 * sample_32);
                
                // 峰值检测
                if (sample < min_val) min_val = sample;
                if (sample > max_val) max_val = sample;
                
                // 零样本计数 (绝对值小于等于 1)
                if (sample >= -1 && sample <= 1) {
                    zero_count++;
                }
                
                // 削波检测 (接近最大值)
                if (sample <= -32767 || sample >= 32767) {
                    clipped_count++;
                }
            }
            
            // 计算 RMS Level (dB)
            double rms_linear = 0.0;
            if (frames > 0) {
                double mean_square = (double)sum_squares / (double)frames;
                rms_linear = sqrt(mean_square);
            }
            // 转换为 dB (参考值为 32768)
            double rms_db = (rms_linear > 0) ? (20.0 * log10(rms_linear / 32768.0)) : -96.0;
            
            // 计算百分比
            double zero_percent = (frames > 0) ? (zero_count * 100.0 / frames) : 0.0;
            double clipped_percent = (frames > 0) ? (clipped_count * 100.0 / frames) : 0.0;
            
            // 打印统计结果 (格式类似图片)
            ESP_LOGI(TAG, "  RMS Level: %.1f dB", rms_db);
            ESP_LOGI(TAG, "  Peak: %d to %d", min_val, max_val);
            ESP_LOGI(TAG, "  Zero samples: %zu (%.0f%%)", zero_count, zero_percent);
            ESP_LOGI(TAG, "  Clipped samples: %zu (%.2f%%)", clipped_count, clipped_percent);
        }
    }
    
    ESP_LOGI(TAG, "=== Analysis Results ===");
    
    // 清理资源
    free(record_buffer);
    
    // 清空播放管线，避免残留音频
    esp_err_t clr_ret = clear_audio_pipeline(120);
    if (clr_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear playback pipeline: %s", esp_err_to_name(clr_ret));
    }
    
    ESP_LOGI(TAG, "=== Record and Play Test Completed ===");
    return ESP_OK;
}

esp_err_t audio_es_tools::record_and_play_test_with_channel_select(uint32_t record_duration_seconds, audio_mic_channel_t target_mic_channel, bool analysis_only)
{
    // 仅分析模式只需要ES7210初始化
    if (!es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 正常播放模式需要两个设备都初始化
    if (!analysis_only && !es8311_initialized) {
        ESP_LOGE(TAG, "ES8311 not initialized (required for playback mode)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!analysis_only && !play_dev) {
        ESP_LOGE(TAG, "Playback device not available (required for playback mode)");
        return ESP_ERR_INVALID_STATE;
    }

    // 验证目标麦克风通道是否为单一通道（不能是组合通道）
    uint8_t target_mic_value = (uint8_t)target_mic_channel;
    bool is_single_channel = (target_mic_value == 0x01 || target_mic_value == 0x02 || 
                              target_mic_value == 0x04 || target_mic_value == 0x08);
    
    if (!is_single_channel) {
        ESP_LOGE(TAG, "target_mic_channel must be a single microphone (AUDIO_MIC_CHANNEL_1/2/3/4), got: 0x%02X", 
                 target_mic_value);
        return ESP_ERR_INVALID_ARG;
    }

    // 将 audio_mic_channel_t 转换为索引 (0-3)
    uint8_t target_channel = 0;
    switch (target_mic_channel) {
        case AUDIO_MIC_CHANNEL_1: target_channel = 0; break;
        case AUDIO_MIC_CHANNEL_2: target_channel = 1; break;
        case AUDIO_MIC_CHANNEL_3: target_channel = 2; break;
        case AUDIO_MIC_CHANNEL_4: target_channel = 3; break;
        default:
            ESP_LOGE(TAG, "Invalid target_mic_channel: 0x%02X", target_mic_value);
            return ESP_ERR_INVALID_ARG;
    }

    // 检查目标通道是否已启用
    if (!(mic_channels & target_mic_value)) {
        ESP_LOGE(TAG, "Target MIC%u (0x%02X) is not enabled in current config (0x%02X)", 
                 target_channel + 1, target_mic_value, mic_channels);
        ESP_LOGE(TAG, "Please initialize ES7210 with the target microphone enabled");
        return ESP_ERR_INVALID_ARG;
    }

    // 计算启用的麦克风数量
    uint8_t mic_count = 0;
    for (int i = 0; i < 4; i++) {
        if (mic_channels & (1 << i)) {
            mic_count++;
        }
    }

    // 直接参考ES7210的TDM状态
    bool is_tdm_mode = es7210_use_tdm;
    
    ESP_LOGI(TAG, "=== Record and %s Test (Channel Select) ===", analysis_only ? "Analysis" : "Play");
    ESP_LOGI(TAG, "Duration: %lu seconds, Target: MIC%u (0x%02X)", 
             record_duration_seconds, target_channel + 1, target_mic_value);
    ESP_LOGI(TAG, "Mode: %s (%u mics enabled: 0x%02X), Analysis only: %s", 
             is_tdm_mode ? "TDM" : "Standard I2S", mic_count, mic_channels,
             analysis_only ? "YES" : "NO");
    
    // 获取当前音频格式信息
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)this->sample_rate;
    fs.bits_per_sample = (uint32_t)this->bits_per_sample;
    
    // 确定I2S通道数配置
    if (is_tdm_mode) {
        fs.channel = rx_tdm_slot_count ? rx_tdm_slot_count : 4;  // TDM使用实际配置的slot数量（默认4）
    } else {
        fs.channel = (this->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    }
    
    size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    
    ESP_LOGI(TAG, "Audio format: %lu Hz, %u channels, %u bits per sample",
             fs.sample_rate, fs.channel, fs.bits_per_sample);
    
    // 计算录音缓冲区大小
    size_t buffer_size = (size_t)fs.sample_rate * fs.channel * bytes_per_sample * record_duration_seconds;
    
    if (is_tdm_mode) {
        ESP_LOGI(TAG, "TDM buffer calculation:");
        ESP_LOGI(TAG, "  - Total buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
        ESP_LOGI(TAG, "  - TDM slots: %u", fs.channel);
        ESP_LOGI(TAG, "  - Active mics: %u (0x%02X)", mic_count, mic_channels);
        ESP_LOGI(TAG, "  - Data rate: %.2f KB/s", 
                 (float)(fs.sample_rate * fs.channel * bytes_per_sample) / 1024.0f);
    } else {
        ESP_LOGI(TAG, "Standard I2S buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    }
    
    ESP_LOGI(TAG, "Allocating buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate memory for recording");
        ESP_LOGE(TAG, "Required: %zu bytes (%.2f KB), Free: %zu bytes (%.2f KB)",
                buffer_size, buffer_size / 1024.0f, free_heap, free_heap / 1024.0f);
        return ESP_ERR_NO_MEM;
    }
    
    memset(record_buffer, 0, buffer_size);
    
    // ========== Phase 1: Recording ==========
    ESP_LOGI(TAG, "Phase 1: Recording all TDM channels...");
    
    const size_t BLOCK_SIZE = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    
    while (bytes_read < buffer_size) {
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu bytes: %s", bytes_read, esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 进度提示
        static size_t last_progress = 0;
        size_t progress = (bytes_read * 100) / buffer_size;
        if (progress >= last_progress + 25) {
            ESP_LOGI(TAG, "Recording progress: %zu%%", progress);
            last_progress = progress;
        }
        
        // 超时检查
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached, stopping early");
            break;
        }
    }
    
    TickType_t end_time = xTaskGetTickCount();
    uint32_t actual_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Recording completed! Actual: %.2f sec, Bytes: %zu",
             actual_duration_ms / 1000.0f, bytes_read);
    
    // ========== Phase 2: Channel Extraction ==========
    ESP_LOGI(TAG, "Phase 2: Splitting MIC data from %s stream...",
             is_tdm_mode ? "TDM" : "Standard I2S");

    channel_split_result_t split_result = split_recorded_channels(
        record_buffer,
        bytes_read,
        fs,
        is_tdm_mode,
        mic_channels);

    free(record_buffer);
    record_buffer = nullptr;

    if (split_result.status != ESP_OK) {
        ESP_LOGE(TAG, "Channel splitting failed: %s", esp_err_to_name(split_result.status));
        free_channel_split_result(split_result);
        return split_result.status;
    }

    int16_t* target_samples = split_result.mic_buffers[target_channel];
    if (!target_samples) {
        ESP_LOGE(TAG, "Target MIC%u buffer unavailable after splitting", target_channel + 1);
        free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    mic_channel_quality_t channel_quality[4] = {};
    compute_split_channel_quality(split_result, channel_quality);
    const mic_channel_quality_t& target_quality = channel_quality[target_channel];
    const size_t extracted_samples = target_quality.sample_count;
    const float target_duration_sec = extracted_samples
                                          ? (static_cast<float>(extracted_samples) / static_cast<float>(fs.sample_rate))
                                          : 0.0f;

    ESP_LOGI(TAG, "Extracted %zu samples from MIC%u (%.2f seconds)",
             extracted_samples, target_channel + 1, target_duration_sec);

    ESP_LOGI(TAG, "=== MIC%u Audio Analysis Report ===", target_channel + 1);
    ESP_LOGI(TAG, "  Total samples: %zu (%.2f seconds)", extracted_samples, target_duration_sec);
    ESP_LOGI(TAG, "  Average amplitude: %ld", static_cast<long>(target_quality.average_abs_amplitude));
    ESP_LOGI(TAG, "  Peak range: [%d, %d]", target_quality.min_value, target_quality.max_value);
    ESP_LOGI(TAG, "  RMS Level: %.1f dB", target_quality.rms_db);
    ESP_LOGI(TAG, "  Signal quality: %s", target_quality.average_abs_amplitude > 100 ? "GOOD" : "LOW");

    if (target_quality.average_abs_amplitude > 100) {
        ESP_LOGI(TAG, "Valid audio signal detected from MIC%u", target_channel + 1);
    } else {
        ESP_LOGW(TAG, "Low audio signal from MIC%u - check microphone connection/gain", target_channel + 1);
    }
    ESP_LOGI(TAG, "====================================");

    ESP_LOGI(TAG, "=== All Channel Quality Summary ===");
    for (int mic_index = 0; mic_index < 4; ++mic_index) {
        const mic_channel_quality_t& mic_quality = channel_quality[mic_index];
        if (!mic_quality.available) {
            continue;
        }

        const char* marker = (mic_index == target_channel) ? " (target)" : "";
        ESP_LOGI(TAG, "  MIC%u%s -> avg:%ld peak:[%d,%d] rms:%.1f dB signal:%s",
                 mic_index + 1,
                 marker,
                 static_cast<long>(mic_quality.average_abs_amplitude),
                 mic_quality.min_value,
                 mic_quality.max_value,
                 mic_quality.rms_db,
                 mic_quality.average_abs_amplitude > 100 ? "GOOD" : "LOW");
    }
    ESP_LOGI(TAG, "====================================");

    if (analysis_only) {
        ESP_LOGI(TAG, "Analysis-only mode: Skipping playback phase");
        free_channel_split_result(split_result);
        ESP_LOGI(TAG, "=== Channel Analysis Completed ===");
        return ESP_OK;
    }

    // 短暂延迟
    vTaskDelay(pdMS_TO_TICKS(500));

    // ========== Phase 3: Playback ==========
    ESP_LOGI(TAG, "Phase 3: Playing extracted MIC%u audio...", target_channel + 1);

    start_time = xTaskGetTickCount();

    const size_t bytes_to_write = extracted_samples * sizeof(int16_t);
    esp_err_t ret = play_audio_buffer(
        reinterpret_cast<const uint8_t*>(target_samples),
        bytes_to_write,
        static_cast<uint32_t>(fs.sample_rate),
        AUDIO_CHANNELS_MONO,
        I2S_DATA_BIT_WIDTH_16BIT,
        AUDIO_PLAYBACK_BLOCKING);

    end_time = xTaskGetTickCount();
    uint32_t playback_duration_ms = pdTICKS_TO_MS(end_time - start_time);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        free_channel_split_result(split_result);
        return ret;
    }

    ESP_LOGI(TAG, "Playback completed! Duration: %.2f sec, Bytes: %zu",
             playback_duration_ms / 1000.0f, bytes_to_write);

    free_channel_split_result(split_result);

    // 清空播放管线
    esp_err_t clr_ret = clear_audio_pipeline(120);
    if (clr_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear playback pipeline: %s", esp_err_to_name(clr_ret));
    }

    ESP_LOGI(TAG, "=== Channel Select Test Completed ===");
    return ESP_OK;
}

esp_err_t audio_es_tools::record_and_playback_test(uint32_t record_duration_seconds, bool loop_playback)
{
    if (!es7210_initialized || !es8311_initialized) {
        ESP_LOGE(TAG, "Audio devices not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev || !play_dev) {
        ESP_LOGE(TAG, "Record or playback device not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Record and playback test (%lu seconds, loop: %s) ===", 
             record_duration_seconds, loop_playback ? "YES" : "NO");
    
    // 获取当前音频格式信息（使用录音设备的声道配置）
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)this->sample_rate;
    fs.channel = (this->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    fs.bits_per_sample = (uint32_t)this->bits_per_sample;
        
    // 计算缓冲区大小
    size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    size_t record_bytes = (size_t)fs.sample_rate * fs.channel * bytes_per_sample * record_duration_seconds;
    uint8_t *data = (uint8_t *)malloc(record_bytes);
    if (data == NULL) {
        size_t free_heap = esp_get_free_heap_size();
        size_t required_kb = (record_bytes + 1023) / 1024;
        size_t free_kb = (free_heap + 1023) / 1024;
        size_t missing_kb = (free_heap < record_bytes) ? (required_kb - free_kb) : 0;

        ESP_LOGE(TAG, "Failed to allocate memory for recording");
        ESP_LOGE(TAG, "Required: %zu bytes (%zu KB), Free: %zu bytes (%zu KB), Missing: %zu KB",
                record_bytes, required_kb, free_heap, free_kb, missing_kb);
        return ESP_ERR_NO_MEM;
    }
    
    int buffer_size = (int)record_bytes;
    const size_t BLOCK_SIZE = 512; // 使用固定块大小
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 300);

    if (loop_playback) {
        ESP_LOGI(TAG, "Starting record-playback loop mode (each cycle: record %lu sec -> play %lu sec)...", 
                 record_duration_seconds, record_duration_seconds);
        
        // 录音-播放循环模式
        int cycle_count = 0;
        while (true) {
            cycle_count++;
            ESP_LOGI(TAG, "=== Cycle #%d START ===", cycle_count);
            
            // 清零缓冲区
            memset(data, 0, record_bytes);
            
            // 录音阶段
            ESP_LOGI(TAG, "Cycle #%d: Recording %lu seconds... (buffer: %d bytes)", 
                     cycle_count, record_duration_seconds, buffer_size);
            
            TickType_t start_time = xTaskGetTickCount();
            size_t bytes_read = 0;
            
            while (bytes_read < record_bytes) {
                // 计算本次读取的块大小
                size_t read_size = (record_bytes - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (record_bytes - bytes_read);
                
                esp_err_t ret = esp_codec_dev_read(record_dev, data + bytes_read, (int)read_size);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Cycle #%d recording failed: %s", cycle_count, esp_err_to_name(ret));
                    free(data);
                    return ESP_FAIL;
                }
                
                bytes_read += read_size;
                
                // 检查是否超时
                if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
                    ESP_LOGW(TAG, "Cycle #%d record timeout reached, stopping early", cycle_count);
                    break;
                }
            }
            
            ESP_LOGI(TAG, "Cycle #%d: Recording completed, bytes read: %zu", cycle_count, bytes_read);
            
            // 录音和播放之间的短暂间隔
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // 播放阶段
            ESP_LOGI(TAG, "Cycle #%d: Playing recorded audio...", cycle_count);
            
            // 使用 play_audio_buffer 播放录音，支持自适应格式
            esp_err_t write_ret = play_audio_buffer(
                data,
                bytes_read,
                fs.sample_rate,
                (fs.channel == 1) ? AUDIO_CHANNELS_MONO : AUDIO_CHANNELS_STEREO,
                (i2s_data_bit_width_t)fs.bits_per_sample,
                AUDIO_PLAYBACK_BLOCKING
            );
            
            if (write_ret != ESP_OK) {
                ESP_LOGE(TAG, "Cycle #%d playback failed: %s", cycle_count, esp_err_to_name(write_ret));
                // 播放失败不退出循环，继续下一轮录音
            } else {
                ESP_LOGI(TAG, "Cycle #%d: Playback completed -> bytes played: %zu", cycle_count, bytes_read);
            }
            
            // 清理音频管道，避免循环间的音频残留
            esp_err_t clear_ret = clear_audio_pipeline(50);
            if (clear_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to clear audio pipeline in cycle #%d", cycle_count);
            }
            
            ESP_LOGI(TAG, "=== Cycle #%d COMPLETED ===", cycle_count);
            
            // 循环间隔
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // 这里可以添加退出条件，比如检查按键或其他信号
            // 为了演示，我们执行3个录音-播放循环后自动退出
            // if (cycle_count >= 3) {
            //     ESP_LOGI(TAG, "Auto-stop after %d record-playback cycles for demonstration", cycle_count);
            //     break;
            // }
        }
        
        ESP_LOGI(TAG, "Record-playback loop completed (%d cycles)", cycle_count);
    } else {
        // 单次录音播放模式
        ESP_LOGI(TAG, "Single record-playback mode");
        
        // 清零缓冲区
        memset(data, 0, record_bytes);
        
        ESP_LOGI(TAG, "Start recording %lu seconds... (buffer: %d bytes)", record_duration_seconds, buffer_size);
            
        // 录音阶段 - 使用固定块大小循环读取
        TickType_t start_time = xTaskGetTickCount();
        size_t bytes_read = 0;
        
        while (bytes_read < record_bytes) {
            // 计算本次读取的块大小
            size_t read_size = (record_bytes - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (record_bytes - bytes_read);
            
            esp_err_t ret = esp_codec_dev_read(record_dev, data + bytes_read, (int)read_size);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Recording failed: %s", esp_err_to_name(ret));
                free(data);
                return ESP_FAIL;
            }
            
            bytes_read += read_size;
            
            // 检查是否超时
            if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
                ESP_LOGW(TAG, "Record timeout reached, stopping early");
                break;
            }
        }
        ESP_LOGI(TAG, "Recording completed, bytes read: %zu", bytes_read);

        // 播放录音内容
        ESP_LOGI(TAG, "Playing recorded audio once...");
        
        // 使用 play_audio_buffer 播放录音，支持自适应格式
        esp_err_t write_ret = play_audio_buffer(
            data,
            bytes_read,
            fs.sample_rate,
            (fs.channel == 1) ? AUDIO_CHANNELS_MONO : AUDIO_CHANNELS_STEREO,
            (i2s_data_bit_width_t)fs.bits_per_sample,
            AUDIO_PLAYBACK_BLOCKING
        );
        
        if (write_ret != ESP_OK) {
            ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(write_ret));
        } else {
            ESP_LOGI(TAG, "Playback completed successfully -> bytes played: %zu", bytes_read);
        }
        
        esp_err_t clear_ret = clear_audio_pipeline(80);
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear audio pipeline in record_and_playback_test");
        }
    }
        
    // 释放内存
    free(data);
    
    ESP_LOGI(TAG, "=== Record and playback test completed ===");
    return ESP_OK;
}

esp_codec_dev_handle_t audio_es_tools::get_play_device_handle() const
{
    return play_dev;
}

esp_codec_dev_handle_t audio_es_tools::get_record_device_handle() const
{
    return record_dev;
}

bool audio_es_tools::is_es8311_initialized() const
{
    return es8311_initialized;
}

bool audio_es_tools::is_es7210_initialized() const
{
    return es7210_initialized;
}

bool audio_es_tools::is_system_initialized() const
{
    return system_initialized;
}

bool audio_es_tools::is_es8311_sleeping() const
{
    return es8311_sleeping;
}

bool audio_es_tools::is_es7210_sleeping() const
{
    return es7210_sleeping;
}

void audio_es_tools::set_i2s_pin_config(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                                        gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin)
{
    i2s_bck_pin = bck_pin;
    i2s_mck_pin = mck_pin;
    i2s_data_in_pin = data_in_pin;
    i2s_data_out_pin = data_out_pin;
    i2s_ws_pin = ws_pin;
    this->pa_pin = pa_pin;
    
    ESP_LOGI(TAG, "I2S pin config updated: BCK=%d, MCK=%d, DATA_IN=%d, DATA_OUT=%d, WS=%d, PA=%d", 
             bck_pin, mck_pin, data_in_pin, data_out_pin, ws_pin, pa_pin);
}

bool audio_es_tools::is_pcm_candy_wind_1ch_16k_available() const
{
#ifdef USE_PCM_CANDY_WIND_1CH_16K_16B_9S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_candy_wind_1ch_44k_available() const
{
#ifdef USE_PCM_CANDY_WIND_1CH_44K_16B_45S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_candy_wind_2ch_16k_available() const
{
#ifdef USE_PCM_CANDY_WIND_2CH_16K_16B_9S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_candy_wind_2ch_44k_available() const
{
#ifdef USE_PCM_CANDY_WIND_2CH_44K_16B_45S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_sine_440hz_2ch_16k_16b_10s_available() const
{
#ifdef USE_PCM_SINE_440HZ_2CH_16K_16B_10S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_startup_1ch_16k_available() const
{
#ifdef USE_PCM_STARTUP_1CH_16K_16B_4S
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_startup_2ch_16k_available() const
{
#ifdef USE_PCM_STARTUP_2CH_16K_16B_4S
    return true;
#else
    return false;
#endif
}

int audio_es_tools::get_available_pcm_count() const
{
    int count = 0;
    
#ifdef USE_PCM_CANDY_WIND_1CH_16K_16B_9S
    count++;
#endif

#ifdef USE_PCM_CANDY_WIND_1CH_44K_16B_45S
    count++;
#endif

#ifdef USE_PCM_CANDY_WIND_2CH_16K_16B_9S
    count++;
#endif

#ifdef USE_PCM_CANDY_WIND_2CH_44K_16B_45S
    count++;
#endif

#ifdef USE_PCM_STARTUP_1CH_16K_16B_4S
    count++;
#endif

#ifdef USE_PCM_STARTUP_2CH_16K_16B_4S
    count++;
#endif

#ifdef USE_PCM_SINE_440HZ_2CH_16K_16B_10S
    count++;
#endif

    return count;
}

const char* audio_es_tools::get_audio_file_name(audio_file_type_t audio_type) const
{
    switch (audio_type) {
        case AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S:
            return "candy_wind_pcm_1ch_16k_16bit_9s.pcm";
        case AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S:
            return "candy_wind_pcm_1ch_44.1k_16bit_45.5s.pcm";
        case AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S:
            return "candy_wind_pcm_2ch_16k_16bit_9s.pcm";
        case AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S:
            return "candy_wind_pcm_2ch_44.1k_16bit_45.5s.pcm";
        case AUDIO_FILE_STARTUP_1CH_16K_16B_4S:
            return "startup_pcm_1ch_16k_16bit_4s.pcm";
        case AUDIO_FILE_STARTUP_2CH_16K_16B_4S:
            return "startup_pcm_2ch_16k_16bit_4s.pcm";
        case AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S:
            return "sine_440Hz_pcm_2ch_16k_16bit_10s.pcm";
        default:
            return "unknown";
    }
}

bool audio_es_tools::is_audio_file_available(audio_file_type_t audio_type) const
{
    switch (audio_type) {
        case AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S:
            return is_pcm_candy_wind_1ch_16k_available();
        case AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S:
            return is_pcm_candy_wind_1ch_44k_available();
        case AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S:
            return is_pcm_candy_wind_2ch_16k_available();
        case AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S:
            return is_pcm_candy_wind_2ch_44k_available();
        case AUDIO_FILE_STARTUP_1CH_16K_16B_4S:
            return is_pcm_startup_1ch_16k_available();
        case AUDIO_FILE_STARTUP_2CH_16K_16B_4S:
            return is_pcm_startup_2ch_16k_available();
        case AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S:
            return is_pcm_sine_440hz_2ch_16k_16b_10s_available();
        default:
            return false;
    }
}

// ============================================================================
// 获取PCM数据和格式参数
// ============================================================================

esp_err_t audio_es_tools::get_pcm_data_and_format(audio_file_type_t audio_type,
                                                   const uint8_t*& pcm_start,
                                                   size_t& pcm_len,
                                                   uint32_t& file_sample_rate_hz,
                                                   audio_channels_t& file_channels,
                                                   i2s_data_bit_width_t& file_bits)
{
    // 初始化输出参数
    pcm_start = nullptr;
    pcm_len = 0;
    
    // 根据文件类型获取PCM数据
    switch (audio_type) {
        case AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S:
#ifdef USE_PCM_CANDY_WIND_1CH_16K_16B_9S
            pcm_start = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
            pcm_len = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
#else
            ESP_LOGE(TAG, "candy_wind_pcm_1ch_16k_16bit_9s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S:
#ifdef USE_PCM_CANDY_WIND_1CH_44K_16B_45S
            pcm_start = _binary_candy_wind_pcm_1ch_44_1k_16bit_45_5s_pcm_start;
            pcm_len = _binary_candy_wind_pcm_1ch_44_1k_16bit_45_5s_pcm_end - _binary_candy_wind_pcm_1ch_44_1k_16bit_45_5s_pcm_start;
#else
            ESP_LOGE(TAG, "candy_wind_pcm_1ch_44.1k_16bit_45.5s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S:
#ifdef USE_PCM_CANDY_WIND_2CH_16K_16B_9S
            pcm_start = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
            pcm_len = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
#else
            ESP_LOGE(TAG, "candy_wind_pcm_2ch_16k_16bit_9s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S:
#ifdef USE_PCM_CANDY_WIND_2CH_44K_16B_45S
            pcm_start = _binary_candy_wind_pcm_2ch_44_1k_16bit_45_5s_pcm_start;
            pcm_len = _binary_candy_wind_pcm_2ch_44_1k_16bit_45_5s_pcm_end - _binary_candy_wind_pcm_2ch_44_1k_16bit_45_5s_pcm_start;
#else
            ESP_LOGE(TAG, "candy_wind_pcm_2ch_44.1k_16bit_45.5s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_STARTUP_1CH_16K_16B_4S:
#ifdef USE_PCM_STARTUP_1CH_16K_16B_4S
            pcm_start = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
            pcm_len = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end - _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
#else
            ESP_LOGE(TAG, "startup_pcm_1ch_16k_16bit_4s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_STARTUP_2CH_16K_16B_4S:
#ifdef USE_PCM_STARTUP_2CH_16K_16B_4S
            pcm_start = _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start;
            pcm_len = _binary_startup_pcm_2ch_16k_16bit_4s_pcm_end - _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start;
#else
            ESP_LOGE(TAG, "startup_pcm_2ch_16k_16bit_4s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S:
#ifdef USE_PCM_SINE_440HZ_2CH_16K_16B_10S
            pcm_start = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
            pcm_len = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end - _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
#else
            ESP_LOGE(TAG, "sine_440Hz_pcm_2ch_16k_16bit_10s.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        default:
            ESP_LOGE(TAG, "Invalid audio file type: %d", audio_type);
            return ESP_ERR_INVALID_ARG;
    }

    // 验证PCM数据有效性
    if (!pcm_start || pcm_len == 0) {
        ESP_LOGE(TAG, "Invalid PCM data for %s", get_audio_file_name(audio_type));
        return ESP_ERR_INVALID_SIZE;
    }

    // 设置文件格式参数（内置PCM文件的编解码格式是固定的）
    switch (audio_type) {
        case AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S:
            file_sample_rate_hz = 16000;
            file_channels = AUDIO_CHANNELS_STEREO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S:
            file_sample_rate_hz = 16000;
            file_channels = AUDIO_CHANNELS_MONO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S:
            file_sample_rate_hz = 44100;
            file_channels = AUDIO_CHANNELS_MONO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S:
            file_sample_rate_hz = 16000;
            file_channels = AUDIO_CHANNELS_STEREO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S:
            file_sample_rate_hz = 44100;
            file_channels = AUDIO_CHANNELS_STEREO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_STARTUP_1CH_16K_16B_4S:
            file_sample_rate_hz = 16000;
            file_channels = AUDIO_CHANNELS_MONO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_FILE_STARTUP_2CH_16K_16B_4S:
            file_sample_rate_hz = 16000;
            file_channels = AUDIO_CHANNELS_STEREO;
            file_bits = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        default:
            // 对于未明确指定的文件类型，使用默认值（已在调用前初始化）
            break;
    }

    return ESP_OK;
}

// ============================================================================
// 播放音频文件（内部实现）
// ============================================================================

esp_err_t audio_es_tools::play_audio_file_impl(audio_file_type_t audio_type, bool check_stop_signal, float duration_limit_seconds)
{
    // 边界检查
    if (audio_type < 0 || audio_type >= AUDIO_FILE_MAX) {
        ESP_LOGE(TAG, "Invalid audio file type: %d (valid range: 0-%d)", 
                 audio_type, AUDIO_FILE_MAX - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 仅要求播放设备已准备
    if (!play_dev || !es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (!play_dev) {
        ESP_LOGE(TAG, "Play device not available");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查音频文件是否可用
    if (!is_audio_file_available(audio_type)) {
        ESP_LOGE(TAG, "Audio file %s is not available", get_audio_file_name(audio_type));
        return ESP_ERR_NOT_FOUND;
    }

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Playing audio file: %s (limited to %.1f seconds)", get_audio_file_name(audio_type), duration_limit_seconds);
    } else {
        ESP_LOGI(TAG, "Playing audio file: %s (full file)", get_audio_file_name(audio_type));
    }

    // 获取PCM数据和格式参数
    const uint8_t *pcm_start = nullptr;
    size_t pcm_len = 0;
    uint32_t file_sample_rate_hz = static_cast<uint32_t>(sample_rate);
    audio_channels_t file_channels = tx_channels;
    i2s_data_bit_width_t file_bits = bits_per_sample;
    
    esp_err_t ret = get_pcm_data_and_format(audio_type, pcm_start, pcm_len, 
                                             file_sample_rate_hz, file_channels, file_bits);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "PCM data size: %zu bytes", pcm_len);

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(tx_channels);

    uint8_t* converted_buffer = nullptr;
    size_t converted_size = 0;

    ret = remix_convert_pcm_to_format(pcm_start,
                                      pcm_len,
                                      file_sample_rate_hz,
                                      static_cast<uint32_t>(file_channels),
                                      static_cast<uint32_t>(file_bits),
                                      system_sample_rate_hz,
                                      system_channels,
                                      system_bits,
                                      &converted_buffer,
                                      &converted_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert audio format: %s", esp_err_to_name(ret));
        return ret;
    }

    const bool using_converted = converted_buffer != nullptr;
    uint8_t* playback_buffer = using_converted ? converted_buffer : const_cast<uint8_t*>(pcm_start);
    const size_t playback_size = using_converted ? converted_size : pcm_len;

    if (using_converted) {
        ESP_LOGI(TAG, "Audio format converted: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch (%zu -> %zu bytes)",
                 file_sample_rate_hz,
                 static_cast<uint32_t>(file_bits),
                 static_cast<uint32_t>(file_channels),
                 system_sample_rate_hz,
                 system_bits,
                 system_channels,
                 pcm_len,
                 playback_size);
    } else {
        ESP_LOGI(TAG, "Audio format matches system configuration");
    }

    size_t bytes_to_play = playback_size;
    if (duration_limit_seconds > 0) {
        const uint32_t bytes_per_frame = (system_bits * system_channels) / 8;

        if (bytes_per_frame == 0 || system_sample_rate_hz == 0) {
            ESP_LOGW(TAG, "Duration limit skipped due to invalid system format (bytes_per_frame=%u, sample_rate=%u)",
                     bytes_per_frame, system_sample_rate_hz);
        } else {
            const size_t bytes_per_second = static_cast<size_t>(system_sample_rate_hz) * bytes_per_frame;
            const size_t limited_bytes = static_cast<size_t>(bytes_per_second * duration_limit_seconds);

            if (limited_bytes < bytes_to_play) {
                bytes_to_play = limited_bytes;
                ESP_LOGI(TAG, "Duration limit: will play %zu bytes out of %zu bytes", bytes_to_play, playback_size);
            } else {
                ESP_LOGI(TAG, "Duration limit (%.1f s) exceeds file length, playing full file", duration_limit_seconds);
            }
        }
    }

    // 分块播放PCM数据,支持中途停止
    const size_t CHUNK_SIZE = 4096;  // 每次写4KB数据
    size_t bytes_written = 0;
    ret = ESP_OK;
    
    while (bytes_written < bytes_to_play) {
        // 检查停止信号（仅在异步模式下）
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Received stop signal during playback, stopping gracefully...");
                ret = ESP_ERR_INVALID_STATE;  // 使用特殊错误码表示被中断
                break;
            }
        }
        
        size_t remaining = bytes_to_play - bytes_written;
        size_t to_write = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
        
    ret = esp_codec_dev_write(play_dev, playback_buffer + bytes_written, to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write audio chunk at offset %zu: %s", bytes_written, esp_err_to_name(ret));
            if (using_converted) {
                heap_caps_free(converted_buffer);
            }
            return ret;
        }
        
        bytes_written += to_write;
    }
    
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Playback interrupted by stop signal at %zu/%zu bytes", bytes_written, bytes_to_play);
        // 即使中断也要清空管道,避免残留数据
        // 先临时降低音量，减少清理时的杂音
        float saved_volume = volume;
        set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(20));  // 等待音量过渡
        
        esp_err_t clear_ret = clear_audio_pipeline(150);
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear audio pipeline after interruption: %s", esp_err_to_name(clear_ret));
        }
        
        // 恢复音量
        set_volume(saved_volume);
        if (using_converted) {
            heap_caps_free(converted_buffer);
        }
        return ret;
    }

    ESP_LOGI(TAG, "Audio playback completed successfully (%zu bytes)", bytes_written);
    
    // 播放完成时清理管道,清空残留数据
    // 先临时降低音量到10%，减少管道清理时的杂音
    float saved_volume = volume;
    set_volume(10.0);
    vTaskDelay(pdMS_TO_TICKS(20));  // 等待音量平滑过渡
    
    // 使用更长的静音时间彻底清除硬件缓冲区
    esp_err_t clear_ret = clear_audio_pipeline(200);
    if (clear_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear audio pipeline: %s", esp_err_to_name(clear_ret));
    }
    
    // 恢复原始音量
    set_volume(saved_volume);

    if (using_converted) {
        heap_caps_free(converted_buffer);
    }

    return ESP_OK;
}

esp_err_t audio_es_tools::play_audio_buffer_impl(const uint8_t* buffer, size_t buffer_size, 
                                                   uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels, 
                                                   i2s_data_bit_width_t buffer_bits,
                                                   bool check_stop_signal, float duration_limit_seconds)
{
    // 参数验证
    if (!buffer || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer_sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", buffer_sample_rate_hz);
        return ESP_ERR_INVALID_ARG;
    }

    // 仅要求播放设备已准备
    if (!play_dev || !es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - limited to %.1f seconds", 
                 buffer_size, buffer_sample_rate_hz, 
                 static_cast<uint32_t>(buffer_channels), 
                 static_cast<uint32_t>(buffer_bits),
                 duration_limit_seconds);
    } else {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - full buffer", 
                 buffer_size, buffer_sample_rate_hz, 
                 static_cast<uint32_t>(buffer_channels), 
                 static_cast<uint32_t>(buffer_bits));
    }

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(tx_channels);

    uint8_t* converted_buffer = nullptr;
    size_t converted_size = 0;
    esp_err_t ret = ESP_OK;

    // 检查是否需要格式转换
    const bool need_conversion = (buffer_sample_rate_hz != system_sample_rate_hz) ||
                                  (static_cast<uint32_t>(buffer_channels) != system_channels) ||
                                  (static_cast<uint32_t>(buffer_bits) != system_bits);

    if (need_conversion) {
        ret = remix_convert_pcm_to_format(buffer,
                                          buffer_size,
                                          buffer_sample_rate_hz,
                                          static_cast<uint32_t>(buffer_channels),
                                          static_cast<uint32_t>(buffer_bits),
                                          system_sample_rate_hz,
                                          system_channels,
                                          system_bits,
                                          &converted_buffer,
                                          &converted_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert audio format: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGI(TAG, "Audio format converted: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch (%zu -> %zu bytes)",
                 buffer_sample_rate_hz,
                 static_cast<uint32_t>(buffer_bits),
                 static_cast<uint32_t>(buffer_channels),
                 system_sample_rate_hz,
                 system_bits,
                 system_channels,
                 buffer_size,
                 converted_size);
    } else {
        ESP_LOGI(TAG, "Audio format matches system configuration, no conversion needed");
    }

    const bool using_converted = converted_buffer != nullptr;
    const uint8_t* playback_buffer = using_converted ? converted_buffer : buffer;
    size_t playback_size = using_converted ? converted_size : buffer_size;

    // 计算播放时长限制
    size_t bytes_to_play = playback_size;
    if (duration_limit_seconds > 0) {
        const uint32_t bytes_per_frame = (system_bits * system_channels) / 8;

        if (bytes_per_frame == 0 || system_sample_rate_hz == 0) {
            ESP_LOGW(TAG, "Duration limit skipped due to invalid system format (bytes_per_frame=%u, sample_rate=%u)",
                     bytes_per_frame, system_sample_rate_hz);
        } else {
            const size_t bytes_per_second = static_cast<size_t>(system_sample_rate_hz) * bytes_per_frame;
            const size_t limited_bytes = static_cast<size_t>(bytes_per_second * duration_limit_seconds);

            if (limited_bytes < bytes_to_play) {
                bytes_to_play = limited_bytes;
                ESP_LOGI(TAG, "Duration limit: will play %zu bytes out of %zu bytes", bytes_to_play, playback_size);
            } else {
                ESP_LOGI(TAG, "Duration limit (%.1f s) exceeds buffer length, playing full buffer", duration_limit_seconds);
            }
        }
    }

    // 分块播放PCM数据，支持中途停止
    const size_t CHUNK_SIZE = 4096;  // 每次写4KB数据
    size_t bytes_written = 0;
    ret = ESP_OK;
    
    while (bytes_written < bytes_to_play) {
        // 检查停止信号（仅在异步模式下）
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Received stop signal during playback, stopping gracefully...");
                ret = ESP_ERR_INVALID_STATE;  // 使用特殊错误码表示被中断
                break;
            }
        }
        
        size_t remaining = bytes_to_play - bytes_written;
        size_t to_write = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
        
        ret = esp_codec_dev_write(play_dev, const_cast<uint8_t*>(playback_buffer + bytes_written), to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write audio chunk at offset %zu: %s", bytes_written, esp_err_to_name(ret));
            if (using_converted) {
                heap_caps_free(converted_buffer);
            }
            return ret;
        }
        
        bytes_written += to_write;
    }
    
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Playback interrupted by stop signal at %zu/%zu bytes", bytes_written, bytes_to_play);
        // 即使中断也要清空管道，避免残留数据
        // 先临时降低音量，减少清理时的杂音
        float saved_volume = volume;
        set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(20));  // 等待音量过渡
        
        esp_err_t clear_ret = clear_audio_pipeline(150);
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear audio pipeline after interruption: %s", esp_err_to_name(clear_ret));
        }
        
        // 恢复音量
        set_volume(saved_volume);
        if (using_converted) {
            heap_caps_free(converted_buffer);
        }
        return ret;
    }

    ESP_LOGI(TAG, "Audio buffer playback completed successfully (%zu bytes)", bytes_written);
    
    // 播放完成时清理管道，清空残留数据
    // 先临时降低音量到10%，减少管道清理时的杂音
    float saved_volume = volume;
    set_volume(10.0);
    vTaskDelay(pdMS_TO_TICKS(20));  // 等待音量平滑过渡
    
    // 使用更长的静音时间彻底清除硬件缓冲区
    esp_err_t clear_ret = clear_audio_pipeline(200);
    if (clear_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear audio pipeline: %s", esp_err_to_name(clear_ret));
    }
    
    // 恢复原始音量
    set_volume(saved_volume);

    if (using_converted) {
        heap_caps_free(converted_buffer);
    }

    return ESP_OK;
}

esp_err_t audio_es_tools::play_audio_file(audio_file_type_t audio_type, audio_playback_mode_t mode, float duration_limit_seconds)
{
    if (mode == AUDIO_PLAYBACK_BLOCKING) {
        return play_audio_file_impl(audio_type, false, duration_limit_seconds);  // 阻塞模式不检查停止信号
    }

    if (playback_task_handle) {
        ESP_LOGW(TAG, "Playback task already running");
        return ESP_ERR_INVALID_STATE;
    }

    playback_task_args* args = static_cast<playback_task_args*>(calloc(1, sizeof(playback_task_args)));
    if (!args) {
        ESP_LOGE(TAG, "Failed to allocate playback task args");
        return ESP_ERR_NO_MEM;
    }

    args->instance = this;
    args->audio_type = audio_type;
    args->duration_limit_seconds = duration_limit_seconds;

    BaseType_t task_ret = xTaskCreate(playback_task_entry, "audio_play_task", 4096, args, 5, &playback_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create playback task");
        free(args);
        playback_task_handle = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Async playback task started for %s", get_audio_file_name(audio_type));
    return ESP_OK;
}

esp_err_t audio_es_tools::play_audio_buffer(const uint8_t* buffer, size_t buffer_size, 
                                             uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels, 
                                             i2s_data_bit_width_t buffer_bits,
                                             audio_playback_mode_t mode, 
                                             float duration_limit_seconds)
{
    // 参数验证
    if (!buffer || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer_sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", buffer_sample_rate_hz);
        return ESP_ERR_INVALID_ARG;
    }

    // 阻塞模式：直接播放
    if (mode == AUDIO_PLAYBACK_BLOCKING) {
        return play_audio_buffer_impl(buffer, buffer_size, 
                                       buffer_sample_rate_hz, buffer_channels, buffer_bits,
                                       false, duration_limit_seconds);
    }

    // 异步模式：创建播放任务
    if (playback_task_handle) {
        ESP_LOGW(TAG, "Playback task already running");
        return ESP_ERR_INVALID_STATE;
    }

    // 为异步播放分配并复制缓冲区（因为调用者可能在函数返回后释放原始缓冲区）
    uint8_t* buffer_copy = static_cast<uint8_t*>(heap_caps_malloc(buffer_size, MALLOC_CAP_8BIT));
    if (!buffer_copy) {
        ESP_LOGE(TAG, "Failed to allocate buffer copy for async playback (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }
    memcpy(buffer_copy, buffer, buffer_size);

    buffer_playback_task_args* args = static_cast<buffer_playback_task_args*>(calloc(1, sizeof(buffer_playback_task_args)));
    if (!args) {
        ESP_LOGE(TAG, "Failed to allocate buffer playback task args");
        heap_caps_free(buffer_copy);
        return ESP_ERR_NO_MEM;
    }

    args->instance = this;
    args->buffer = buffer_copy;
    args->buffer_size = buffer_size;
    args->buffer_sample_rate_hz = buffer_sample_rate_hz;
    args->buffer_channels = buffer_channels;
    args->buffer_bits = buffer_bits;
    args->duration_limit_seconds = duration_limit_seconds;
    args->own_buffer = true;  // 任务需要释放buffer_copy

    BaseType_t task_ret = xTaskCreate(buffer_playback_task_entry, "audio_buf_play", 4096, args, 5, &playback_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create buffer playback task");
        heap_caps_free(buffer_copy);
        free(args);
        playback_task_handle = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Async buffer playback task started (%zu bytes, %u Hz, %u ch, %u bit)", 
             buffer_size, buffer_sample_rate_hz, 
             static_cast<uint32_t>(buffer_channels), 
             static_cast<uint32_t>(buffer_bits));
    return ESP_OK;
}

void audio_es_tools::playback_task_entry(void* param)
{
    auto* args = static_cast<playback_task_args*>(param);
    audio_es_tools* instance = args->instance;
    audio_file_type_t audio_type = args->audio_type;
    uint32_t duration_limit_seconds = args->duration_limit_seconds;
    free(args);

    ESP_LOGI(TAG, "Playback task started, ready to receive stop signals");
    
    // 异步模式：启用停止信号检查
    esp_err_t result = instance->play_audio_file_impl(audio_type, true, duration_limit_seconds);
    
    if (result == ESP_ERR_INVALID_STATE) {
        // 被停止信号中断
        ESP_LOGI(TAG, "Playback interrupted by stop request, performing cleanup...");
        
        // 快速静音
        float original_volume = instance->volume;
        instance->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(30));
        
        // 清理音频管道
        esp_err_t clear_ret = instance->clear_audio_pipeline(200);
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear pipeline after stop: %s", esp_err_to_name(clear_ret));
        } else {
            // 二次清理确保彻底
            vTaskDelay(pdMS_TO_TICKS(30));
            instance->clear_audio_pipeline(100);
        }
        
        // 恢复音量
        instance->set_volume(original_volume);
        ESP_LOGI(TAG, "Playback stopped and cleanup completed");
    } else if (result != ESP_OK) {
        ESP_LOGE(TAG, "Async playback failed for %s: %s", instance->get_audio_file_name(audio_type), esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Playback task completed normally");
    }

    instance->playback_task_handle = nullptr;
    vTaskDelete(nullptr);
}

void audio_es_tools::buffer_playback_task_entry(void* param)
{
    auto* args = static_cast<buffer_playback_task_args*>(param);
    audio_es_tools* instance = args->instance;
    const uint8_t* buffer = args->buffer;
    size_t buffer_size = args->buffer_size;
    uint32_t buffer_sample_rate_hz = args->buffer_sample_rate_hz;
    audio_channels_t buffer_channels = args->buffer_channels;
    i2s_data_bit_width_t buffer_bits = args->buffer_bits;
    float duration_limit_seconds = args->duration_limit_seconds;
    bool own_buffer = args->own_buffer;
    free(args);

    ESP_LOGI(TAG, "Buffer playback task started, ready to receive stop signals");
    
    // 异步模式：启用停止信号检查
    esp_err_t result = instance->play_audio_buffer_impl(buffer, buffer_size,
                                                         buffer_sample_rate_hz, buffer_channels, buffer_bits,
                                                         true, duration_limit_seconds);
    
    if (result == ESP_ERR_INVALID_STATE) {
        // 被停止信号中断
        ESP_LOGI(TAG, "Buffer playback interrupted by stop request, performing cleanup...");
        
        // 快速静音
        float original_volume = instance->volume;
        instance->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(30));
        
        // 清理音频管道
        esp_err_t clear_ret = instance->clear_audio_pipeline(200);
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear pipeline after stop: %s", esp_err_to_name(clear_ret));
        } else {
            // 二次清理确保彻底
            vTaskDelay(pdMS_TO_TICKS(30));
            instance->clear_audio_pipeline(100);
        }
        
        // 恢复音量
        instance->set_volume(original_volume);
        ESP_LOGI(TAG, "Buffer playback stopped and cleanup completed");
    } else if (result != ESP_OK) {
        ESP_LOGE(TAG, "Async buffer playback failed: %s", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Buffer playback task completed normally");
    }

    // 释放缓冲区（如果需要）
    if (own_buffer && buffer) {
        heap_caps_free(const_cast<uint8_t*>(buffer));
    }

    instance->playback_task_handle = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t audio_es_tools::stop_async_playback()
{
    TaskHandle_t task_to_stop = nullptr;

    // 获取互斥锁保护
    if (audio_mutex && xSemaphoreTake(audio_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire audio mutex for stop");
        return ESP_ERR_TIMEOUT;
    }

    if (!playback_task_handle) {
        ESP_LOGI(TAG, "No async playback task running");
        if (audio_mutex) xSemaphoreGive(audio_mutex);
        return ESP_OK;
    }

    task_to_stop = playback_task_handle;
    ESP_LOGI(TAG, "Sending stop signal to playback task...");

    BaseType_t notify_result = xTaskNotify(task_to_stop, 1, eSetValueWithOverwrite);
    if (notify_result != pdPASS) {
        ESP_LOGW(TAG, "Failed to send stop signal, notify result=%ld", (long)notify_result);
        if (audio_mutex) xSemaphoreGive(audio_mutex);
        return ESP_FAIL;
    }

    if (audio_mutex) xSemaphoreGive(audio_mutex);

    ESP_LOGI(TAG, "Stop signal sent, waiting for task to finish...");
    
    // 等待任务自然结束（任务会清空 playback_task_handle）
    const TickType_t max_wait = pdMS_TO_TICKS(3000);  // 最多等待3秒
    TickType_t start_tick = xTaskGetTickCount();
    
    while (playback_task_handle != nullptr) {
        if ((xTaskGetTickCount() - start_tick) > max_wait) {
            ESP_LOGW(TAG, "Playback task did not exit gracefully within timeout");
            // 超时后仍未退出，强制标记为nullptr（任务可能已卡死）
            playback_task_handle = nullptr;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "Playback task exited gracefully");
    return ESP_OK;
}

esp_err_t audio_es_tools::clear_audio_pipeline(uint32_t silence_duration_ms)
{
    // 获取互斥锁保护
    if (audio_mutex && xSemaphoreTake(audio_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire audio mutex for pipeline clear");
        return ESP_ERR_TIMEOUT;
    }

    if (!play_dev || !es8311_initialized) {
        ESP_LOGW(TAG, "Playback device not ready, skipping pipeline clear");
        if (audio_mutex) xSemaphoreGive(audio_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Clearing audio pipeline with %lu ms silence...", silence_duration_ms);

    // 计算静音数据大小（使用播放设备的声道配置）
    uint32_t sample_rate_hz = (uint32_t)sample_rate;
    uint32_t channels = (uint32_t)tx_channels;
    uint32_t bits_per_sample_val = (uint32_t)bits_per_sample;
    uint32_t bytes_per_sample = (bits_per_sample_val * channels) / 8;
    size_t silence_size = (sample_rate_hz * bytes_per_sample * silence_duration_ms) / 1000;

    if (silence_size == 0) {
        if (audio_mutex) xSemaphoreGive(audio_mutex);
        ESP_LOGW(TAG, "Silence size calculated as 0, skipping pipeline clear");
        return ESP_OK;
    }

    size_t chunk_capacity = SILENCE_CHUNK_CAPACITY - (SILENCE_CHUNK_CAPACITY % bytes_per_sample);
    if (chunk_capacity == 0) {
        chunk_capacity = bytes_per_sample;
    }

    esp_codec_dev_handle_t local_play_dev = play_dev;
    size_t remaining = silence_size;
    esp_err_t ret = ESP_OK;

    while (remaining > 0) {
        size_t send_size = remaining < chunk_capacity ? remaining : chunk_capacity;
        ret = esp_codec_dev_write(local_play_dev, g_silence_chunk, send_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write silence chunk (%zu bytes): %s", send_size, esp_err_to_name(ret));
            break;
        }
        remaining -= send_size;
    }

    if (audio_mutex) xSemaphoreGive(audio_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    // 等待静音播放完成
    vTaskDelay(pdMS_TO_TICKS(silence_duration_ms + 50));
    ESP_LOGI(TAG, "Audio pipeline cleared successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::set_volume(float volume_value)
{
    // 检查音量范围
    if (volume_value < 0.0 || volume_value > 100.0) {
        ESP_LOGE(TAG, "Invalid volume %.1f, must be between 0.0 and 100.0", volume_value);
        return ESP_ERR_INVALID_ARG;
    }
    
    volume = volume_value;
    ESP_LOGI(TAG, "Volume set to %.1f", volume_value);
    
    // 如果播放设备已初始化，立即应用新的音量设置
    if (play_dev && es8311_initialized) {
        esp_err_t ret = esp_codec_dev_set_out_vol(play_dev, volume_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to apply volume %.1f: %s", volume_value, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Applied volume %.1f to ES8311", volume_value);
    }
    
    return ESP_OK;
}

esp_err_t audio_es_tools::set_record_gain(float gain)
{
    // 检查增益范围（ES7210支持0-66dB）
    if (gain < 0.0 || gain > 66.0) {
        ESP_LOGE(TAG, "Invalid record gain %.1f dB, must be between 0.0 and 66.0", gain);
        return ESP_ERR_INVALID_ARG;
    }
    
    record_gain = gain;
    ESP_LOGI(TAG, "Record gain set to %.1f dB", gain);
    
    // 如果录音设备已初始化，立即应用新的增益设置
    if (record_dev && es7210_initialized) {
        esp_err_t ret = esp_codec_dev_set_in_gain(record_dev, gain);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to apply record gain %.1f dB: %s", gain, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Applied record gain %.1f dB to ES7210", gain);
    }
    
    return ESP_OK;
}

esp_err_t audio_es_tools::set_audio_levels(float volume, float gain)
{
    esp_err_t ret_vol = set_volume(volume);
    esp_err_t ret_gain = set_record_gain(gain);
    
    // 如果任一设置失败，返回失败
    if (ret_vol != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set volume in audio levels configuration");
        return ret_vol;
    }
    
    if (ret_gain != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set record gain in audio levels configuration");
        return ret_gain;
    }
    
    ESP_LOGI(TAG, "Audio levels configured successfully - Volume: %.1f, Gain: %.1f dB", volume, gain);
    return ESP_OK;
}

esp_err_t audio_es_tools::es7210_set_mic_channel_gain(audio_mic_channel_t mic_channels_to_set, es7210_mic_gain_t gain)
{
    // 检查 ES7210 是否已初始化
    if (!es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized, cannot set channel gain");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查录音设备句柄是否可用
    if (!record_dev) {
        ESP_LOGE(TAG, "ES7210 record device not available");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查要设置的通道是否是已初始化通道的子集
    if ((mic_channels_to_set & ~mic_channels) != 0) {
        ESP_LOGE(TAG, "Invalid channel mask 0x%02X, not a subset of initialized channels 0x%02X",
                 mic_channels_to_set, mic_channels);
        return ESP_ERR_INVALID_ARG;
    }

    // 增益值范围检查：0-14对应0dB-37.5dB
    uint8_t gain_reg_value = static_cast<uint8_t>(gain);
    if (gain_reg_value > 14) {
        ESP_LOGE(TAG, "Invalid gain register value %d, must be 0-14", gain_reg_value);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算实际dB值用于日志显示
    static const float gain_db_table[15] = {
        0.0f, 3.0f, 6.0f, 9.0f, 12.0f, 15.0f, 18.0f, 21.0f, 
        24.0f, 27.0f, 30.0f, 33.0f, 34.5f, 36.0f, 37.5f
    };
    float actual_gain_db = gain_db_table[gain_reg_value];
    
    // ES7210 麦克风增益寄存器地址:
    // 0x43 = MIC1_GAIN, 0x44 = MIC2_GAIN, 0x45 = MIC3_GAIN, 0x46 = MIC4_GAIN
    // MIC1GAIN_SETTING 占 bit[3:0]，其他位需保留
    const uint8_t gain_regs[4] = {0x43, 0x44, 0x45, 0x46};
    
    // 遍历所有通道，先读后写对应的增益寄存器
    for (int ch = 0; ch < 4; ch++) {
        if (mic_channels_to_set & (1 << ch)) {
            // 1. 先读取当前寄存器值
            int current_val = 0;
            int ret = esp_codec_dev_read_reg(record_dev, gain_regs[ch], &current_val);
            
            if (ret != ESP_CODEC_DEV_OK) {
                ESP_LOGE(TAG, "Failed to read CH%d gain register (reg=0x%02X), ret=%d", 
                         ch + 1, gain_regs[ch], ret);
                return ESP_FAIL;
            }
            
            // 2. 保留高4位（bit[7:4]），只修改低4位（bit[3:0]）
            int new_val = (current_val & 0xF0) | (gain_reg_value & 0x0F);
            
            // 3. 写回修改后的值
            ret = esp_codec_dev_write_reg(record_dev, gain_regs[ch], new_val);
            
            if (ret != ESP_CODEC_DEV_OK) {
                ESP_LOGE(TAG, "Failed to write gain to CH%d (reg=0x%02X), ret=%d", 
                         ch + 1, gain_regs[ch], ret);
                return ESP_FAIL;
            }

            // 确保对应声道未被数字静音，允许即时生效
            const uint8_t mute_reg = (ch < 2) ? 0x14u : 0x15u;
            const uint8_t mute_bit = (ch % 2 == 0) ? 0x01u : 0x02u;
            int mute_val = 0;
            int mute_ret = esp_codec_dev_read_reg(record_dev, mute_reg, &mute_val);
            if (mute_ret == ESP_CODEC_DEV_OK) {
                int cleared_val = mute_val & ~mute_bit;
                if (cleared_val != mute_val) {
                    mute_ret = esp_codec_dev_write_reg(record_dev, mute_reg, cleared_val);
                    if (mute_ret != ESP_CODEC_DEV_OK) {
                        ESP_LOGW(TAG, "Failed to clear mute bit for CH%d (reg=0x%02X, ret=%d)",
                                 ch + 1, mute_reg, mute_ret);
                    } else {
                        ESP_LOGD(TAG, "Cleared mute bit for CH%d (reg 0x%02X: 0x%02X -> 0x%02X)",
                                 ch + 1, mute_reg, mute_val, cleared_val);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Failed to read mute reg 0x%02X for CH%d (ret=%d)",
                         mute_reg, ch + 1, mute_ret);
            }
            
            ESP_LOGD(TAG, "CH%d gain set to %.1fdB (reg=0x%02X, old=0x%02X, new=0x%02X, gain_bits=%d)", 
                     ch + 1, actual_gain_db, gain_regs[ch], current_val, new_val, gain_reg_value);
        }
    }

    ESP_LOGI(TAG, "Set gain %.1fdB for channels 0x%02X (%s) successfully",
             actual_gain_db, mic_channels_to_set, get_mic_channels_description(mic_channels_to_set));
    return ESP_OK;
}

// set_mic_channels函数已删除，请直接在es7210_init中传入麦克风通道参数

const char* audio_es_tools::get_mic_channels_description(audio_mic_channel_t channels) const
{
    switch (channels) {
        case AUDIO_MIC_NONE:
            return "None";
        case AUDIO_MIC_CHANNEL_1:
            return "MIC1";
        case AUDIO_MIC_CHANNEL_2:
            return "MIC2";
        case AUDIO_MIC_CHANNEL_3:
            return "MIC3";
        case AUDIO_MIC_CHANNEL_4:
            return "MIC4";
        case AUDIO_MIC_CHANNEL_12:
            return "MIC1+MIC2";
        case AUDIO_MIC_CHANNEL_13:
            return "MIC1+MIC3";
        case AUDIO_MIC_CHANNEL_14:
            return "MIC1+MIC4";
        case AUDIO_MIC_CHANNEL_23:
            return "MIC2+MIC3";
        case AUDIO_MIC_CHANNEL_24:
            return "MIC2+MIC4";
        case AUDIO_MIC_CHANNEL_34:
            return "MIC3+MIC4";
        case AUDIO_MIC_CHANNEL_123:
            return "MIC1+MIC2+MIC3";
        case AUDIO_MIC_CHANNEL_124:
            return "MIC1+MIC2+MIC4";
        case AUDIO_MIC_CHANNEL_134:
            return "MIC1+MIC3+MIC4";
        case AUDIO_MIC_CHANNEL_234:
            return "MIC2+MIC3+MIC4";
        case AUDIO_MIC_CHANNEL_ALL:
            return "MIC1+MIC2+MIC3+MIC4";
        default:
            return "Custom";
    }
}

bool audio_es_tools::is_mic_channels_valid(audio_mic_channel_t channels) const
{
    // 检查通道值是否在有效范围内（0x00-0x0F）
    if (channels > AUDIO_MIC_CHANNEL_ALL) {
        return false;
    }
    
    // AUDIO_MIC_NONE (0x00) 是有效的，表示不使用任何麦克风
    return true;
}

int audio_es_tools::count_selected_mics() const
{
    uint8_t v = static_cast<uint8_t>(mic_channels);
    int cnt = 0;
    while (v) {
        cnt += (v & 0x1);
        v >>= 1;
    }
    return cnt;
}

esp_err_t audio_es_tools::record_to_file(const char* filepath, uint32_t record_duration_seconds, size_t chunk_size)
{
    if (!system_initialized || !es7210_initialized || !record_dev) {
        ESP_LOGE(TAG, "Audio recording path not ready (system:%d, es7210:%d, dev:%p)",
                 system_initialized, es7210_initialized, record_dev);
        return ESP_ERR_INVALID_STATE;
    }

    if (filepath == nullptr || filepath[0] == '\0' || record_duration_seconds == 0) {
        ESP_LOGE(TAG, "Invalid arguments: filepath=%p, duration=%lu", filepath, record_duration_seconds);
        return ESP_ERR_INVALID_ARG;
    }

    // 使用录音设备的声道配置
    uint32_t channels = (rx_channels == AUDIO_CHANNELS_MONO) ? 1u : 2u;
    uint32_t bits = static_cast<uint32_t>(bits_per_sample);
    uint32_t sample_rate_hz = static_cast<uint32_t>(sample_rate);
    uint32_t bytes_per_sample = (bits * channels) / 8;

    if (bytes_per_sample == 0 || sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid audio format: sr=%u, bits=%u, channels=%u", sample_rate_hz, bits, channels);
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t total_bytes_64 = static_cast<uint64_t>(sample_rate_hz) * record_duration_seconds * bytes_per_sample;
    if (total_bytes_64 == 0) {
        ESP_LOGE(TAG, "Calculated recording size is zero");
        return ESP_ERR_INVALID_STATE;
    }

    if (total_bytes_64 > SIZE_MAX) {
        ESP_LOGE(TAG, "Requested recording exceeds size_t capacity: %llu bytes", (unsigned long long)total_bytes_64);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t total_bytes = static_cast<size_t>(total_bytes_64);
    size_t frame_size = bytes_per_sample;
    if (chunk_size < frame_size) {
        ESP_LOGW(TAG, "chunk_size %zu too small, adjusting to frame size %zu", chunk_size, frame_size);
        chunk_size = frame_size;
    }

    size_t aligned_chunk = (chunk_size / frame_size) * frame_size;
    if (aligned_chunk == 0) {
        aligned_chunk = frame_size;
    }
    if (aligned_chunk != chunk_size) {
        ESP_LOGW(TAG, "Aligning chunk size from %zu to %zu to match frame size", chunk_size, aligned_chunk);
        chunk_size = aligned_chunk;
    }

    esp_err_t dir_ret = ensure_parent_directories(filepath);
    if (dir_ret != ESP_OK) {
        return dir_ret;
    }

    uint8_t* chunk_buffer = static_cast<uint8_t*>(malloc(chunk_size));
    if (!chunk_buffer) {
        ESP_LOGE(TAG, "Failed to allocate chunk buffer: %zu bytes", chunk_size);
        return ESP_ERR_NO_MEM;
    }

    FILE* file = fopen(filepath, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open %s for writing (errno=%d, %s)", filepath, errno, strerror(errno));
        free(chunk_buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Recording %u Hz, %u-bit, %u-ch audio for %lu s -> %zu bytes into %s",
             sample_rate_hz, bits, channels, record_duration_seconds, total_bytes, filepath);

    size_t bytes_written = 0;
    esp_err_t ret = ESP_OK;
    TickType_t start_ticks = xTaskGetTickCount();

    while (bytes_written < total_bytes) {
        size_t remaining = total_bytes - bytes_written;
        size_t to_read = (chunk_size < remaining) ? chunk_size : remaining;
        ret = esp_codec_dev_read(record_dev, chunk_buffer, static_cast<int>(to_read));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu/%zu bytes: %s", bytes_written, total_bytes, esp_err_to_name(ret));
            break;
        }

        size_t written = fwrite(chunk_buffer, 1, to_read, file);
        if (written != to_read) {
            ESP_LOGE(TAG, "File write error at %zu/%zu bytes (written %zu)", bytes_written, total_bytes, written);
            ret = ESP_FAIL;
            break;
        }

        bytes_written += written;
    }

    fflush(file);
    fclose(file);
    free(chunk_buffer);

    TickType_t elapsed_ticks = xTaskGetTickCount() - start_ticks;
    uint32_t elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);
    ESP_LOGI(TAG, "Recording loop elapsed %u ms", elapsed_ms);

    if (ret != ESP_OK) {
        return ret;
    }

    if (bytes_written == 0) {
        ESP_LOGE(TAG, "No audio captured, deleting %s", filepath);
        remove(filepath);
        return ESP_FAIL;
    }

    if (bytes_written < total_bytes) {
        ESP_LOGW(TAG, "Recording stopped early: saved %zu/%zu bytes", bytes_written, total_bytes);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Recording saved successfully: %s (%zu bytes)", filepath, bytes_written);
    return ESP_OK;
}
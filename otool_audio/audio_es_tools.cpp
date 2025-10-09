/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_es_tools.h"
#include "audio_config.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <new>
#include <sys/stat.h>

#ifdef USE_PCM_TEST_A
// test_a.pcm 可用时的处理逻辑
extern const uint8_t _binary_test_a_pcm_start[];
extern const uint8_t _binary_test_a_pcm_end[];
#endif

#ifdef USE_PCM_TEST_B  
// test_b.pcm 可用时的处理逻辑
extern const uint8_t _binary_test_b_pcm_start[];
extern const uint8_t _binary_test_b_pcm_end[];
#endif

#ifdef USE_PCM_SINE_440HZ
// sine_440Hz_30s_44100Hz_16bit_1ch.pcm 可用时的处理逻辑
extern const uint8_t _binary_sine_440Hz_30s_44100Hz_16bit_1ch_pcm_start[];
extern const uint8_t _binary_sine_440Hz_30s_44100Hz_16bit_1ch_pcm_end[];
#endif

#ifdef USE_PCM_STARTUP_1CH
// startup_1ch.pcm 可用时的处理逻辑
extern const uint8_t _binary_startup_1ch_pcm_start[];
extern const uint8_t _binary_startup_1ch_pcm_end[];
#endif

static const char *TAG = "audio_es_tools";

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

    ESP_LOGI(TAG, "Initializing I2S channels (TX and RX) with port %d, %s, %d Hz, %d bits...", 
             i2s_port_num, 
             (audio_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
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
    
    ESP_LOGI(TAG, "I2S channels created successfully");
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
             (audio_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
             (int)sample_rate, 
             (int)bits_per_sample);
    
    esp_err_t ret = ESP_OK;
    
    // 根据声道数量设置slot模式
    i2s_slot_mode_t slot_mode = (audio_channels == AUDIO_CHANNELS_MONO) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    
    // 根据位深度设置数据位宽
    i2s_data_bit_width_t data_bit_width;
    switch (bits_per_sample) {
        case AUDIO_BITS_16:
            data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_BITS_24:
            data_bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            break;
        case AUDIO_BITS_32:
            data_bit_width = I2S_DATA_BIT_WIDTH_32BIT;
            break;
        default:
            data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
    }
    
    // 配置 I2S TX 标准模式
    gpio_num_t tx_dout_pin = i2s_cross_data_pins ? i2s_data_in_pin : i2s_data_out_pin;
    gpio_num_t tx_din_pin  = i2s_cross_data_pins ? i2s_data_out_pin : I2S_GPIO_UNUSED;
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(data_bit_width, slot_mode),
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

    ESP_LOGI(TAG, "Configuring I2S RX channel with %s, %d Hz, %d bits...", 
             (audio_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
             (int)sample_rate, 
             (int)bits_per_sample);
    
    esp_err_t ret = ESP_OK;
    
    // 根据声道数量设置slot模式
    i2s_slot_mode_t slot_mode = (audio_channels == AUDIO_CHANNELS_MONO) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    
    // 根据位深度设置数据位宽
    i2s_data_bit_width_t data_bit_width;
    switch (bits_per_sample) {
        case AUDIO_BITS_16:
            data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case AUDIO_BITS_24:
            data_bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            break;
        case AUDIO_BITS_32:
            data_bit_width = I2S_DATA_BIT_WIDTH_32BIT;
            break;
        default:
            data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
    }
    
    // 配置 I2S RX 标准模式
    // cross 模式下：保持你原先“交叉”关系，使 RX 的 din 使用 data_out_pin；否则使用常规 data_in_pin。
    gpio_num_t rx_din_pin = i2s_cross_data_pins ? i2s_data_out_pin : i2s_data_in_pin;
    gpio_num_t rx_dout_pin = i2s_cross_data_pins ? i2s_data_in_pin : I2S_GPIO_UNUSED;
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(data_bit_width, slot_mode),
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
    
    // 启用 I2S RX 通道
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
    
    ESP_LOGI(TAG, "I2S RX channel disabled successfully");
    return ESP_OK;
}

// ES8311和ES7210的具体实现已移至对应的独立源文件中
// ES8311相关函数实现在 audio_es_es8311.cpp
// ES7210相关函数实现在 audio_es_es7210.cpp

esp_err_t audio_es_tools::audio_system_init(i2c_master_bus_handle_t i2c_bus_handle, i2s_port_t i2s_port_num, audio_channels_t channels, audio_sample_rate_t sample_rate, audio_bits_per_sample_t bits_per_sample)
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

    // 存储I2C总线句柄和I2S配置参数
    this->i2c_bus_handle = i2c_bus_handle;
    this->i2s_port_num = i2s_port_num;
    this->audio_channels = channels;
    this->sample_rate = sample_rate;
    this->bits_per_sample = bits_per_sample;

    ESP_LOGI(TAG, "Initializing audio system with I2C bus handle, I2S port %d, %s, %d Hz, %d bits...", 
             i2s_port_num, 
             (channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
             (int)sample_rate, 
             (int)bits_per_sample);
    
    // 根据需要初始化相应的模块
    esp_err_t ret = ESP_OK;

    // 按需只准备 I2S 通道，codec 由外部手动调用 init，以实现解耦
    ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    system_initialized = true;
    
    ESP_LOGI(TAG, "Audio system initialized successfully with I2S port %d, %s, %d Hz, %d bits", 
             i2s_port_num,
             (channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
             (int)sample_rate,
             (int)bits_per_sample);
    return ESP_OK;
}

esp_err_t audio_es_tools::audio_system_deinit()
{
    if (!system_initialized) {
        ESP_LOGW(TAG, "Audio system not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing audio system...");
    suppress_release = true;  // 避免在两个 codec 先后 deinit 间隙多次释放 I2S
    
    // 去初始化所有已初始化的模块（各自安全判断）
    if (es8311_initialized) es8311_deinit();
    if (es7210_initialized) es7210_deinit();
    // 现在统一处理 I2S 释放
    suppress_release = false;
    try_release_i2s();
    // 若两个都释放了会自动释放 I2S
    
    system_initialized = false;
    
    ESP_LOGI(TAG, "Audio system deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::play_and_record_test()
{
    // 播放音乐测试可在系统已创建 I2S 且已初始化播放设备后运行
    if (!play_dev || !es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting play and record test...");
    
    esp_err_t ret = ESP_OK;
    
    // 1. 测试播放功能
    if (es8311_initialized && play_dev) {
        ESP_LOGI(TAG, "Testing playback functionality...");
        ret = play_music_test();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Playback test failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // 播放和录音之间的间隔
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        ESP_LOGW(TAG, "ES8311 not initialized, skipping playback test");
    }
    
    // 2. 测试录音功能
    if (es7210_initialized && record_dev) {
        ESP_LOGI(TAG, "Testing record functionality...");
        ret = record_test(3000);  // 录音3秒
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record test failed: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "ES7210 not initialized, skipping record test");
    }
    
    ESP_LOGI(TAG, "Play and record test completed successfully");
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

esp_err_t audio_es_tools::play_music_test()
{
    if (!system_initialized) {
        ESP_LOGE(TAG, "Audio system not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting music test...");
    
    // 检查可用的PCM资源
    ESP_LOGI(TAG, "Available PCM resources: %d", get_available_pcm_count());
    ESP_LOGI(TAG, "PCM Test A available: %s", is_pcm_test_a_available() ? "YES" : "NO");
    ESP_LOGI(TAG, "PCM Test B available: %s", is_pcm_test_b_available() ? "YES" : "NO");
    ESP_LOGI(TAG, "PCM Sine 440Hz available: %s", is_pcm_sine_440hz_available() ? "YES" : "NO");
    
    // 使用自动模式播放第一个可用的音频文件
    esp_err_t ret = play_audio_file(AUDIO_FILE_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Music test failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Music test completed successfully");
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
    
    // 获取当前音频格式信息
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)this->sample_rate;
    fs.channel = (this->audio_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
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
            
            esp_err_t write_ret = esp_codec_dev_write(play_dev, data, buffer_size);
            if (write_ret != ESP_OK) {
                ESP_LOGE(TAG, "Cycle #%d playback failed: %s", cycle_count, esp_err_to_name(write_ret));
                // 播放失败不退出循环，继续下一轮录音
            } else {
                ESP_LOGI(TAG, "Cycle #%d: Playback completed -> bytes written: %d", cycle_count, buffer_size);
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
        
        esp_err_t write_ret = esp_codec_dev_write(play_dev, data, buffer_size);
        if (write_ret != ESP_OK) {
            ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(write_ret));
        } else {
            ESP_LOGI(TAG, "Playback completed successfully -> bytes written: %d", buffer_size);
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

bool audio_es_tools::is_pcm_test_a_available() const
{
#ifdef USE_PCM_TEST_A
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_test_b_available() const
{
#ifdef USE_PCM_TEST_B
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_sine_440hz_available() const
{
#ifdef USE_PCM_SINE_440HZ
    return true;
#else
    return false;
#endif
}

bool audio_es_tools::is_pcm_startup_1ch_available() const
{
#ifdef USE_PCM_STARTUP_1CH
    return true;
#else
    return false;
#endif
}

int audio_es_tools::get_available_pcm_count() const
{
    int count = 0;
    
#ifdef USE_PCM_TEST_A
    count++;
#endif

#ifdef USE_PCM_TEST_B
    count++;
#endif

#ifdef USE_PCM_STARTUP_1CH
    count++;
#endif

#ifdef USE_PCM_SINE_440HZ
    count++;
#endif

    return count;
}

const char* audio_es_tools::get_audio_file_name(audio_file_type_t audio_type) const
{
    switch (audio_type) {
        case AUDIO_FILE_TEST_A:
            return "test_a.pcm";
        case AUDIO_FILE_TEST_B:
            return "test_b.pcm";
        case AUDIO_FILE_STARTUP_1CH:
            return "startup_1ch.pcm";
        case AUDIO_FILE_SINE_440HZ:
            return "sine_440Hz_30s_44100Hz_16bit_1ch.pcm";
        case AUDIO_FILE_AUTO:
            return "auto";
        default:
            return "unknown";
    }
}

bool audio_es_tools::is_audio_file_available(audio_file_type_t audio_type) const
{
    switch (audio_type) {
        case AUDIO_FILE_TEST_A:
            return is_pcm_test_a_available();
        case AUDIO_FILE_TEST_B:
            return is_pcm_test_b_available();
        case AUDIO_FILE_STARTUP_1CH:
            return is_pcm_startup_1ch_available();
        case AUDIO_FILE_SINE_440HZ:
            return is_pcm_sine_440hz_available();
        case AUDIO_FILE_AUTO:
            return (get_available_pcm_count() > 0);
        default:
            return false;
    }
}

esp_err_t audio_es_tools::play_audio_file_impl(audio_file_type_t audio_type)
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

    ESP_LOGI(TAG, "Playing audio file: %s", get_audio_file_name(audio_type));

    const uint8_t *pcm_start = nullptr;
    size_t pcm_len = 0;
    audio_file_type_t selected_type = audio_type;

    // 如果是自动模式，选择第一个可用的文件
    if (audio_type == AUDIO_FILE_AUTO) {
        if (is_pcm_test_a_available()) {
            selected_type = AUDIO_FILE_TEST_A;
        } else if (is_pcm_test_b_available()) {
            selected_type = AUDIO_FILE_TEST_B;
        } else if (is_pcm_startup_1ch_available()) {
            selected_type = AUDIO_FILE_STARTUP_1CH;
        } else if (is_pcm_sine_440hz_available()) {
            selected_type = AUDIO_FILE_SINE_440HZ;
        } else {
            ESP_LOGE(TAG, "No audio files available");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "Auto mode selected: %s", get_audio_file_name(selected_type));
    }

    // 根据选择的类型获取PCM数据
    switch (selected_type) {
        case AUDIO_FILE_TEST_A:
#ifdef USE_PCM_TEST_A
            pcm_start = _binary_test_a_pcm_start;
            pcm_len = _binary_test_a_pcm_end - _binary_test_a_pcm_start;
#else
            ESP_LOGE(TAG, "test_a.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_TEST_B:
#ifdef USE_PCM_TEST_B
            pcm_start = _binary_test_b_pcm_start;
            pcm_len = _binary_test_b_pcm_end - _binary_test_b_pcm_start;
#else
            ESP_LOGE(TAG, "test_b.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_STARTUP_1CH:
#ifdef USE_PCM_STARTUP_1CH
            pcm_start = _binary_startup_1ch_pcm_start;
            pcm_len = _binary_startup_1ch_pcm_end - _binary_startup_1ch_pcm_start;
#else
            ESP_LOGE(TAG, "startup_1ch.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        case AUDIO_FILE_SINE_440HZ:
#ifdef USE_PCM_SINE_440HZ
            pcm_start = _binary_sine_440Hz_30s_44100Hz_16bit_1ch_pcm_start;
            pcm_len = _binary_sine_440Hz_30s_44100Hz_16bit_1ch_pcm_end - _binary_sine_440Hz_30s_44100Hz_16bit_1ch_pcm_start;
#else
            ESP_LOGE(TAG, "sine_440Hz_30s_44100Hz_16bit_1ch.pcm not compiled in");
            return ESP_ERR_NOT_SUPPORTED;
#endif
            break;

        default:
            ESP_LOGE(TAG, "Invalid audio file type: %d", selected_type);
            return ESP_ERR_INVALID_ARG;
    }

    if (!pcm_start || pcm_len == 0) {
        ESP_LOGE(TAG, "Invalid PCM data for %s", get_audio_file_name(selected_type));
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "PCM data size: %zu bytes", pcm_len);

    // 播放PCM数据
    esp_err_t ret = esp_codec_dev_write(play_dev, (uint8_t*)pcm_start, pcm_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to play audio: %s", esp_err_to_name(ret));
        return ret;
    }

    // 播放完成后，适度清理音频管道，避免残留声音
    esp_err_t clear_ret = clear_audio_pipeline(80);
    if (clear_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear audio pipeline: %s", esp_err_to_name(clear_ret));
    }

    ESP_LOGI(TAG, "Audio playbook completed successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::play_audio_file(audio_file_type_t audio_type, audio_playback_mode_t mode)
{
    if (mode == AUDIO_PLAYBACK_BLOCKING) {
        return play_audio_file_impl(audio_type);
    }

    if (playback_task_handle) {
        ESP_LOGW(TAG, "Playback task already running");
        return ESP_ERR_INVALID_STATE;
    }

    playback_task_args* args = new (std::nothrow) playback_task_args{this, audio_type};
    if (!args) {
        ESP_LOGE(TAG, "Failed to allocate playback task args");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_ret = xTaskCreate(playback_task_entry, "audio_play_task", 4096, args, 5, &playback_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create playback task");
        delete args;
        playback_task_handle = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Async playback task started for %s", get_audio_file_name(audio_type));
    return ESP_OK;
}

void audio_es_tools::playback_task_entry(void* param)
{
    auto* args = static_cast<playback_task_args*>(param);
    audio_es_tools* instance = args->instance;
    audio_file_type_t audio_type = args->audio_type;
    delete args;

    esp_err_t result = instance->play_audio_file_impl(audio_type);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Async playback failed for %s: %s", instance->get_audio_file_name(audio_type), esp_err_to_name(result));
    }

    instance->playback_task_handle = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t audio_es_tools::clear_audio_pipeline(uint32_t silence_duration_ms)
{
    if (!play_dev || !es8311_initialized) {
        ESP_LOGW(TAG, "Playback device not ready, skipping pipeline clear");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Clearing audio pipeline with %lu ms silence...", silence_duration_ms);

    // 计算静音数据大小
    uint32_t sample_rate_hz = (uint32_t)sample_rate;
    uint32_t channels = (uint32_t)audio_channels;
    uint32_t bits_per_sample_val = (uint32_t)bits_per_sample;
    uint32_t bytes_per_sample = (bits_per_sample_val * channels) / 8;
    size_t silence_size = (sample_rate_hz * bytes_per_sample * silence_duration_ms) / 1000;

    // 分配并填充静音数据
    uint8_t *silence_data = (uint8_t *)calloc(silence_size, 1); // calloc自动填充为0
    if (!silence_data) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for silence buffer", silence_size);
        return ESP_ERR_NO_MEM;
    }

    // 发送静音数据
    esp_err_t ret = esp_codec_dev_write(play_dev, silence_data, silence_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write silence data: %s", esp_err_to_name(ret));
        free(silence_data);
        return ret;
    }

    // 等待静音播放完成
    vTaskDelay(pdMS_TO_TICKS(silence_duration_ms + 50)); // 额外50ms确保完成

    free(silence_data);
    ESP_LOGI(TAG, "Audio pipeline cleared successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::play_all_available_files() const
{
    if (!play_dev || !es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Playing all available audio files...");
    
    int played_count = 0;
    esp_err_t last_error = ESP_OK;

    // 使用AUDIO_FILE_MAX遍历所有枚举值
    for (int i = 0; i < AUDIO_FILE_MAX; i++) {
        audio_file_type_t audio_type = (audio_file_type_t)i;
        
        // 跳过自动模式，因为它不是实际的文件
        if (audio_type == AUDIO_FILE_AUTO) {
            continue;
        }
        
        // 检查文件是否可用
        if (is_audio_file_available(audio_type)) {
            ESP_LOGI(TAG, "Playing file %d/%d: %s", 
                     played_count + 1, get_available_pcm_count(), 
                     get_audio_file_name(audio_type));
            
            // 这里需要使用const_cast，因为play_audio_file不是const函数
            esp_err_t ret = const_cast<audio_es_tools*>(this)->play_audio_file(audio_type);
            
            if (ret == ESP_OK) {
                played_count++;
                
                // 额外清理音频管道，确保没有残留声音
                ret = const_cast<audio_es_tools*>(this)->clear_audio_pipeline(150);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to clear pipeline after %s", get_audio_file_name(audio_type));
                }
                
                // 文件间播放间隔，避免音频重叠
                vTaskDelay(pdMS_TO_TICKS(800));
            } else {
                ESP_LOGW(TAG, "Failed to play %s: %s", 
                         get_audio_file_name(audio_type), esp_err_to_name(ret));
                last_error = ret;
            }
        } else {
            ESP_LOGI(TAG, "Skipping unavailable file: %s", get_audio_file_name(audio_type));
        }
    }

    if (played_count == 0) {
        ESP_LOGW(TAG, "No audio files were available to play");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Completed playing %d audio files", played_count);
    return last_error;
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

    uint32_t channels = (audio_channels == AUDIO_CHANNELS_MONO) ? 1u : 2u;
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
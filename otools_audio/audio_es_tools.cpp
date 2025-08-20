/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_es_tools.h"
#include "audio_config.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "es7210_adc.h"

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

static const char *TAG = "audio_es_tools";

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
}

// 析构函数
audio_es_tools::~audio_es_tools()
{
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

esp_err_t audio_es_tools::es8311_init()
{
    if (es8311_initialized) {
        ESP_LOGW(TAG, "ES8311 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ES8311 (DAC/Playback)...");
    
    esp_err_t ret = ESP_OK;
    
    // 确保 I2S 通道存在
    ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (play_dev) {
        ESP_LOGW(TAG, "ES8311 codec device already created");
        return ESP_OK;
    }

    // 仅播放其实只需要 TX，但底层数据接口可能允许同时给 RX，保持灵活
    if (!tx_configured) {
        ret = i2s_tx_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure I2S TX: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus handle not set, cannot create codec device");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Creating ES8311 codec device...");

    
    // 1. 创建 I2S 数据接口
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.tx_handle = tx_handle;
    i2s_cfg.rx_handle = rx_handle;
    
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!data_if) {
        ESP_LOGE(TAG, "Failed to create I2S data interface");
        return ESP_FAIL;
    }

    // 2. 创建 I2C 控制接口
    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES8311_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = i2c_bus_handle; 
    
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        ESP_LOGE(TAG, "Failed to create I2C control interface");
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }
    
    
    // 3. 创建 GPIO 接口
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!gpio_if) {
        ESP_LOGE(TAG, "Failed to create GPIO interface");
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }
    
    // 4. 创建 ES8311 编解码器接口
    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
    es8311_cfg.ctrl_if = ctrl_if;
    es8311_cfg.gpio_if = gpio_if;
    es8311_cfg.pa_pin = pa_pin;
    es8311_cfg.use_mclk = true;
    es8311_cfg.pa_reverted = false;
    
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    if (!codec_if) {
        ESP_LOGE(TAG, "Failed to create ES8311 codec interface");
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }
    
    // 5. 创建并配置播放设备
    esp_codec_dev_cfg_t codec_dev_cfg = {};
    codec_dev_cfg.codec_if = codec_if;
    codec_dev_cfg.data_if = data_if;
    codec_dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_OUT;
    
    play_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (!play_dev) {
        ESP_LOGE(TAG, "Failed to create codec device");
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }

    // 6. 设置声音大小为80%
    ret = esp_codec_dev_set_out_vol(play_dev, 80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set output volume: %s", esp_err_to_name(ret));
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ret;
    }
    
    // 7. 打开播放设备
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)sample_rate;
    fs.channel = (uint32_t)audio_channels;
    fs.bits_per_sample = (uint32_t)bits_per_sample;
    
    ret = esp_codec_dev_open(play_dev, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec device: %s", esp_err_to_name(ret));
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ret;
    }
    
    es8311_initialized = true;
    incr_i2s_user();
    es8311_sleeping = false;
    
    ESP_LOGI(TAG, "ES8311 initialized successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::es8311_deinit()
{
    if (!es8311_initialized) {
        ESP_LOGW(TAG, "ES8311 not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ES8311...");
    
    // 关闭并删除播放设备
    if (play_dev) {
        esp_codec_dev_close(play_dev);
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
    }
    // 由 esp_codec_dev_close 完成禁用，同步标志
    tx_configured = false;
    
    // 仅减少引用计数；真正释放由引用计数归零时处理
    decr_i2s_user();
    
    es8311_initialized = false;
    es8311_sleeping = false;
    
    ESP_LOGI(TAG, "ES8311 deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::es7210_init()
{
    if (es7210_initialized) {
        ESP_LOGW(TAG, "ES7210 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ES7210 (ADC/Record)...");
    
    esp_err_t ret = ESP_OK;
    
    // 1. 检查前置条件
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus handle not set, cannot create ES7210 device");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 确保 I2S 通道存在并配置 RX
    ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    if (!rx_configured) {
        ret = i2s_rx_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure I2S RX: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // 2. 创建 I2S 数据接口（仅RX用于录音）
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.rx_handle = rx_handle;
    i2s_cfg.tx_handle = NULL;  // ES7210只需要RX
    
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!data_if) {
        ESP_LOGE(TAG, "Failed to create I2S data interface for ES7210");
        return ESP_FAIL;
    }

    // 3. 创建 I2C 控制接口
    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES7210_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = i2c_bus_handle;
    
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        ESP_LOGE(TAG, "Failed to create I2C control interface for ES7210");
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }
    
    // 4. 创建 ES7210 编解码器接口
    es7210_codec_cfg_t es7210_cfg = {};
    es7210_cfg.ctrl_if = ctrl_if;
    es7210_cfg.master_mode = false;  // ES7210工作在从模式
    es7210_cfg.mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC3;  // 选择麦克风1和3
    es7210_cfg.mclk_src = ES7210_MCLK_FROM_PAD;
    es7210_cfg.mclk_div = 256;  // MCLK/LRCK = 256 (44.1kHz)
    
    const audio_codec_if_t *codec_if = es7210_codec_new(&es7210_cfg);
    if (!codec_if) {
        ESP_LOGE(TAG, "Failed to create ES7210 codec interface");
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }
    
    // 5. 创建并配置录音设备
    esp_codec_dev_cfg_t codec_dev_cfg = {};
    codec_dev_cfg.codec_if = codec_if;
    codec_dev_cfg.data_if = data_if;
    codec_dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;  // 输入设备（录音）
    
    record_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (!record_dev) {
        ESP_LOGE(TAG, "Failed to create ES7210 codec device");
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ESP_FAIL;
    }

    // 6. 设置输入增益（可选）
    ret = esp_codec_dev_set_in_gain(record_dev, 30.0);  // 30dB增益
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set input gain: %s", esp_err_to_name(ret));
        // 继续执行，增益设置失败不是致命错误
    }
    
    // 7. 打开录音设备
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)sample_rate;
    fs.channel = 1;  // 单通道录音
    fs.bits_per_sample = (uint32_t)bits_per_sample;
    
    ret = esp_codec_dev_open(record_dev, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open ES7210 codec device: %s", esp_err_to_name(ret));
        esp_codec_dev_delete(record_dev);
        record_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        audio_codec_delete_data_if(data_if);
        return ret;
    }
    
    es7210_initialized = true;
    incr_i2s_user();
    es7210_sleeping = false;
    
    ESP_LOGI(TAG, "ES7210 initialized successfully");
    return ESP_OK;
}

esp_err_t audio_es_tools::es7210_deinit()
{
    if (!es7210_initialized) {
        ESP_LOGW(TAG, "ES7210 not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ES7210...");
    
    // 关闭并删除录音设备
    if (record_dev) {
        esp_codec_dev_close(record_dev);
        esp_codec_dev_delete(record_dev);
        record_dev = NULL;
    }
    // 由 esp_codec_dev_close 完成禁用，同步标志
    rx_configured = false;
    
    es7210_initialized = false;
    decr_i2s_user();
    es7210_sleeping = false;
    
    ESP_LOGI(TAG, "ES7210 deinitialized successfully");
    return ESP_OK;
}

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

esp_err_t audio_es_tools::es8311_sleep()
{
    if (!es8311_initialized) {
        ESP_LOGE(TAG, "ES8311 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Putting ES8311 to sleep...");
    
    esp_err_t ret = ESP_OK;
    
    // 播放设备进入低功耗模式
    if (play_dev) {
        // 1. 设置输出音量为0（静音）
        ret = esp_codec_dev_set_out_vol(play_dev, 0.0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to mute ES8311: %s", esp_err_to_name(ret));
        }
        
        // 2. 设置设备在关闭时禁用
        ret = esp_codec_set_disable_when_closed(play_dev, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set ES8311 disable when closed: %s", esp_err_to_name(ret));
        }
        
        // 3. 关闭播放设备
        ret = esp_codec_dev_close(play_dev);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to close ES8311: %s", esp_err_to_name(ret));
        }
        
        ESP_LOGI(TAG, "ES8311 play device muted, disabled and closed");
    }
    
    es8311_sleeping = true;
    
    ESP_LOGI(TAG, "ES8311 sleep mode enabled");
    return ESP_OK;
}

esp_err_t audio_es_tools::es7210_sleep()
{
    if (!es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Putting ES7210 to sleep...");
    
    esp_err_t ret = ESP_OK;
    
    // 录音设备进入低功耗模式
    if (record_dev) {
        // 1. 设置输入增益为0（静音）
        ret = esp_codec_dev_set_in_gain(record_dev, 0.0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to mute ES7210: %s", esp_err_to_name(ret));
        }
        
        // 2. 设置设备在关闭时禁用
        ret = esp_codec_set_disable_when_closed(record_dev, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set ES7210 disable when closed: %s", esp_err_to_name(ret));
        }
        
        // 3. 关闭录音设备
        ret = esp_codec_dev_close(record_dev);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to close ES7210: %s", esp_err_to_name(ret));
        }
        
        ESP_LOGI(TAG, "ES7210 record device muted, disabled and closed");
    }
    
    // 根据需要将 I2S 引脚置为高阻（睡眠优化），PA 由 ES8311 编解码器托管
    if (pins_high_z_on_sleep) {
        _gpio_set_high_z(i2s_mck_pin);
        _gpio_set_high_z(i2s_bck_pin);
        _gpio_set_high_z(i2s_ws_pin);
        _gpio_set_high_z(i2s_data_out_pin);
        _gpio_set_high_z(i2s_data_in_pin);
        if (pa_pin != GPIO_NUM_NC) {
            _gpio_set_high_z(pa_pin);
        }
        ESP_LOGI(TAG, "I2S + PA GPIOs set to High-Z for sleep");
    }
    es7210_sleeping = true;
    
    ESP_LOGI(TAG, "ES7210 sleep mode enabled");
    return ESP_OK;
}

esp_err_t audio_es_tools::audio_system_sleep()
{
    if (!system_initialized) {
        ESP_LOGE(TAG, "Audio system not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Putting audio system to sleep...");
    
    // 让所有已初始化的模块进入睡眠
    if (es8311_initialized) {
        es8311_sleep();
    }
    
    if (es7210_initialized) {
        es7210_sleep();
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

    // 分配录音缓冲区
    const size_t sample_rate = 44100;
    const size_t channels = 1;
    const size_t bits_per_sample = 16;
    const size_t bytes_per_sample = bits_per_sample / 8;
    const size_t buffer_size = (sample_rate * channels * bytes_per_sample * record_duration_ms) / 1000;
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate record buffer (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Record buffer allocated: %zu bytes", buffer_size);
    ESP_LOGI(TAG, "Sample rate: %zu Hz, Channels: %zu, Bits: %zu", sample_rate, channels, bits_per_sample);

    // 开始录音
    TickType_t start_time = xTaskGetTickCount();
    int bytes_read = esp_codec_dev_read(record_dev, record_buffer, buffer_size);
    TickType_t end_time = xTaskGetTickCount();
    
    uint32_t actual_duration = pdTICKS_TO_MS(end_time - start_time);
    
    if (bytes_read < 0) {
        ESP_LOGE(TAG, "Record failed with error code: %d", bytes_read);
        free(record_buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Record completed successfully");
    ESP_LOGI(TAG, "Requested duration: %lu ms, Actual duration: %lu ms", record_duration_ms, actual_duration);
    ESP_LOGI(TAG, "Expected bytes: %zu, Actual bytes read: %d", buffer_size, bytes_read);
    
    // 简单的音频数据分析
    if (bytes_read > 0) {
        int16_t *samples = (int16_t *)record_buffer;
        size_t sample_count = bytes_read / sizeof(int16_t);
        
        // 计算音频信号的基本统计信息
        int32_t sum = 0;
        int16_t min_val = INT16_MAX;
        int16_t max_val = INT16_MIN;
        
        for (size_t i = 0; i < sample_count; i++) {
            int16_t sample = samples[i];
            sum += abs(sample);
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        
        int32_t avg_amplitude = sum / sample_count;
        ESP_LOGI(TAG, "Audio analysis - Samples: %zu, Avg amplitude: %ld, Range: [%d, %d]", 
                 sample_count, avg_amplitude, min_val, max_val);
        
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

esp_err_t audio_es_tools::record_and_playback_test(uint32_t record_duration_seconds)
{
    if (!es7210_initialized || !es8311_initialized) {
        ESP_LOGE(TAG, "Audio devices not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev || !play_dev) {
        ESP_LOGE(TAG, "Record or playback device not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Record and playback test (%lu seconds) ===", record_duration_seconds);
    
    // 仿照示例代码的while循环结构（实际只执行一次）
    while (true)
    {
        // 获取当前音频格式信息
        esp_codec_dev_sample_info_t fs = {};
        fs.sample_rate = (uint32_t)this->sample_rate;
        fs.channel = (uint32_t)this->audio_channels;
        fs.bits_per_sample = (uint32_t)this->bits_per_sample;
        
        // 计算缓冲区大小（仿照示例代码的计算方式）
        uint8_t *data = (uint8_t *)malloc(fs.sample_rate * fs.channel * (fs.bits_per_sample >> 3) * record_duration_seconds);
        if (data == NULL) {
            {
                size_t required = fs.sample_rate * fs.channel * (fs.bits_per_sample >> 3) * record_duration_seconds;
                size_t free_heap = esp_get_free_heap_size();
                size_t required_kb = (required + 1023) / 1024;
                size_t free_kb = (free_heap + 1023) / 1024;
                size_t missing_kb = (free_heap < required) ? ((required - free_heap + 1023) / 1024) : 0;

                ESP_LOGE(TAG, "Failed to allocate memory for recording");
                ESP_LOGE(TAG, "Required: %zu bytes (%zu KB), Free: %zu bytes (%zu KB), Missing: %zu KB",
                         required, required_kb, free_heap, free_kb, missing_kb);
            }
            break;
        }
        
        int buffer_size = fs.sample_rate * fs.channel * (fs.bits_per_sample >> 3) * record_duration_seconds;
        
        ESP_LOGI(TAG, "Start recording %lu seconds... (buffer: %d bytes)", record_duration_seconds, buffer_size);
        
        // 录音阶段
        esp_err_t ret = esp_codec_dev_read(record_dev, data, buffer_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record failed: %s", esp_err_to_name(ret));
            free(data);
            break;
        }
        
        ESP_LOGI(TAG, "Record completed, start playing...");
        
        // 播放阶段
        ret = esp_codec_dev_write(play_dev, data, buffer_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Playback completed");
        }
        
        // 释放内存
        free(data);
        break;  // 仿照示例代码，只执行一次
    }
    
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

int audio_es_tools::get_available_pcm_count() const
{
    int count = 0;
    
#ifdef USE_PCM_TEST_A
    count++;
#endif

#ifdef USE_PCM_TEST_B
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
        case AUDIO_FILE_SINE_440HZ:
            return is_pcm_sine_440hz_available();
        case AUDIO_FILE_AUTO:
            return (get_available_pcm_count() > 0);
        default:
            return false;
    }
}

esp_err_t audio_es_tools::play_audio_file(audio_file_type_t audio_type)
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

    ESP_LOGI(TAG, "Audio playback completed successfully");
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
                // 播放间隔，避免音频重叠
                vTaskDelay(pdMS_TO_TICKS(500));
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
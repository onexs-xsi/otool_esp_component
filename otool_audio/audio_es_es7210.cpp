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
#include "es7210_adc.h"
#include "esp_log.h"

static const char *TAG = "audio_es7210";

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

esp_err_t audio_es_tools::es7210_init(audio_mic_channel_t mic_channels)
{
    if (es7210_initialized) {
        ESP_LOGW(TAG, "ES7210 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ES7210 (ADC/Record) with %s...", 
             (audio_channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO");
    
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
    es7210_cfg.mic_selected = (uint8_t)mic_channels;  // 使用传入的麦克风通道参数
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

    // 6. 设置录音增益
    ret = esp_codec_dev_set_in_gain(record_dev, record_gain);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set input gain to %.1f dB: %s", record_gain, esp_err_to_name(ret));
    }
    
    // 7. 打开录音设备
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)sample_rate;
    fs.channel = (uint32_t)audio_channels;  // 使用系统配置的声道数
    fs.bits_per_sample = (uint32_t)bits_per_sample;
    // 打印fs
    ESP_LOGI(TAG, "Opening ES7210 with sample rate %d Hz, %d channels, %d bits per sample",
             fs.sample_rate, fs.channel, fs.bits_per_sample);
    
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

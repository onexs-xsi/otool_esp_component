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
#include "esp_log.h"

static const char *TAG = "audio_es8311";

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

esp_err_t audio_es_tools::es8311_init(audio_channels_t channels)
{
    if (es8311_initialized) {
        ESP_LOGW(TAG, "ES8311 already initialized");
        return ESP_OK;
    }

    // 保存TX声道配置
    this->tx_channels = channels;

    ESP_LOGI(TAG, "Initializing ES8311 (DAC/Playback) with %s...", 
             (channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO");
    
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

    
    // 1. 使用共享 I2S 数据接口(避免 mode conflict)
    if (!shared_i2s_data_if) {
        ESP_LOGE(TAG, "Shared I2S data interface not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    const audio_codec_data_if_t *data_if = shared_i2s_data_if;

    // 2. 创建 I2C 控制接口
    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES8311_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = i2c_bus_handle; 
    
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        ESP_LOGE(TAG, "Failed to create I2C control interface");
        // 不要删除共享接口
        return ESP_FAIL;
    }
    
    
    // 3. 创建 GPIO 接口
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!gpio_if) {
        ESP_LOGE(TAG, "Failed to create GPIO interface");
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
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
        // 不要删除共享接口
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
        // 不要删除共享接口
        return ESP_FAIL;
    }

    // 6. 设置播放音量
    ret = esp_codec_dev_set_out_vol(play_dev, volume);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set output volume to %.1f: %s", volume, esp_err_to_name(ret));
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
        return ret;
    }
    
    // 7. 打开播放设备
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)sample_rate;
    fs.channel = (uint32_t)channels;  // 使用传入的声道参数
    fs.bits_per_sample = (uint32_t)bits_per_sample;
    
    ret = esp_codec_dev_open(play_dev, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec device: %s", esp_err_to_name(ret));
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
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

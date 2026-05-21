/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_tools.h"
#include "audio_es_es7210.h"
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

esp_err_t audio_tools::es7210_init(audio_channels_t channels, audio_mic_channel_t mic_channels, es7210_tdm_mode_t use_tdm)
{
    if (es7210_initialized) {
        ESP_LOGW(TAG, "ES7210 already initialized");
        return ESP_OK;
    }

    if (es8311_initialized && es8311_has_adc_path()) {
        ESP_LOGE(TAG, "ES8311 ADC path already active; deinitialize ES8311 ADC before starting ES7210");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t requested_mask = static_cast<uint8_t>(mic_channels);
    if (requested_mask == 0u) {
        ESP_LOGE(TAG, "ES7210 init requires at least one microphone (mask=0x%02X)", requested_mask);
        return ESP_ERR_INVALID_ARG;
    }

    const bool tdm_enabled = (use_tdm == ES7210_TDM_ENABLED);

    if (!tdm_enabled && (channels == AUDIO_CHANNELS_3CHs || channels == AUDIO_CHANNELS_4CHs)) {
        ESP_LOGE(TAG, "Channels=%d requires TDM mode. Please call es7210_init(..., ES7210_TDM_ENABLED)", static_cast<int>(channels));
        return ESP_ERR_INVALID_ARG;
    }

    if (!tdm_enabled && (channels == AUDIO_CHANNELS_MONO || channels == AUDIO_CHANNELS_STEREO)) {
        const bool supported = (mic_channels == AUDIO_MIC_CHANNEL_1) ||
                               (mic_channels == AUDIO_MIC_CHANNEL_2) ||
                               (mic_channels == AUDIO_MIC_CHANNEL_12);
        if (!supported) {
            ESP_LOGE(TAG, "For %s mode only MIC1/MIC2/MIC12 are supported (mask=0x%02X)",
                     (channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO",
                     requested_mask);
            return ESP_ERR_INVALID_ARG;
        }
    }

    bool mic1_enabled = (requested_mask & 0x01u) != 0u;
    bool mic2_enabled = (requested_mask & 0x02u) != 0u;
    bool mic3_enabled = (requested_mask & 0x04u) != 0u;
    bool mic4_enabled = (requested_mask & 0x08u) != 0u;

    bool mute_mic1_after_init = false;

    uint8_t effective_mask = requested_mask;
    uint8_t active_mask = requested_mask;

    if (!tdm_enabled) {
        if (mic2_enabled && !mic1_enabled && !mic3_enabled && !mic4_enabled) {
            ESP_LOGW(TAG, "MIC2-only selected; enabling MIC1 as muted companion for standard I2S");
            effective_mask |= static_cast<uint8_t>(AUDIO_MIC_CHANNEL_1);
            mic1_enabled = true;
            mute_mic1_after_init = true;
        }
        active_mask = effective_mask;
    } else {
        effective_mask = static_cast<uint8_t>(AUDIO_MIC_CHANNEL_ALL);
        active_mask = requested_mask;
    }

    auto count_bits = [](uint8_t value) -> uint8_t {
        uint8_t count = 0;
        while (value) {
            count += (value & 0x01u);
            value >>= 1;
        }
        return count;
    };

    const uint8_t requested_mic_count = count_bits(requested_mask);
    const uint8_t effective_mic_count_raw = count_bits(effective_mask);
    if (effective_mic_count_raw == 0u) {
        ESP_LOGE(TAG, "Effective mic mask resolves to zero (requested=0x%02X)", requested_mask);
        return ESP_ERR_INVALID_ARG;
    }

    es7210_use_tdm = tdm_enabled;

    if (es7210_use_tdm && channels != AUDIO_CHANNELS_3CHs && channels != AUDIO_CHANNELS_4CHs) {
        ESP_LOGW(TAG, "TDM mode requested but channels=%d is not 3CH/4CH; continuing with provided channels",
                 static_cast<int>(channels));
    }

    audio_channels_t effective_channels = es7210_use_tdm ? AUDIO_CHANNELS_STEREO : channels;
    audio_mic_channel_t effective_mic_channels = static_cast<audio_mic_channel_t>(effective_mask);
    const uint8_t active_mic_count = count_bits(active_mask);
    const uint8_t effective_mic_count = es7210_use_tdm ? 4u : effective_mic_count_raw;

    ESP_LOGI(TAG,
             "Initializing ES7210 (ADC) request mask=0x%02X (%u mics) -> effective mask=0x%02X (%u mics), mode=%s",
             requested_mask,
             requested_mic_count,
             effective_mask,
             effective_mic_count,
             es7210_use_tdm ? "TDM-4CH" : ((channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO"));

    if (es7210_use_tdm && active_mic_count == 0u) {
        ESP_LOGE(TAG, "TDM mode requires at least one active microphone channel");
        return ESP_ERR_INVALID_ARG;
    }

    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus handle not set, cannot create ES7210 device");
        return ESP_ERR_INVALID_STATE;
    }

    this->rx_channels = effective_channels;
    rx_tdm_slot_count = es7210_use_tdm ? 4u : static_cast<uint8_t>(effective_channels);

    esp_err_t ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    if (rx_configured) {
        ESP_LOGI(TAG, "Reconfiguring I2S RX channel for ES7210 to match new mode");
        esp_err_t rx_stop = i2s_rx_deinit();
        if (rx_stop != ESP_OK) {
            ESP_LOGW(TAG, "Unable to fully disable previous I2S RX config: %s", esp_err_to_name(rx_stop));
        }
    }

    ret = i2s_rx_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2S RX: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 使用共享 I2S 数据接口(避免 mode conflict)
    if (!shared_i2s_data_if) {
        ESP_LOGE(TAG, "Shared I2S data interface not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    const audio_codec_data_if_t *data_if = shared_i2s_data_if;

    // 创建 I2C 控制接口
    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES7210_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = i2c_bus_handle;

    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        ESP_LOGE(TAG, "Failed to create I2C control interface for ES7210");
        // 不要删除共享接口
        return ESP_FAIL;
    }
    
    // 创建 ES7210 编解码器接口
    es7210_codec_cfg_t es7210_cfg = {};
    es7210_cfg.ctrl_if = ctrl_if;
    es7210_cfg.master_mode = false;
    es7210_cfg.mic_selected = static_cast<uint8_t>(effective_mic_channels);
    es7210_cfg.mclk_src = ES7210_MCLK_FROM_PAD;
    es7210_cfg.mclk_div = 256;  // MCLK/LRCK = 256 (标准倍频，适用于8kHz~48kHz)
    
    const audio_codec_if_t *codec_if = es7210_codec_new(&es7210_cfg);
    if (!codec_if) {
        ESP_LOGE(TAG, "Failed to create ES7210 codec interface");
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
        return ESP_FAIL;
    }
    
    // 创建并配置录音设备
    esp_codec_dev_cfg_t codec_dev_cfg = {};
    codec_dev_cfg.codec_if = codec_if;
    codec_dev_cfg.data_if = data_if;
    codec_dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;  // 输入设备（录音）
    
    record_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (!record_dev) {
        ESP_LOGE(TAG, "Failed to create ES7210 codec device");
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
        return ESP_FAIL;
    }
    
    // 打开录音设备
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(sample_rate);
    fs.channel = (channels == AUDIO_CHANNELS_MONO) ? 1u : 2u;
    fs.bits_per_sample = static_cast<uint32_t>(bits_per_sample);
    
    ESP_LOGI(TAG, "Opening ES7210 with sample rate %d Hz, %d channels, %d bits per sample (TDM=%s)",
             static_cast<int>(fs.sample_rate),
             static_cast<int>(fs.channel),
             static_cast<int>(fs.bits_per_sample),
             es7210_use_tdm ? "YES" : "NO");
    
    ret = esp_codec_dev_open(record_dev, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open ES7210 codec device: %s", esp_err_to_name(ret));
        esp_codec_dev_delete(record_dev);
        record_dev = NULL;
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        // 不要删除共享接口
        return ret;
    }

    // 保存接口对象，便于 deinit 时正确释放（避免泄漏）
    es7210_ctrl_if  = ctrl_if;
    es7210_codec_if = codec_if;

    // 标记为已初始化，以便后续的 mute/gain 操作可以正常执行
    this->mic_channels = effective_mic_channels;
    es7210_initialized = true;
    incr_i2s_user();
    es7210_sleeping = false;
    
    // 设置每个通道的静音状态
    const audio_mic_channel_t channel_map[4] = {
        AUDIO_MIC_CHANNEL_1,
        AUDIO_MIC_CHANNEL_2,
        AUDIO_MIC_CHANNEL_3,
        AUDIO_MIC_CHANNEL_4
    };
    
    for (int ch = 0; ch < 4; ++ch) {
        bool channel_active = (active_mask & (1u << ch)) != 0u;
        bool should_mute = !channel_active;
        if (!es7210_use_tdm && channel_active && ch == 0) {
            should_mute = mute_mic1_after_init;
        }

        esp_err_t mute_ret = es7210_set_mic_channel_mute(channel_map[ch], should_mute);
        if (mute_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to %s MIC%d mute control",
                     should_mute ? "apply" : "clear", ch + 1);
        }
    }

    if (mute_mic1_after_init) {
        ESP_LOGI(TAG, "MIC1 muted to support MIC2-only stereo capture");
    }

    ESP_LOGI(TAG, "ES7210 initialized successfully");
    return ESP_OK;
}

esp_err_t audio_tools::es7210_deinit()
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
    rx_tdm_slot_count = 0;
    es7210_use_tdm = false;

    // 释放 init 时创建的接口对象，避免泄漏
    if (es7210_codec_if) {
        audio_codec_delete_codec_if(es7210_codec_if);
        es7210_codec_if = nullptr;
    }
    if (es7210_ctrl_if) {
        audio_codec_delete_ctrl_if(es7210_ctrl_if);
        es7210_ctrl_if = nullptr;
    }

    es7210_initialized = false;
    decr_i2s_user();
    es7210_sleeping = false;
    
    ESP_LOGI(TAG, "ES7210 deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_tools::es7210_sleep()
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

esp_err_t audio_tools::es7210_set_mic_channel_mute(audio_mic_channel_t mic_channel, bool mute)
{
    // 参数验证：仅允许单一通道
    uint8_t channel_mask = static_cast<uint8_t>(mic_channel);
    if (channel_mask != AUDIO_MIC_CHANNEL_1 &&
        channel_mask != AUDIO_MIC_CHANNEL_2 &&
        channel_mask != AUDIO_MIC_CHANNEL_3 &&
        channel_mask != AUDIO_MIC_CHANNEL_4) {
        ESP_LOGE(TAG, "Invalid mic_channel 0x%02X (only single channel allowed: 0x01/0x02/0x04/0x08)", channel_mask);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized, cannot set channel mute");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 将枚举值转换为通道索引 (0-3)
    int channel_index = -1;
    if (channel_mask == AUDIO_MIC_CHANNEL_1) channel_index = 0;
    else if (channel_mask == AUDIO_MIC_CHANNEL_2) channel_index = 1;
    else if (channel_mask == AUDIO_MIC_CHANNEL_3) channel_index = 2;
    else if (channel_mask == AUDIO_MIC_CHANNEL_4) channel_index = 3;
    
    // 根据数据手册确定寄存器地址和位掩码
    // 0x15 (REG15) 控制 ADC1/ADC2 (MIC1/MIC2)
    // 0x14 (REG14) 控制 ADC3/ADC4 (MIC3/MIC4)
    const uint8_t reg_addr = (channel_index < 2) ? 0x15u : 0x14u;
    const uint8_t bit_mask = (channel_index % 2 == 0) ? 0x01u : 0x02u;
    
    // 读取当前寄存器值
    int current_val = 0;
    int codec_ret = esp_codec_dev_read_reg(record_dev, reg_addr, &current_val);
    if (codec_ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to read mute register 0x%02X for MIC%d (ret=%d)",
                 reg_addr, channel_index + 1, codec_ret);
        return ESP_FAIL;
    }
    
    // 计算新值
    int new_val = mute ? (current_val | bit_mask) : (current_val & ~bit_mask);
    
    // 如果值未改变，直接返回（优化）
    if (new_val == current_val) {
        ESP_LOGD(TAG, "MIC%d mute state unchanged (already %s)",
                 channel_index + 1, mute ? "muted" : "unmuted");
        return ESP_OK;
    }
    
    // 写入新值
    codec_ret = esp_codec_dev_write_reg(record_dev, reg_addr, new_val);
    if (codec_ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to write mute register 0x%02X for MIC%d (ret=%d)",
                 reg_addr, channel_index + 1, codec_ret);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "MIC%d %s (reg 0x%02X: 0x%02X -> 0x%02X)",
             channel_index + 1,
             mute ? "muted" : "unmuted",
             reg_addr,
             current_val,
             new_val);
    
    return ESP_OK;
}

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_tools.h"
#include "audio_es_es8311.h"
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

esp_err_t audio_tools::es8311_init(audio_channels_t channels, es8311_path_mode_t mode)
{
    esp_codec_dec_work_mode_t requested_mode = ESP_CODEC_DEV_WORK_MODE_NONE;
    switch (mode) {
        case ES8311_MODE_DAC:
            requested_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
            break;
        case ES8311_MODE_ADC:
            requested_mode = ESP_CODEC_DEV_WORK_MODE_ADC;
            break;
        case ES8311_MODE_DAC_AND_ADC:
            requested_mode = ESP_CODEC_DEV_WORK_MODE_BOTH;
            break;
        default:
            requested_mode = ESP_CODEC_DEV_WORK_MODE_NONE;
            break;
    }

    const bool enable_dac = (requested_mode & ESP_CODEC_DEV_WORK_MODE_DAC) != 0;
    const bool enable_adc = (requested_mode & ESP_CODEC_DEV_WORK_MODE_ADC) != 0;

    if (!enable_dac && !enable_adc) {
        ESP_LOGE(TAG, "Invalid ES8311 mode: 0x%02X", static_cast<int>(mode));
        return ESP_ERR_INVALID_ARG;
    }

    if (channels != AUDIO_CHANNELS_MONO && channels != AUDIO_CHANNELS_STEREO) {
        ESP_LOGE(TAG, "ES8311 supports MONO/STEREO only, requested=%d", static_cast<int>(channels));
        return ESP_ERR_INVALID_ARG;
    }

    if (es8311_initialized) {
        ESP_LOGW(TAG, "ES8311 already initialized (current mode=%u)", static_cast<unsigned>(es8311_work_mode));
        return ESP_OK;
    }

    if (enable_adc && es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 ADC already active, deinit it before enabling ES8311 ADC");
        return ESP_ERR_INVALID_STATE;
    }

    const char *mode_desc = enable_dac && enable_adc ? "DAC+ADC" : (enable_dac ? "DAC-only" : "ADC-only");

    ESP_LOGI(TAG, "Initializing ES8311 with %s mode (%s)...",
             mode_desc,
             (channels == AUDIO_CHANNELS_MONO) ? "MONO" : "STEREO");

    esp_err_t ret = ensure_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    bool tx_was_configured = tx_configured;
    bool rx_was_configured = rx_configured;

    auto revert_i2s = [&]() {
        if (enable_dac && !tx_was_configured && tx_configured) {
            i2s_tx_deinit();
        }
        if (enable_adc && !rx_was_configured && rx_configured) {
            i2s_rx_deinit();
        }
    };

    if (enable_dac) {
        tx_channels = channels;
        if (!tx_configured) {
            ret = i2s_tx_init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure I2S TX: %s", esp_err_to_name(ret));
                revert_i2s();
                return ret;
            }
        }
    }

    if (enable_adc) {
        rx_channels = channels;
        if (!rx_configured) {
            ret = i2s_rx_init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure I2S RX: %s", esp_err_to_name(ret));
                revert_i2s();
                return ret;
            }
        }
    }

    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus handle not set, cannot create codec device");
        revert_i2s();
        return ESP_ERR_INVALID_STATE;
    }

    if (!shared_i2s_data_if) {
        ESP_LOGE(TAG, "Shared I2S data interface not initialized");
        revert_i2s();
        return ESP_ERR_INVALID_STATE;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES8311_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = i2c_bus_handle;

    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        ESP_LOGE(TAG, "Failed to create I2C control interface");
        revert_i2s();
        return ESP_FAIL;
    }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!gpio_if) {
        ESP_LOGE(TAG, "Failed to create GPIO interface");
        audio_codec_delete_ctrl_if(ctrl_if);
        revert_i2s();
        return ESP_FAIL;
    }

    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.codec_mode = requested_mode;
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
        revert_i2s();
        return ESP_FAIL;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {};
    codec_dev_cfg.codec_if = codec_if;
    codec_dev_cfg.data_if = shared_i2s_data_if;
    codec_dev_cfg.dev_type = enable_dac && enable_adc ? ESP_CODEC_DEV_TYPE_IN_OUT :
                             (enable_dac ? ESP_CODEC_DEV_TYPE_OUT : ESP_CODEC_DEV_TYPE_IN);

    esp_codec_dev_handle_t codec_handle = esp_codec_dev_new(&codec_dev_cfg);
    if (!codec_handle) {
        ESP_LOGE(TAG, "Failed to create codec device");
        audio_codec_delete_codec_if(codec_if);
        audio_codec_delete_gpio_if(gpio_if);
        audio_codec_delete_ctrl_if(ctrl_if);
        revert_i2s();
        return ESP_FAIL;
    }

    auto cleanup = [&]() {
        if (codec_handle) {
            esp_codec_dev_delete(codec_handle);
            codec_handle = nullptr;
        }
        if (codec_if) {
            audio_codec_delete_codec_if(codec_if);
            codec_if = nullptr;
        }
        if (gpio_if) {
            audio_codec_delete_gpio_if(gpio_if);
            gpio_if = nullptr;
        }
        if (ctrl_if) {
            audio_codec_delete_ctrl_if(ctrl_if);
            ctrl_if = nullptr;
        }
        revert_i2s();
    };

    if (enable_dac) {
        ret = esp_codec_dev_set_out_vol(codec_handle, static_cast<int>(volume));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set output volume to %.1f: %s", volume, esp_err_to_name(ret));
            cleanup();
            return ret;
        }
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(sample_rate);
    fs.channel = (channels == AUDIO_CHANNELS_MONO) ? 1u : 2u;
    fs.bits_per_sample = static_cast<uint32_t>(bits_per_sample);

    ret = esp_codec_dev_open(codec_handle, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec device: %s", esp_err_to_name(ret));
        cleanup();
        return ret;
    }

    if (enable_adc) {
        int gain_ret = esp_codec_dev_set_in_gain(codec_handle, record_gain);
        if (gain_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "Failed to apply record gain %.1f dB: %s",
                     record_gain, esp_err_to_name(static_cast<esp_err_t>(gain_ret)));
        }
    }

    es8311_dev_handle = codec_handle;
    if (enable_dac) {
        play_dev = codec_handle;
    }
    if (enable_adc) {
        record_dev = codec_handle;
    }

    es8311_work_mode = requested_mode;
    es8311_initialized = true;
    es8311_sleeping = false;
    incr_i2s_user();

    ESP_LOGI(TAG, "ES8311 initialized successfully (%s, %u Hz, %u bits)",
             mode_desc, fs.sample_rate, fs.bits_per_sample);
    return ESP_OK;
}

esp_err_t audio_tools::es8311_deinit()
{
    if (!es8311_initialized) {
        ESP_LOGW(TAG, "ES8311 not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ES8311 (mode=%u)...", static_cast<unsigned>(es8311_work_mode));

    esp_codec_dev_handle_t handle = es8311_dev_handle ? es8311_dev_handle : play_dev;
    if (!handle && record_dev && es8311_has_adc_path()) {
        handle = record_dev;
    }

    if (handle) {
        esp_codec_dev_close(handle);
        esp_codec_dev_delete(handle);
    }

    if (play_dev == handle) {
        play_dev = NULL;
    }
    if (record_dev == handle) {
        record_dev = NULL;
    }

    if (es8311_has_dac_path()) {
        tx_configured = false;
    }
    if (es8311_has_adc_path()) {
        rx_configured = false;
    }

    es8311_dev_handle = NULL;
    es8311_work_mode = ESP_CODEC_DEV_WORK_MODE_NONE;

    decr_i2s_user();

    es8311_initialized = false;
    es8311_sleeping = false;

    ESP_LOGI(TAG, "ES8311 deinitialized successfully");
    return ESP_OK;
}

esp_err_t audio_tools::es8311_sleep()
{
    if (!es8311_initialized) {
        ESP_LOGE(TAG, "ES8311 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Putting ES8311 to sleep...");
    
    esp_err_t ret = ESP_OK;
    esp_codec_dev_handle_t handle = es8311_dev_handle ? es8311_dev_handle : play_dev;
    if (!handle && record_dev && es8311_has_adc_path()) {
        handle = record_dev;
    }

    if (!handle) {
        ESP_LOGE(TAG, "ES8311 device handle unavailable, cannot enter sleep");
        return ESP_ERR_INVALID_STATE;
    }

    if (es8311_has_dac_path()) {
        ret = esp_codec_dev_set_out_vol(handle, 0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to mute ES8311 DAC: %s", esp_err_to_name(ret));
        }
    }

    if (es8311_has_adc_path()) {
        int mute_ret = esp_codec_dev_set_in_gain(handle, 0.0f);
        if (mute_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "Failed to reduce ES8311 ADC gain: %s",
                     esp_err_to_name(static_cast<esp_err_t>(mute_ret)));
        }
    }

    ret = esp_codec_set_disable_when_closed(handle, true);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set ES8311 disable when closed: %s", esp_err_to_name(ret));
    }

    ret = esp_codec_dev_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to close ES8311: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "ES8311 device closed for sleep");
    }

    es8311_sleeping = true;
    
    ESP_LOGI(TAG, "ES8311 sleep mode enabled");
    return ESP_OK;
}

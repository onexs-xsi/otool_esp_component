/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_tools.h"
#include "audio_sr_afe.h"
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
#include <cctype>

static const char *TAG = "audio_tools";

// I2S DMA buffers must live in internal DMA-capable RAM on ESP32-P4. Keep the
// ring deliberately bounded so audio can be recreated after Factory P1
// restores the shared SYS I2C/M5BUS resources. Preserve the original 256-frame
// cadence used by simultaneous P5 playback/capture, while reducing the ring
// from six to three descriptors (6 KiB per direction instead of 12 KiB at
// 16 kHz, stereo, 32-bit).
static constexpr uint32_t AUDIO_I2S_DMA_DESC_NUM = 3;
static constexpr uint32_t AUDIO_I2S_DMA_FRAME_NUM = 256;

// PCM extern declarations, AudioDataGetter functions, AUDIO_FILE_TABLE,
// find_audio_metadata, load_int16_le, load_uint32_le -> moved to audio_playback.cpp / audio_recorder.cpp
// free_channel_split_result, split_recorded_channels, compute_split_channel_quality,
// ensure_parent_directories -> moved to audio_recorder.cpp

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
audio_tools::audio_tools()
{
    ESP_LOGI(TAG, "audio_tools object created with default parameters");
    play_dev = NULL;
    record_dev = NULL;
    es8311_dev_handle = NULL;
    es8311_work_mode = ESP_CODEC_DEV_WORK_MODE_NONE;
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
    
    // 子对象稍后按需创建
    sr_afe_ = nullptr;
    playback_ = nullptr;
    recorder_ = nullptr;

    // 创建系统互斥锁
    audio_mutex = xSemaphoreCreateMutex();
    if (!audio_mutex) {
        ESP_LOGE(TAG, "Failed to create audio mutex");
    }
}

// 构造函数（带参数）
audio_tools::audio_tools(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                               gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin)
{
    ESP_LOGI(TAG, "audio_tools object created with custom parameters");
    play_dev = NULL;
    record_dev = NULL;
    es8311_dev_handle = NULL;
    es8311_work_mode = ESP_CODEC_DEV_WORK_MODE_NONE;
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
    
    // 子对象稍后按需创建
    sr_afe_ = nullptr;
    playback_ = nullptr;
    recorder_ = nullptr;

    // 创建系统互斥锁
    audio_mutex = xSemaphoreCreateMutex();
    if (!audio_mutex) {
        ESP_LOGE(TAG, "Failed to create audio mutex");
    }
}

// 析构函数
audio_tools::~audio_tools()
{
    ESP_LOGI(TAG, "audio_tools object destroyed");

    cancel_pending_pa_enable();
    
    // 先清理子对象
    if (playback_) {
        delete playback_;
        playback_ = nullptr;
    }
    if (recorder_) {
        delete recorder_;
        recorder_ = nullptr;
    }
    if (sr_afe_) {
        delete sr_afe_;
        sr_afe_ = nullptr;
    }
    
    // 清理音频资源
    audio_system_deinit();

    // 最后释放系统互斥锁
    if (audio_mutex) {
        vSemaphoreDelete(audio_mutex);
        audio_mutex = nullptr;
    }
}

// 内部辅助：确保 I2S 通道存在
esp_err_t audio_tools::ensure_i2s_channel()
{
    if (tx_handle && rx_handle) return ESP_OK;
    return i2s_channel_init();
}

void audio_tools::incr_i2s_user()
{
    i2s_user_count++;
    ESP_LOGD(TAG, "I2S user ++ => %d", i2s_user_count);
}

void audio_tools::decr_i2s_user()
{
    if (i2s_user_count > 0) {
        i2s_user_count--;
        ESP_LOGD(TAG, "I2S user -- => %d", i2s_user_count);
    }
    try_release_i2s();
}

void audio_tools::try_release_i2s()
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

esp_err_t audio_tools::i2s_channel_init()
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
    chan_cfg.dma_desc_num = AUDIO_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = AUDIO_I2S_DMA_FRAME_NUM;
    ret = i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channels: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S channels created successfully (channel config will be set individually)");
    return ESP_OK;
}

esp_err_t audio_tools::i2s_channel_deinit()
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

esp_err_t audio_tools::i2s_tx_init()
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

esp_err_t audio_tools::i2s_tx_deinit()
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

esp_err_t audio_tools::i2s_rx_init()
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

esp_err_t audio_tools::i2s_rx_deinit()
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

esp_err_t audio_tools::audio_system_init(i2c_master_bus_handle_t i2c_bus_handle, i2s_port_t i2s_port_num, audio_sample_rate_t sample_rate, i2s_data_bit_width_t bits_per_sample)
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

esp_err_t audio_tools::audio_system_deinit(bool for_deep_sleep)
{
    cancel_pending_pa_enable();

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

    ESP_LOGI(TAG, "Deinitializing audio system (for_deep_sleep=%s)...",
             for_deep_sleep ? "YES" : "NO");

    // 在释放任何 codec/I2S 资源前，先停掉所有后台任务，避免它们在
    // codec/I2S 资源释放途中继续访问 play_dev/record_dev/DMA 缓冲区，
    // 进而破坏堆元数据。
    if (sr_afe_ && sr_afe_->aec_session_is_running()) {
        ESP_LOGI(TAG, "Stopping running AEC session before deinit");
        esp_err_t sr_ret = sr_afe_->aec_session_stop();
        if (sr_ret != ESP_OK) {
            ESP_LOGW(TAG, "AEC session stop reported: %s", esp_err_to_name(sr_ret));
        }
    }
    if (recorder_ && recorder_->record_session_is_running()) {
        ESP_LOGI(TAG, "Stopping running record session before deinit");
        esp_err_t rec_ret = recorder_->record_session_stop();
        if (rec_ret != ESP_OK) {
            ESP_LOGW(TAG, "Record session stop reported: %s", esp_err_to_name(rec_ret));
        }
    }
    if (playback_ && playback_->is_async_playback_running()) {
        ESP_LOGI(TAG, "Stopping running async playback before deinit");
        esp_err_t pb_ret = playback_->stop_async_playback();
        if (pb_ret != ESP_OK) {
            ESP_LOGW(TAG, "Async playback stop reported: %s", esp_err_to_name(pb_ret));
        }
    }

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

    if (for_deep_sleep) {
        // 深度睡眠路径：仅 disable I2S 通道，不调用 i2s_del_channel。
        // 原因：ESP32-P4 进入 deep sleep 时整个 HP 域断电，DMA 描述符与
        // 内部 SRAM 都会被复位；显式 free() DMA 描述符曾触发 TLSF 堆元
        // 数据断言（i2s_free_dma_desc 中 free() 失败）。
        if (tx_handle && tx_configured) {
            esp_err_t r = i2s_channel_disable(tx_handle);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Disable TX before sleep failed: %s", esp_err_to_name(r));
            }
            tx_configured = false;
        }
        if (rx_handle && rx_configured) {
            esp_err_t r = i2s_channel_disable(rx_handle);
            if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Disable RX before sleep failed: %s", esp_err_to_name(r));
            }
            rx_configured = false;
        }
        // 故意保留 tx_handle/rx_handle，不调用 i2s_del_channel/i2s_free_dma_desc。
        // 引用计数清零但跳过通道释放。
        i2s_user_count = 0;
        suppress_release = false;
        ESP_LOGI(TAG,
                 "Deep-sleep path: skipping i2s_del_channel (HP domain power-off will reset DMA)");
    } else {
        // 常规释放路径
        suppress_release = false;
        try_release_i2s();
    }

    system_initialized = false;

    ESP_LOGI(TAG, "Audio system deinitialized successfully");

    // 释放互斥锁
    if (audio_mutex) xSemaphoreGive(audio_mutex);
    return ESP_OK;
}

// ES8311和ES7210的睡眠功能实现已移至对应的独立源文件中

esp_err_t audio_tools::audio_system_sleep()
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

// record_test, record_and_play_test, record_and_play_test_with_channel_select,
// record_all_channel_to_files, record_and_playback_test -> moved to audio_recorder.cpp

esp_codec_dev_handle_t audio_tools::get_play_device_handle() const
{
    return play_dev;
}

esp_codec_dev_handle_t audio_tools::get_record_device_handle() const
{
    return record_dev;
}

bool audio_tools::is_es8311_initialized() const
{
    return es8311_initialized;
}

bool audio_tools::is_es7210_initialized() const
{
    return es7210_initialized;
}

bool audio_tools::is_system_initialized() const
{
    return system_initialized;
}

bool audio_tools::is_es8311_sleeping() const
{
    return es8311_sleeping;
}

bool audio_tools::is_es7210_sleeping() const
{
    return es7210_sleeping;
}

void audio_tools::set_i2s_pin_config(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
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

void audio_tools::set_pa_power_callback(PaPowerCallback callback, void *arg)
{
    pa_power_callback = callback;
    pa_power_callback_arg = arg;
    ESP_LOGI(TAG, "Audio PA power callback %s", callback ? "registered" : "cleared");
}

esp_err_t audio_tools::set_pa_power(bool on)
{
    if (pa_power_callback == nullptr) {
        ESP_LOGE(TAG, "Audio PA power callback is not registered");
        return ESP_ERR_INVALID_STATE;
    }

    return pa_power_callback(pa_power_callback_arg, on);
}

void audio_tools::delayed_pa_enable_task_entry(void *arg)
{
    auto *audio = static_cast<audio_tools *>(arg);
    if (audio == nullptr) {
        ESP_LOGE(TAG, "Audio PA enable task got null audio_tools");
        vTaskDelete(nullptr);
        return;
    }

    const uint32_t delay_ms = audio->pa_enable_delay_ms;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    esp_err_t ret = audio->set_pa_power(true);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Audio PA enabled after %lu ms delay",
                 static_cast<unsigned long>(delay_ms));
    } else {
        ESP_LOGE(TAG, "Failed to enable Audio PA: %s", esp_err_to_name(ret));
    }

    audio->pa_enable_task_handle = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t audio_tools::enable_pa_after_delay(uint32_t delay_ms)
{
    if (pa_power_callback == nullptr) {
        ESP_LOGE(TAG, "Audio PA power callback is not registered");
        return ESP_ERR_INVALID_STATE;
    }

    cancel_pending_pa_enable();

    if (delay_ms == 0) {
        return set_pa_power(true);
    }

    pa_enable_delay_ms = delay_ms;
    BaseType_t task_ret = xTaskCreate(delayed_pa_enable_task_entry,
                                      "audio_pa_on",
                                      2048,
                                      this,
                                      5,
                                      &pa_enable_task_handle);
    if (task_ret != pdPASS) {
        pa_enable_task_handle = nullptr;
        ESP_LOGE(TAG, "Failed to create Audio PA enable task; enabling immediately");
        esp_err_t ret = set_pa_power(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable Audio PA immediately: %s", esp_err_to_name(ret));
            return ret;
        }
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void audio_tools::cancel_pending_pa_enable()
{
    if (pa_enable_task_handle == nullptr) {
        return;
    }

    TaskHandle_t task = pa_enable_task_handle;
    pa_enable_task_handle = nullptr;
    vTaskDelete(task);
    ESP_LOGI(TAG, "Cancelled pending Audio PA enable task");
}

// get_available_pcm_count, get_audio_file_name, is_audio_file_available -> inline forwarding in audio_tools.h
// get_pcm_data_and_format, play_audio_file_impl, play_audio_buffer_impl -> moved to audio_playback.cpp
// play_audio_file, play_audio_buffer, playback_task_entry, buffer_playback_task_entry -> moved to audio_playback.cpp
// stop_async_playback, clear_audio_pipeline -> moved to audio_playback.cpp

esp_err_t audio_tools::set_volume(float volume_value)
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

esp_err_t audio_tools::set_record_gain(float gain)
{
    // 检查增益范围（ES7210支持0-66dB）
    if (gain < 0.0 || gain > 66.0) {
        ESP_LOGE(TAG, "Invalid record gain %.1f dB, must be between 0.0 and 66.0", gain);
        return ESP_ERR_INVALID_ARG;
    }
    
    record_gain = gain;
    ESP_LOGI(TAG, "Record gain set to %.1f dB", gain);
    
    const bool es8311_adc_ready = es8311_initialized && es8311_has_adc_path() && (record_dev == es8311_dev_handle);
    const bool es7210_adc_ready = es7210_initialized;

    if (record_dev && (es7210_adc_ready || es8311_adc_ready)) {
        int ret = esp_codec_dev_set_in_gain(record_dev, gain);
        if (ret != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "Failed to apply record gain %.1f dB: %s", gain,
                     esp_err_to_name(static_cast<esp_err_t>(ret)));
            return static_cast<esp_err_t>(ret);
        }
        ESP_LOGI(TAG, "Applied record gain %.1f dB to %s", gain,
                 es7210_adc_ready ? "ES7210" : "ES8311");
    }
    
    return ESP_OK;
}

esp_err_t audio_tools::set_audio_levels(float volume, float gain)
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

esp_err_t audio_tools::es7210_set_mic_channel_gain(audio_mic_channel_t mic_channels_to_set, es7210_mic_gain_t gain)
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

const char* audio_tools::get_mic_channels_description(audio_mic_channel_t channels) const
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

bool audio_tools::is_mic_channels_valid(audio_mic_channel_t channels) const
{
    // 检查通道值是否在有效范围内（0x00-0x0F）
    if (channels > AUDIO_MIC_CHANNEL_ALL) {
        return false;
    }
    
    // AUDIO_MIC_NONE (0x00) 是有效的，表示不使用任何麦克风
    return true;
}

int audio_tools::count_selected_mics() const
{
    uint8_t v = static_cast<uint8_t>(mic_channels);
    int cnt = 0;
    while (v) {
        cnt += (v & 0x1);
        v >>= 1;
    }
    return cnt;
}

// record_to_file -> moved to audio_recorder.cpp

// ========== Sub-object lazy creation ==========

audio_sr_afe* audio_tools::get_sr_afe()
{
    if (!sr_afe_) {
        sr_afe_ = new (std::nothrow) audio_sr_afe(this);
        if (!sr_afe_) {
            ESP_LOGE(TAG, "Failed to allocate audio_sr_afe object");
            return nullptr;
        }
        ESP_LOGI(TAG, "audio_sr_afe object created");
    }
    return sr_afe_;
}

audio_playback* audio_tools::get_playback()
{
    if (!playback_) {
        playback_ = new (std::nothrow) audio_playback(this);
        if (!playback_) {
            ESP_LOGE(TAG, "Failed to allocate audio_playback object");
            return nullptr;
        }
        ESP_LOGI(TAG, "audio_playback object created");
    }
    return playback_;
}

audio_recorder* audio_tools::get_recorder()
{
    if (!recorder_) {
        recorder_ = new (std::nothrow) audio_recorder(this);
        if (!recorder_) {
            ESP_LOGE(TAG, "Failed to allocate audio_recorder object");
            return nullptr;
        }
        ESP_LOGI(TAG, "audio_recorder object created");
    }
    return recorder_;
}

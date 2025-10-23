/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file aec_adaptive_sample_rate_example.cpp
 * @brief AEC自适应采样率使用示例
 * 
 * 本文件展示如何在不同采样率下使用AEC（回声消除）功能
 */

#include "audio_es_tools.h"
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "aec_example";

/**
 * @brief 示例1: 使用16kHz采样率（无重采样，最佳性能）
 */
void example_aec_16khz(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "=== Example 1: AEC at 16kHz (No Resampling) ===");
    
    audio_es_tools audio;
    
    // 初始化音频系统为16kHz（AEC原生采样率）
    esp_err_t ret = audio.audio_system_init(
        i2c_bus,
        I2S_NUM_0,
        AUDIO_SAMPLE_RATE_16K,  // 16kHz，无需重采样
        I2S_DATA_BIT_WIDTH_16BIT
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio system");
        return;
    }
    
    // 初始化DAC和ADC
    audio.es8311_init(AUDIO_CHANNELS_MONO);
    audio.es7210_init(AUDIO_CHANNELS_MONO, AUDIO_MIC_CHANNEL_1, ES7210_TDM_DISABLED);
    
    // 运行AEC测试（无重采样开销）
    // 参数：录音5秒，滤波器长度4，低功耗模式，播放处理后的音频
    ret = audio.aec_test(5, 4, AFE_MODE_LOW_COST, false);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AEC test completed successfully");
    } else {
        ESP_LOGE(TAG, "AEC test failed: %s", esp_err_to_name(ret));
    }
    
    // 清理
    audio.audio_system_deinit();
}

/**
 * @brief 示例2: 使用44.1kHz采样率（自动重采样）
 */
void example_aec_44khz(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "=== Example 2: AEC at 44.1kHz (Auto Resampling) ===");
    
    audio_es_tools audio;
    
    // 初始化音频系统为44.1kHz（CD质量）
    esp_err_t ret = audio.audio_system_init(
        i2c_bus,
        I2S_NUM_0,
        AUDIO_SAMPLE_RATE_44K1,  // 44.1kHz，自动重采样
        I2S_DATA_BIT_WIDTH_16BIT
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio system");
        return;
    }
    
    // 初始化DAC和ADC
    audio.es8311_init(AUDIO_CHANNELS_MONO);
    audio.es7210_init(AUDIO_CHANNELS_MONO, AUDIO_MIC_CHANNEL_1, ES7210_TDM_DISABLED);
    
    // 运行AEC测试（自动重采样：44.1kHz ↔ 16kHz）
    // 参数：录音5秒，滤波器长度4，高性能模式，播放处理后的音频
    ret = audio.aec_test(5, 4, AFE_MODE_HIGH_PERF, false);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AEC test with resampling completed successfully");
    } else {
        ESP_LOGE(TAG, "AEC test failed: %s", esp_err_to_name(ret));
    }
    
    // 清理
    audio.audio_system_deinit();
}

/**
 * @brief 示例3: 使用48kHz采样率（专业音频）
 */
void example_aec_48khz(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "=== Example 3: AEC at 48kHz (Professional Audio) ===");
    
    audio_es_tools audio;
    
    // 初始化音频系统为48kHz（专业音频标准）
    esp_err_t ret = audio.audio_system_init(
        i2c_bus,
        I2S_NUM_0,
        AUDIO_SAMPLE_RATE_48K,  // 48kHz专业音频
        I2S_DATA_BIT_WIDTH_16BIT
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio system");
        return;
    }
    
    // 初始化DAC和ADC
    audio.es8311_init(AUDIO_CHANNELS_MONO);
    audio.es7210_init(AUDIO_CHANNELS_MONO, AUDIO_MIC_CHANNEL_1, ES7210_TDM_DISABLED);
    
    // 设置音量和增益
    audio.set_volume(80.0f);  // 80% 音量
    audio.set_record_gain(30.0f);  // 30dB 增益
    
    // 运行AEC测试（自动重采样：48kHz ↔ 16kHz）
    // 对比原始录音和处理后的音频
    ESP_LOGI(TAG, "Playing original audio for comparison...");
    audio.aec_test(3, 4, AFE_MODE_HIGH_PERF, true);  // 播放原始录音
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒
    
    ESP_LOGI(TAG, "Playing AEC-processed audio...");
    audio.aec_test(3, 4, AFE_MODE_HIGH_PERF, false);  // 播放AEC处理后的音频
    
    // 清理
    audio.audio_system_deinit();
}

/**
 * @brief 示例4: 动态切换采样率
 */
void example_dynamic_sample_rate(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "=== Example 4: Dynamic Sample Rate Switching ===");
    
    audio_es_tools audio;
    
    // 测试不同采样率
    audio_sample_rate_t test_rates[] = {
        AUDIO_SAMPLE_RATE_16K,
        AUDIO_SAMPLE_RATE_22K,
        AUDIO_SAMPLE_RATE_32K,
        AUDIO_SAMPLE_RATE_44K1,
        AUDIO_SAMPLE_RATE_48K
    };
    
    const char* rate_names[] = {
        "16kHz",
        "22.05kHz", 
        "32kHz",
        "44.1kHz",
        "48kHz"
    };
    
    for (int i = 0; i < sizeof(test_rates) / sizeof(test_rates[0]); i++) {
        ESP_LOGI(TAG, "--- Testing at %s ---", rate_names[i]);
        
        // 初始化音频系统
        esp_err_t ret = audio.audio_system_init(
            i2c_bus,
            I2S_NUM_0,
            test_rates[i],
            I2S_DATA_BIT_WIDTH_16BIT
        );
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize audio system at %s", rate_names[i]);
            continue;
        }
        
        // 初始化DAC和ADC
        audio.es8311_init(AUDIO_CHANNELS_MONO);
        audio.es7210_init(AUDIO_CHANNELS_MONO, AUDIO_MIC_CHANNEL_1, ES7210_TDM_DISABLED);
        
        // 运行快速测试（2秒）
        ret = audio.aec_test(2, 4, AFE_MODE_LOW_COST, false);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AEC test at %s completed successfully", rate_names[i]);
        } else {
            ESP_LOGE(TAG, "AEC test at %s failed", rate_names[i]);
        }
        
        // 清理并准备下一次测试
        audio.audio_system_deinit();
        vTaskDelay(pdMS_TO_TICKS(500));  // 短暂延迟
    }
}

/**
 * @brief 示例5: 长时间录音测试（内存管理）
 */
void example_long_recording(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "=== Example 5: Long Recording Test (Memory Management) ===");
    
    audio_es_tools audio;
    
    // 初始化音频系统为16kHz(最省内存)
    esp_err_t ret = audio.audio_system_init(
        i2c_bus,
        I2S_NUM_0,
        AUDIO_SAMPLE_RATE_16K,  // 使用16kHz节省内存
        I2S_DATA_BIT_WIDTH_16BIT
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio system");
        return;
    }
    
    // 初始化DAC和ADC
    audio.es8311_init(AUDIO_CHANNELS_MONO);
    audio.es7210_init(AUDIO_CHANNELS_MONO, AUDIO_MIC_CHANNEL_1, ES7210_TDM_DISABLED);
    
    // 检查可用内存
    ESP_LOGI(TAG, "Free SPIRAM before test: %d bytes", 
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // 运行10秒录音测试
    ret = audio.aec_test(10, 4, AFE_MODE_LOW_COST, false);
    
    // 再次检查内存（应该已释放）
    ESP_LOGI(TAG, "Free SPIRAM after test: %d bytes", 
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Long recording test completed successfully");
    } else {
        ESP_LOGE(TAG, "Long recording test failed: %s", esp_err_to_name(ret));
    }
    
    // 清理
    audio.audio_system_deinit();
}

/**
 * @brief 主测试函数
 */
void run_aec_examples(i2c_master_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "Starting AEC Adaptive Sample Rate Examples...");
    
    // 运行不同示例
    // example_aec_16khz(i2c_bus);        // 示例1: 16kHz无重采样
    // example_aec_44khz(i2c_bus);        // 示例2: 44.1kHz自动重采样
    // example_aec_48khz(i2c_bus);        // 示例3: 48kHz专业音频
    // example_dynamic_sample_rate(i2c_bus);  // 示例4: 动态切换采样率
    // example_long_recording(i2c_bus);   // 示例5: 长时间录音
    
    // 取消注释你想运行的示例
    example_aec_16khz(i2c_bus);
    
    ESP_LOGI(TAG, "All examples completed");
}

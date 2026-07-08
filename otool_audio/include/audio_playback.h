/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_PLAYBACK_H__
#define __AUDIO_PLAYBACK_H__

#include "audio_types.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdint.h>
#include <stddef.h>

// 前置声明，避免循环依赖
class audio_tools;

/**
 * @brief 音频播放子对象
 *
 * 封装所有播放相关功能，作为 audio_tools 的子对象存在。
 * 通过 parent_ 指针访问核心硬件句柄（play_dev、sample_rate 等）。
 *
 * 使用方式：
 *   audio.get_playback()->play_audio_file(AUDIO_FILE_STARTUP_2CH_16K_16B_4S);
 *   audio.get_playback()->play_audio_buffer(buf, len, 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT);
 */
class audio_playback {
private:
    audio_tools* parent_;                                      ///< 父对象指针
    TaskHandle_t playback_task_handle_ = nullptr;              ///< 异步播放任务句柄
    volatile bool playback_progress_valid_ = false;
    volatile uint32_t playback_elapsed_ms_ = 0;
    volatile uint32_t playback_duration_ms_ = 0;


    struct playback_task_args {
        audio_playback* instance;
        audio_file_type_t audio_type;
        float duration_limit_seconds;
    };

    struct buffer_playback_task_args {
        audio_playback* instance;
        const uint8_t* buffer;
        size_t buffer_size;
        uint32_t buffer_sample_rate_hz;
        audio_channels_t buffer_channels;
        i2s_data_bit_width_t buffer_bits;
        float duration_limit_seconds;
        bool own_buffer;
    };

    esp_err_t play_audio_file_impl(audio_file_type_t audio_type, bool check_stop_signal, float duration_limit_seconds);
    esp_err_t play_audio_buffer_impl(const uint8_t* buffer, size_t buffer_size,
                                     uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels,
                                     i2s_data_bit_width_t buffer_bits,
                                     bool check_stop_signal, float duration_limit_seconds);
    static void playback_task_entry(void* param);
    static void buffer_playback_task_entry(void* param);
    esp_err_t get_pcm_data_and_format(audio_file_type_t audio_type,
                                      const uint8_t*& pcm_start,
                                      size_t& pcm_len,
                                      uint32_t& file_sample_rate_hz,
                                      audio_channels_t& file_channels,
                                      i2s_data_bit_width_t& file_bits);

public:
    explicit audio_playback(audio_tools* parent);
    ~audio_playback();

    // ===== 播放 =====

    /**
     * @brief 播放指定类型的音频文件
     */
    esp_err_t play_audio_file(audio_file_type_t audio_type,
                              audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING,
                              float duration_limit_seconds = 0.0f);

    /**
     * @brief 播放内存中的音频缓冲区，支持自适应格式转换
     */
    esp_err_t play_audio_buffer(const uint8_t* buffer, size_t buffer_size,
                                uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels,
                                i2s_data_bit_width_t buffer_bits,
                                audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING,
                                float duration_limit_seconds = 0.0f);

    /**
     * @brief 查询是否存在正在运行的异步播放任务
     */
    bool is_async_playback_running() const { return playback_task_handle_ != nullptr; }

    /**
     * @brief Get playback progress measured from audio written to the codec.
     */
    bool get_playback_progress(uint32_t& elapsed_ms, uint32_t& duration_ms) const;

    /**
     * @brief 停止正在运行的异步播放任务
     */
    esp_err_t stop_async_playback();

    /**
     * @brief 清理音频播放管道，发送静音数据清除残留音频
     */
    esp_err_t clear_audio_pipeline(uint32_t silence_duration_ms = 100);

    // ===== 音频文件信息 =====

    /**
     * @brief 获取可用PCM测试文件的数量
     */
    int get_available_pcm_count() const;

    /**
     * @brief 获取音频文件的名称字符串
     */
    const char* get_audio_file_name(audio_file_type_t audio_type) const;

    esp_err_t get_audio_file_pcm(audio_file_type_t audio_type,
                                 const uint8_t*& pcm_start,
                                 size_t& pcm_len,
                                 uint32_t& file_sample_rate_hz,
                                 audio_channels_t& file_channels,
                                 i2s_data_bit_width_t& file_bits);

    /**
     * @brief 检查指定音频文件是否可用
     */
    bool is_audio_file_available(audio_file_type_t audio_type) const;
};

#endif // __AUDIO_PLAYBACK_H__

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_RECORDER_H__
#define __AUDIO_RECORDER_H__

#include "audio_types.h"
#include "esp_codec_dev.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include <stdint.h>
#include <stddef.h>

// 前置声明，避免循环依赖
class audio_tools;

/**
 * @brief 录音缓冲拆分结果
 *
 * 为最多4路麦克风通道提供独立的样本缓冲和基础元数据。
 * status 非 ESP_OK 时，其余字段可能为空，应先检查再使用。
 */
typedef struct {
    esp_err_t status;                          ///< 拆分操作结果
    size_t samples_per_channel;                ///< 每个通道的样本数
    size_t bytes_per_sample;                   ///< 原始I2S样本的字节数
    bool is_tdm_mode;                          ///< 是否基于TDM模式拆分
    audio_mic_channel_t enabled_mask;          ///< 参与拆分的麦克风掩码
    int16_t* mic_buffers[4];                   ///< 各通道指向16bit单声道缓冲的指针（未启用时为nullptr）
} channel_split_result_t;

typedef struct {
    bool available;                            ///< 通道是否有有效缓冲
    size_t sample_count;                       ///< 通道样本数量
    int16_t min_value;                         ///< 通道最小采样值
    int16_t max_value;                         ///< 通道最大采样值
    int32_t average_abs_amplitude;             ///< 平均绝对幅度
    double rms_db;                             ///< RMS 电平 (dB)
    double zero_percent;                       ///< 零值占比 (%)
    double clipped_percent;                    ///< 剪裁占比 (%)
} mic_channel_quality_t;

/**
 * @brief 录音流式会话配置
 */
struct record_session_config_t {
    audio_mic_channel_t target_channel = AUDIO_MIC_CHANNEL_1;
    size_t output_ringbuf_size = 64 * 1024;
    UBaseType_t task_priority = configMAX_PRIORITIES - 3;
    uint32_t task_stack_size = 4096;
};

/**
 * @brief 音频录音子对象
 *
 * 封装所有录音相关功能，作为 audio_tools 的子对象存在。
 * 通过 parent_ 指针访问核心硬件句柄（record_dev、sample_rate 等）。
 *
 * 使用方式：
 *   audio.get_recorder()->record_to_file("/sdcard/test.pcm", 5);
 *   audio.get_recorder()->record_session_start();
 *   int n = audio.get_recorder()->record_session_read(buf, len, 100);
 *   audio.get_recorder()->record_session_stop();
 */
class audio_recorder {
private:
    audio_tools* parent_;                                      ///< 父对象指针

    // ===== 流式录音会话状态 =====
    TaskHandle_t record_session_task_handle_ = nullptr;
    RingbufHandle_t record_session_ringbuf_ = nullptr;
    volatile bool record_session_stop_flag_ = false;
    record_session_config_t record_session_config_ = {};

    static void record_stream_task(void* param);

public:
    explicit audio_recorder(audio_tools* parent);
    ~audio_recorder();

    // ===== 流式录音会话 (新增 API) =====

    /**
     * @brief 启动流式录音会话
     */
    esp_err_t record_session_start(const record_session_config_t& config = {});

    /**
     * @brief 从流式录音会话读取数据（16-bit mono PCM）
     */
    int record_session_read(void* buf, size_t len, uint32_t timeout_ms = 100);

    /**
     * @brief 停止流式录音会话
     */
    esp_err_t record_session_stop();

    /**
     * @brief 查询流式录音会话是否正在运行
     */
    bool record_session_is_running() const;

    // ===== 批处理录音 =====

    /**
     * @brief 将录音数据保存到指定文件
     */
    esp_err_t record_to_file(const char* filepath, uint32_t record_duration_seconds, size_t chunk_size = 4096);

    /**
     * @brief 录制所有启用的麦克风通道并分别保存为 PCM 文件
     */
    esp_err_t record_all_channel_to_files(uint32_t record_duration_seconds,
                                          const char* output_directory,
                                          const char* file_prefix = nullptr);

    // ===== 通道处理工具 (static) =====

    /**
     * @brief 将录音缓冲区拆分为独立的麦克风通道
     */
    static channel_split_result_t split_recorded_channels(const uint8_t* record_buffer,
                                                          size_t bytes_read,
                                                          const esp_codec_dev_sample_info_t& fs,
                                                          bool is_tdm_mode,
                                                          audio_mic_channel_t mic_channels);

    /**
     * @brief 释放通道拆分结果中分配的缓冲区
     */
    static void free_channel_split_result(channel_split_result_t& result);

    /**
     * @brief 计算拆分后四个通道的质量指标
     */
    static void compute_split_channel_quality(const channel_split_result_t& split_result,
                                              mic_channel_quality_t quality[4]);

    // ===== 废弃测试函数 (代码保留，标注 deprecated) =====

    [[deprecated("Use test_record_quality() from audio_test_utils.h")]]
    esp_err_t record_test(uint32_t record_duration_ms = 3000);

    [[deprecated("Use test_record_and_play() from audio_test_utils.h")]]
    esp_err_t record_and_play_test(uint32_t record_duration_seconds = 3);

    [[deprecated("Use test_record_channel_select() from audio_test_utils.h")]]
    esp_err_t record_and_play_test_with_channel_select(uint32_t record_duration_seconds = 3,
                                                        audio_mic_channel_t target_mic_channel = AUDIO_MIC_CHANNEL_1,
                                                        bool analysis_only = false);

    [[deprecated("Use test_record_and_playback_loop() from audio_test_utils.h")]]
    esp_err_t record_and_playback_test(uint32_t record_duration_seconds = 5,
                                       bool loop_playback = false,
                                       audio_mic_channel_t target_mic_channel = AUDIO_MIC_NONE);
};

#endif // __AUDIO_RECORDER_H__

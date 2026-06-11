/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_TYPES_H__
#define __AUDIO_TYPES_H__

/**
 * @file audio_types.h
 * @brief 音频系统通用类型定义
 * 
 * 此文件包含音频系统中使用的通用枚举和类型定义，
 * 避免在各个设备头文件中重复定义。
 */

/**
 * @brief 音频声道枚举
 * 
 * 定义音频声道配置
 */
typedef enum {
    AUDIO_CHANNELS_MONO = 1,     ///< 单声道
    AUDIO_CHANNELS_STEREO = 2,   ///< 立体声
    AUDIO_CHANNELS_3CHs = 3,     ///< 3声道
    AUDIO_CHANNELS_4CHs = 4      ///< 4声道
} audio_channels_t;

/**
 * @brief 麦克风通道选择枚举
 * 
 * 定义音频系统的麦克风输入通道配置
 * 可以组合使用多个通道（使用位或操作）
 */
typedef enum {
    AUDIO_MIC_NONE = 0x00,                    ///< 不选择任何麦克风
    AUDIO_MIC_CHANNEL_1 = 0x01,               ///< 麦克风通道1 (MIC1)
    AUDIO_MIC_CHANNEL_2 = 0x02,               ///< 麦克风通道2 (MIC2) 
    AUDIO_MIC_CHANNEL_3 = 0x04,               ///< 麦克风通道3 (MIC3)
    AUDIO_MIC_CHANNEL_4 = 0x08,               ///< 麦克风通道4 (MIC4)
    AUDIO_MIC_CHANNEL_12 = 0x03,              ///< 麦克风通道1+2
    AUDIO_MIC_CHANNEL_13 = 0x05,              ///< 麦克风通道1+3
    AUDIO_MIC_CHANNEL_14 = 0x09,              ///< 麦克风通道1+4
    AUDIO_MIC_CHANNEL_23 = 0x06,              ///< 麦克风通道2+3
    AUDIO_MIC_CHANNEL_24 = 0x0A,              ///< 麦克风通道2+4
    AUDIO_MIC_CHANNEL_34 = 0x0C,              ///< 麦克风通道3+4
    AUDIO_MIC_CHANNEL_123 = 0x07,             ///< 麦克风通道1+2+3
    AUDIO_MIC_CHANNEL_124 = 0x0B,             ///< 麦克风通道1+2+4
    AUDIO_MIC_CHANNEL_134 = 0x0D,             ///< 麦克风通道1+3+4
    AUDIO_MIC_CHANNEL_234 = 0x0E,             ///< 麦克风通道2+3+4
    AUDIO_MIC_CHANNEL_ALL = 0x0F              ///< 所有麦克风通道1+2+3+4
} audio_mic_channel_t;

/**
 * @brief 音频文件枚举
 * 
 * 定义可播放的音频文件类型
 */
typedef enum {
    AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S = 0,  ///< candy_wind 1通道 16000Hz 16bit 9.0秒 (candy_wind_pcm_1ch_16k_16bit_9s.pcm)
    AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S,     ///< candy_wind 1通道 44100Hz 16bit 45.0秒 (candy_wind_pcm_1ch_44.1k_16bit_45s.pcm)
    AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S,     ///< candy_wind 2通道 16000Hz 16bit 9.0秒 (candy_wind_pcm_2ch_16k_16bit_9s.pcm)
    AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S,     ///< candy_wind 2通道 44100Hz 16bit 45.0秒 (candy_wind_pcm_2ch_44.1k_16bit_45s.pcm)
    AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S,     ///< sine_440Hz 2通道 16000Hz 16bit 10.0秒 (sine_440Hz_pcm_2ch_16k_16bit_10s.pcm)
    AUDIO_FILE_STARTUP_1CH_16K_16B_4S,     ///< startup 1通道 16000Hz 16bit 4.0秒 (startup_pcm_1ch_16k_16bit_4s.pcm)
    AUDIO_FILE_STARTUP_2CH_16K_16B_4S,     ///< startup 2通道 16000Hz 16bit 4.0秒 (startup_pcm_2ch_16k_16bit_4s.pcm)
    AUDIO_FILE_MAX                             ///< 枚举最大值（用于边界检查）
} audio_file_type_t;

/**
 * @brief 音频播放模式
 *
 * 决定播放接口是阻塞执行还是创建后台任务异步播放
 */
typedef enum {
    AUDIO_PLAYBACK_BLOCKING = 0, ///< 阻塞播放，函数调用完成后表示播放已结束
    AUDIO_PLAYBACK_ASYNC         ///< 异步播放，立即返回并在后台任务中完成播放
} audio_playback_mode_t;

/**
 * @brief 音频采样率枚举
 * 
 * 定义常用的音频采样率
 */
typedef enum {
    AUDIO_SAMPLE_RATE_8K = 8000,      ///< 8kHz - 电话质量
    AUDIO_SAMPLE_RATE_16K = 16000,    ///< 16kHz - 语音通话
    AUDIO_SAMPLE_RATE_22K = 22050,    ///< 22.05kHz - FM广播质量
    AUDIO_SAMPLE_RATE_32K = 32000,    ///< 32kHz - 数字广播
    AUDIO_SAMPLE_RATE_44K1 = 44100,   ///< 44.1kHz - CD质量
    AUDIO_SAMPLE_RATE_48K = 48000,    ///< 48kHz - 专业音频
    AUDIO_SAMPLE_RATE_88K2 = 88200,   ///< 88.2kHz - 高保真
    AUDIO_SAMPLE_RATE_96K = 96000,    ///< 96kHz - 高保真专业
    AUDIO_SAMPLE_RATE_176K4 = 176400, ///< 176.4kHz - 超高保真
    AUDIO_SAMPLE_RATE_192K = 192000   ///< 192kHz - 超高保真专业
} audio_sample_rate_t;

#endif // __AUDIO_TYPES_H__

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

#endif // __AUDIO_TYPES_H__

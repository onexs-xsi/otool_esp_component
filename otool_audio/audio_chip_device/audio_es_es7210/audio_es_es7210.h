/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_ES_ES7210_H__
#define __AUDIO_ES_ES7210_H__

#include "esp_err.h"

/**
 * @brief ES7210麦克风通道选择枚举
 * 
 * 定义ES7210 ADC的麦克风输入通道配置
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
 * @brief ES7210 TDM 模式控制
 */
typedef enum {
    ES7210_TDM_DISABLED = 0,  ///< 禁用TDM，使用标准I2S
    ES7210_TDM_ENABLED = 1    ///< 启用TDM，固定4通道slot
} es7210_tdm_mode_t;

/**
 * @brief ES7210麦克风增益枚举
 * 
 * 定义ES7210 ADC支持的增益档位
 * 寄存器值直接对应档位序号(0-14)
 */
typedef enum {
    ES7210_MIC_GAIN_0DB = 0,        ///< 0dB (寄存器值=0)
    ES7210_MIC_GAIN_3DB = 1,        ///< 3dB (寄存器值=1)
    ES7210_MIC_GAIN_6DB = 2,        ///< 6dB (寄存器值=2)
    ES7210_MIC_GAIN_9DB = 3,        ///< 9dB (寄存器值=3)
    ES7210_MIC_GAIN_12DB = 4,       ///< 12dB (寄存器值=4)
    ES7210_MIC_GAIN_15DB = 5,       ///< 15dB (寄存器值=5)
    ES7210_MIC_GAIN_18DB = 6,       ///< 18dB (寄存器值=6)
    ES7210_MIC_GAIN_21DB = 7,       ///< 21dB (寄存器值=7)
    ES7210_MIC_GAIN_24DB = 8,       ///< 24dB (寄存器值=8)
    ES7210_MIC_GAIN_27DB = 9,       ///< 27dB (寄存器值=9)
    ES7210_MIC_GAIN_30DB = 10,      ///< 30dB (寄存器值=10, 常用值)
    ES7210_MIC_GAIN_33DB = 11,      ///< 33dB (寄存器值=11)
    ES7210_MIC_GAIN_34_5DB = 12,    ///< 34.5dB (寄存器值=12)
    ES7210_MIC_GAIN_36DB = 13,      ///< 36dB (寄存器值=13)
    ES7210_MIC_GAIN_37_5DB = 14     ///< 37.5dB (寄存器值=14, 最大增益)
} es7210_mic_gain_t;

#endif // __AUDIO_ES_ES7210_H__

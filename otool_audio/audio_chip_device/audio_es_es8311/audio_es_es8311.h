/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_ES_ES8311_H__
#define __AUDIO_ES_ES8311_H__

#include "esp_err.h"
#include "driver/i2s_std.h"

/**
 * @brief 音频声道枚举
 * 
 * 定义音频声道配置
 */
typedef enum {
    AUDIO_CHANNELS_MONO = 1,     ///< 单声道
    AUDIO_CHANNELS_STEREO = 2,   ///< 立体声
    AUDIO_CHANNELS_3CHs = 3,      ///< 3声道
    AUDIO_CHANNELS_4CHs = 4       ///< 4声道
} audio_channels_t;

/**
 * @brief ES8311工作路径配置
 *
 * 通过按位组合选择 ES8311 的 ADC/DAC 功能。
 */
typedef enum {
    ES8311_MODE_NONE = 0,                 ///< 不启用任何路径（无效配置）
    ES8311_MODE_DAC = 1 << 0,             ///< 仅启用 DAC（播放）
    ES8311_MODE_ADC = 1 << 1,             ///< 仅启用 ADC（录音）
    ES8311_MODE_DAC_AND_ADC = ES8311_MODE_DAC | ES8311_MODE_ADC ///< 同时启用 ADC 与 DAC
} es8311_path_mode_t;

#endif // __AUDIO_ES_ES8311_H__

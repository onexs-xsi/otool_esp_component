#pragma once

#include <cstddef>
#include <cstdint>
#include "esp_err.h"

/**
 * @brief 音频数据类型枚举
 */
typedef enum {
    AUDIO_TYPE_INT8 = 8,        ///< 8位有符号整数 PCM
    AUDIO_TYPE_INT16 = 16,      ///< 16位有符号整数 PCM (最常用)
    AUDIO_TYPE_INT32 = 32,      ///< 32位有符号整数 PCM
    AUDIO_TYPE_FLOAT32 = 132    ///< 32位浮点数 [-1.0, 1.0] (100 + 32，用于区分INT32)
} audio_data_type_t;

/**
 * @brief 将位深度转换为音频数据类型枚举
 * 
 * @param bits 位深度 (8, 16, 32)
 * @return audio_data_type_t 对应的音频数据类型枚举
 * 
 * @note 如果位深度不支持，默认返回 AUDIO_TYPE_INT16
 */
audio_data_type_t bits_to_audio_data_type(uint32_t bits);

/**
 * @brief 通用音频格式转换函数（支持重采样、声道转换、数据类型转换）
 *
 * 支持的数据类型：
 * - AUDIO_TYPE_INT8: 8位有符号整数 PCM
 * - AUDIO_TYPE_INT16: 16位有符号整数 PCM (最常用)
 * - AUDIO_TYPE_INT32: 32位有符号整数 PCM
 * - AUDIO_TYPE_FLOAT32: 32位浮点数 [-1.0, 1.0]
 *
 * 功能特性：
 * - 自动重采样（线性插值）
 * - 声道转换（单声道↔立体声）
 * - 数据类型转换（int16↔int32↔float32）
 * - 如果输入输出格式完全一致，返回ESP_OK且output_data=nullptr（零拷贝）
 *
 * @param input_data      输入音频数据缓冲区
 * @param input_size      输入数据字节数
 * @param input_rate      输入采样率（Hz）
 * @param input_channels  输入声道数（1=单声道，2=立体声）
 * @param input_type      输入数据类型（AUDIO_TYPE_INT8/INT16/INT32/FLOAT32）
 * @param target_rate     目标采样率（Hz）
 * @param target_channels 目标声道数（1或2）
 * @param target_type     目标数据类型（AUDIO_TYPE_INT8/INT16/INT32/FLOAT32）
 * @param output_data     输出：转换后的缓冲区指针（需要调用 heap_caps_free 释放）
 * @param output_size     输出：转换后数据的字节数
 * 
 * @return esp_err_t 
 *         - ESP_OK: 转换成功（output_data可能为nullptr表示无需转换）
 *         - ESP_ERR_INVALID_ARG: 参数无效
 *         - ESP_ERR_NOT_SUPPORTED: 不支持的数据类型或声道数
 *         - ESP_ERR_NO_MEM: 内存分配失败
 * 
 * @note 输出缓冲区优先使用SPIRAM分配，失败时回退到内部RAM
 * @note 示例：remix_convert_pcm_to_format(data, size, 48000, 2, AUDIO_TYPE_INT16, 
 *                                          16000, 1, AUDIO_TYPE_FLOAT32, &out, &out_size);
 */
esp_err_t remix_convert_pcm_to_format(const uint8_t* input_data,
                                       size_t input_size,
                                       uint32_t input_rate,
                                       uint32_t input_channels,
                                       audio_data_type_t input_type,
                                       uint32_t target_rate,
                                       uint32_t target_channels,
                                       audio_data_type_t target_type,
                                       uint8_t** output_data,
                                       size_t* output_size);

/**
 * @brief 线性插值重采样
 *
 * 将输入音频数据从 input_rate 重采样到 output_rate，支持多声道。
 * 输出缓冲区由函数内部分配（SPIRAM 优先），调用方需用 heap_caps_free() 释放。
 *
 * @param input                      输入 float 音频数据
 * @param input_samples_per_channel  每通道输入样本数
 * @param channel_count              声道数
 * @param input_rate                 输入采样率 (Hz)
 * @param output_rate                输出采样率 (Hz)
 * @param output                     输出: 重采样后缓冲区指针
 * @param output_samples_per_channel 输出: 每通道输出样本数
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t resample_linear(const float* input,
                          size_t input_samples_per_channel,
                          uint32_t channel_count,
                          uint32_t input_rate,
                          uint32_t output_rate,
                          float** output,
                          size_t* output_samples_per_channel);

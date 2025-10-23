#pragma once

#include <cstddef>
#include <cstdint>
#include "esp_err.h"

/**
 * @brief 根据目标格式对PCM音频数据进行重采样、声道转换和位深转换。
 *
 * 如果输入格式与目标格式完全一致，则函数返回ESP_OK并将output_data设置为nullptr，
 * 表示无需额外的转换缓冲区，调用者可以直接使用原始数据。
 *
 * @param input_data      输入PCM数据缓冲区
 * @param input_size      输入PCM数据字节数
 * @param input_rate      输入采样率（Hz）
 * @param input_channels  输入声道数（1表示单声道，2表示立体声）
 * @param input_bits      输入位深（16或32）
 * @param target_rate     目标采样率（Hz）
 * @param target_channels 目标声道数（1或2）
 * @param target_bits     目标位深（16或32）
 * @param output_data     输出：转换后的PCM缓冲区指针（需要调用 heap_caps_free 释放）
 * @param output_size     输出：转换后数据的字节数
 * @return esp_err_t      ESP_OK 表示成功，其他错误码表示转换失败
 */
esp_err_t remix_convert_pcm_to_format(const uint8_t* input_data,
                                       size_t input_size,
                                       uint32_t input_rate,
                                       uint32_t input_channels,
                                       uint32_t input_bits,
                                       uint32_t target_rate,
                                       uint32_t target_channels,
                                       uint32_t target_bits,
                                       uint8_t** output_data,
                                       size_t* output_size);

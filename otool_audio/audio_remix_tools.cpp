// SPDX-FileCopyrightText: 2025 exia
// SPDX-License-Identifier: MIT

#include "audio_remix_tools.h"

#include <algorithm>
#include <cmath>

#include "sdkconfig.h"
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL (5)
#endif
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace {

constexpr const char* TAG = "audio_remix_tools";

inline void* calloc_spiram(size_t count, size_t size)
{
	if (count == 0 || size == 0) {
		return nullptr;
	}

	void* ptr = heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!ptr) {
		ptr = heap_caps_calloc(count, size, MALLOC_CAP_DEFAULT);
	}
	return ptr;
}

inline size_t get_sample_size(audio_data_type_t type)
{
	switch (type) {
		case AUDIO_TYPE_INT8:    return 1;
		case AUDIO_TYPE_INT16:   return 2;
		case AUDIO_TYPE_INT32:   return 4;
		case AUDIO_TYPE_FLOAT32: return 4;
		default: return 0;
	}
}

inline bool is_supported_type(audio_data_type_t type)
{
	return type == AUDIO_TYPE_INT8 || 
	       type == AUDIO_TYPE_INT16 || 
	       type == AUDIO_TYPE_INT32 || 
	       type == AUDIO_TYPE_FLOAT32;
}

inline float int_sample_to_float(int32_t sample, audio_data_type_t type)
{
	switch (type) {
		case AUDIO_TYPE_INT8:
			return static_cast<float>(sample) / 128.0f;
		case AUDIO_TYPE_INT16:
			return static_cast<float>(sample) / 32768.0f;
		case AUDIO_TYPE_INT32:
			return static_cast<float>(sample) / 2147483648.0f;
		default:
			return 0.0f;
	}
}

inline int32_t float_to_int_sample(float value, audio_data_type_t type)
{
	const float clamped = std::clamp(value, -1.0f, 1.0f);
	switch (type) {
		case AUDIO_TYPE_INT8:
			return static_cast<int32_t>(std::lround(clamped * 127.0f));
		case AUDIO_TYPE_INT16:
			return static_cast<int32_t>(std::lround(clamped * 32767.0f));
		case AUDIO_TYPE_INT32:
			return static_cast<int32_t>(std::llround(static_cast<double>(clamped) * 2147483647.0));
		default:
			return 0;
	}
}

// 将任意类型的音频数据转换为float（归一化到[-1.0, 1.0]）
void convert_to_float(const uint8_t* input, size_t total_samples, 
                      audio_data_type_t input_type, float* output)
{
	switch (input_type) {
		case AUDIO_TYPE_INT8: {
			const int8_t* in8 = reinterpret_cast<const int8_t*>(input);
			for (size_t i = 0; i < total_samples; ++i) {
				output[i] = int_sample_to_float(static_cast<int32_t>(in8[i]), AUDIO_TYPE_INT8);
			}
			break;
		}
		case AUDIO_TYPE_INT16: {
			const int16_t* in16 = reinterpret_cast<const int16_t*>(input);
			for (size_t i = 0; i < total_samples; ++i) {
				output[i] = int_sample_to_float(static_cast<int32_t>(in16[i]), AUDIO_TYPE_INT16);
			}
			break;
		}
		case AUDIO_TYPE_INT32: {
			const int32_t* in32 = reinterpret_cast<const int32_t*>(input);
			for (size_t i = 0; i < total_samples; ++i) {
				output[i] = int_sample_to_float(in32[i], AUDIO_TYPE_INT32);
			}
			break;
		}
		case AUDIO_TYPE_FLOAT32: {
			const float* inf32 = reinterpret_cast<const float*>(input);
			std::copy(inf32, inf32 + total_samples, output);
			break;
		}
	}
}

// 将float数据转换为任意类型
void convert_from_float(const float* input, size_t total_samples,
                        audio_data_type_t output_type, uint8_t* output)
{
	switch (output_type) {
		case AUDIO_TYPE_INT8: {
			int8_t* out8 = reinterpret_cast<int8_t*>(output);
			for (size_t i = 0; i < total_samples; ++i) {
				out8[i] = static_cast<int8_t>(float_to_int_sample(input[i], AUDIO_TYPE_INT8));
			}
			break;
		}
		case AUDIO_TYPE_INT16: {
			int16_t* out16 = reinterpret_cast<int16_t*>(output);
			for (size_t i = 0; i < total_samples; ++i) {
				out16[i] = static_cast<int16_t>(float_to_int_sample(input[i], AUDIO_TYPE_INT16));
			}
			break;
		}
		case AUDIO_TYPE_INT32: {
			int32_t* out32 = reinterpret_cast<int32_t*>(output);
			for (size_t i = 0; i < total_samples; ++i) {
				out32[i] = float_to_int_sample(input[i], AUDIO_TYPE_INT32);
			}
			break;
		}
		case AUDIO_TYPE_FLOAT32: {
			float* outf32 = reinterpret_cast<float*>(output);
			std::copy(input, input + total_samples, outf32);
			break;
		}
	}
}

} // end anonymous namespace (temporarily close for resample_linear)

esp_err_t resample_linear(const float* input,
						  size_t input_samples_per_channel,
						  uint32_t channel_count,
						  uint32_t input_rate,
						  uint32_t output_rate,
						  float** output,
						  size_t* output_samples_per_channel)
{
	if (!input || !output || !output_samples_per_channel || channel_count == 0 || input_rate == 0 || output_rate == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	if (input_samples_per_channel == 0) {
		return ESP_ERR_INVALID_SIZE;
	}

	if (input_rate == output_rate) {
		const size_t total_samples = input_samples_per_channel * channel_count;
		float* copy = static_cast<float*>(calloc_spiram(total_samples, sizeof(float)));
		if (!copy) {
			return ESP_ERR_NO_MEM;
		}
		std::copy(input, input + total_samples, copy);
		*output = copy;
		*output_samples_per_channel = input_samples_per_channel;
		return ESP_OK;
	}

	const double rate_ratio = static_cast<double>(input_rate) / static_cast<double>(output_rate);
	size_t estimated_samples = static_cast<size_t>((static_cast<uint64_t>(input_samples_per_channel) * output_rate + input_rate / 2) / input_rate);
	if (estimated_samples == 0) {
		estimated_samples = 1;
	}

	float* out = static_cast<float*>(calloc_spiram(estimated_samples * channel_count, sizeof(float)));
	if (!out) {
		return ESP_ERR_NO_MEM;
	}

	for (size_t i = 0; i < estimated_samples; ++i) {
		const double src_pos = static_cast<double>(i) * rate_ratio;
		const size_t idx = static_cast<size_t>(src_pos);
		const double frac = src_pos - static_cast<double>(idx);

		const size_t idx_next = (idx + 1 < input_samples_per_channel) ? (idx + 1) : (input_samples_per_channel - 1);

		for (uint32_t ch = 0; ch < channel_count; ++ch) {
			const float s0 = input[idx * channel_count + ch];
			const float s1 = input[idx_next * channel_count + ch];
			out[i * channel_count + ch] = static_cast<float>((1.0 - frac) * static_cast<double>(s0) + frac * static_cast<double>(s1));
		}
	}

	*output = out;
	*output_samples_per_channel = estimated_samples;
	return ESP_OK;
}

namespace { // reopen anonymous namespace for remaining internal helpers

esp_err_t convert_channels(const float* input,
						   size_t samples_per_channel,
						   uint32_t input_channels,
						   uint32_t output_channels,
						   float** output)
{
	if (!input || !output || samples_per_channel == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	if (input_channels == output_channels) {
		const size_t total_samples = samples_per_channel * input_channels;
		float* copy = static_cast<float*>(calloc_spiram(total_samples, sizeof(float)));
		if (!copy) {
			return ESP_ERR_NO_MEM;
		}
		std::copy(input, input + total_samples, copy);
		*output = copy;
		return ESP_OK;
	}

	if (input_channels == 1 && output_channels == 2) {
		float* dup = static_cast<float*>(calloc_spiram(samples_per_channel * output_channels, sizeof(float)));
		if (!dup) {
			return ESP_ERR_NO_MEM;
		}
		for (size_t i = 0; i < samples_per_channel; ++i) {
			const float v = input[i];
			dup[i * 2] = v;
			dup[i * 2 + 1] = v;
		}
		*output = dup;
		return ESP_OK;
	}

	if (input_channels == 2 && output_channels == 1) {
		float* mix = static_cast<float*>(calloc_spiram(samples_per_channel, sizeof(float)));
		if (!mix) {
			return ESP_ERR_NO_MEM;
		}
		for (size_t i = 0; i < samples_per_channel; ++i) {
			const float l = input[i * 2];
			const float r = input[i * 2 + 1];
			mix[i] = 0.5f * (l + r);
		}
		*output = mix;
		return ESP_OK;
	}

	ESP_LOGE(TAG, "Unsupported channel conversion: %u -> %u",
	         static_cast<unsigned>(input_channels),
	         static_cast<unsigned>(output_channels));
	return ESP_ERR_NOT_SUPPORTED;
}

} // namespace

// ============================================================================
// 公共导出函数 - 辅助工具
// ============================================================================

audio_data_type_t bits_to_audio_data_type(uint32_t bits)
{
	switch (bits) {
		case 8:  return AUDIO_TYPE_INT8;
		case 16: return AUDIO_TYPE_INT16;
		case 32: return AUDIO_TYPE_INT32;
		default: 
			ESP_LOGW(TAG, "Unsupported bit depth: %u, defaulting to 16-bit", static_cast<unsigned>(bits));
			return AUDIO_TYPE_INT16;
	}
}

// ============================================================================
// 公共导出函数 - 格式转换
// ============================================================================

esp_err_t remix_convert_pcm_to_format(const uint8_t* input_data,
									   size_t input_size,
									   uint32_t input_rate,
									   uint32_t input_channels,
									   audio_data_type_t input_type,
									   uint32_t target_rate,
									   uint32_t target_channels,
									   audio_data_type_t target_type,
									   uint8_t** output_data,
									   size_t* output_size)
{
	if (!output_data || !output_size) {
		return ESP_ERR_INVALID_ARG;
	}

	*output_data = nullptr;
	*output_size = 0;

	if (!input_data || input_size == 0 || input_rate == 0 || target_rate == 0 || 
	    input_channels == 0 || target_channels == 0) {
		ESP_LOGE(TAG, "Invalid arguments: input=%p size=%zu rate_in=%u rate_out=%u ch_in=%u ch_out=%u",
				 input_data, input_size,
				 static_cast<unsigned>(input_rate),
				 static_cast<unsigned>(target_rate),
				 static_cast<unsigned>(input_channels),
				 static_cast<unsigned>(target_channels));
		return ESP_ERR_INVALID_ARG;
	}

	if (!is_supported_type(input_type) || !is_supported_type(target_type)) {
		ESP_LOGE(TAG, "Unsupported data type: in=%d, out=%d", input_type, target_type);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if (input_channels > 2 || target_channels > 2) {
		ESP_LOGE(TAG, "Unsupported channel count: in=%u, out=%u (max=2)",
		         static_cast<unsigned>(input_channels),
		         static_cast<unsigned>(target_channels));
		return ESP_ERR_NOT_SUPPORTED;
	}

	const size_t input_sample_size = get_sample_size(input_type);
	if (input_sample_size == 0 || (input_size % (input_sample_size * input_channels)) != 0) {
		ESP_LOGE(TAG, "Input size mismatch: bytes=%zu, type=%d, channels=%u",
		         input_size, input_type, static_cast<unsigned>(input_channels));
		return ESP_ERR_INVALID_SIZE;
	}

	// 检查是否需要任何转换
	const bool need_resample = (input_rate != target_rate);
	const bool need_channel_convert = (input_channels != target_channels);
	const bool need_type_convert = (input_type != target_type);

	if (!need_resample && !need_channel_convert && !need_type_convert) {
		// 完全一致，无需转换
		*output_data = nullptr;
		*output_size = input_size;
		ESP_LOGI(TAG, "No conversion needed (identical format)");
		return ESP_OK;
	}

	// 计算输入采样点数
	const size_t samples_per_channel = input_size / (input_sample_size * input_channels);
	const size_t total_input_samples = samples_per_channel * input_channels;

	// 辅助函数：获取类型名称字符串
	auto get_type_name = [](audio_data_type_t type) -> const char* {
		switch (type) {
			case AUDIO_TYPE_INT8: return "i8";
			case AUDIO_TYPE_INT16: return "i16";
			case AUDIO_TYPE_INT32: return "i32";
			case AUDIO_TYPE_FLOAT32: return "f32";
			default: return "???";
		}
	};

	const size_t output_samples_per_channel =
	    need_resample ? static_cast<size_t>((samples_per_channel * target_rate + input_rate/2) / input_rate)
	                  : samples_per_channel;

	ESP_LOGI(TAG, "Converting: %zux%u@%uHz %s -> %zux%u@%uHz %s",
	         samples_per_channel,
	         static_cast<unsigned>(input_channels),
	         static_cast<unsigned>(input_rate),
	         get_type_name(input_type),
	         output_samples_per_channel,
	         static_cast<unsigned>(target_channels),
	         static_cast<unsigned>(target_rate),
	         get_type_name(target_type));

	// 步骤1: 转换为float中间格式
	float* float_buffer = static_cast<float*>(calloc_spiram(total_input_samples, sizeof(float)));
	if (!float_buffer) {
		ESP_LOGE(TAG, "Failed to allocate float buffer (%zu samples)", total_input_samples);
		return ESP_ERR_NO_MEM;
	}

	convert_to_float(input_data, total_input_samples, input_type, float_buffer);

	// 步骤2: 重采样（如果需要）
	float* resampled_buffer = nullptr;
	size_t current_samples_per_channel = samples_per_channel;
	const float* current_buffer = float_buffer;
	uint32_t current_channels = input_channels;

	if (need_resample) {
		esp_err_t ret = resample_linear(float_buffer,
										samples_per_channel,
										input_channels,
										input_rate,
										target_rate,
										&resampled_buffer,
										&current_samples_per_channel);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Resample failed: %s", esp_err_to_name(ret));
			heap_caps_free(float_buffer);
			return ret;
		}
		current_buffer = resampled_buffer;
	}

	// 步骤3: 声道转换（如果需要）
	float* channel_buffer = nullptr;
	const float* final_float_buffer = current_buffer;

	if (need_channel_convert) {
		esp_err_t ret = convert_channels(current_buffer,
										 current_samples_per_channel,
										 current_channels,
										 target_channels,
										 &channel_buffer);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Channel convert failed: %s", esp_err_to_name(ret));
			if (resampled_buffer) heap_caps_free(resampled_buffer);
			heap_caps_free(float_buffer);
			return ret;
		}
		final_float_buffer = channel_buffer;
		current_channels = target_channels;
	}

	// 步骤4: 转换为目标数据类型
	const size_t final_samples_per_channel = current_samples_per_channel;
	const size_t final_total_samples = final_samples_per_channel * target_channels;
	const size_t target_sample_size = get_sample_size(target_type);

	if (final_total_samples == 0 || target_sample_size == 0) {
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Invalid final sample count or type");
		return ESP_ERR_INVALID_SIZE;
	}

	void* final_buffer = calloc_spiram(final_total_samples, target_sample_size);
	if (!final_buffer) {
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Failed to allocate output buffer (%zu samples)", final_total_samples);
		return ESP_ERR_NO_MEM;
	}

	convert_from_float(final_float_buffer, final_total_samples, target_type, 
	                   static_cast<uint8_t*>(final_buffer));

	*output_data = reinterpret_cast<uint8_t*>(final_buffer);
	*output_size = final_total_samples * target_sample_size;

	// 清理中间缓冲区
	if (channel_buffer) heap_caps_free(channel_buffer);
	if (resampled_buffer) heap_caps_free(resampled_buffer);
	heap_caps_free(float_buffer);

	ESP_LOGI(TAG, "Conversion completed: output_size=%zu bytes", *output_size);
	return ESP_OK;
}

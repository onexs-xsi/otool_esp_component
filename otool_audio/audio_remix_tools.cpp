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

inline float int_sample_to_float(int32_t sample, uint32_t bits)
{
	if (bits == 16) {
		return static_cast<float>(sample) / 32768.0f;
	}
	if (bits == 32) {
		return static_cast<float>(sample) / 2147483648.0f;
	}
	return 0.0f;
}

inline int32_t float_to_int_sample(float value, uint32_t bits)
{
	const float clamped = std::clamp(value, -1.0f, 1.0f);
	if (bits == 16) {
		return static_cast<int32_t>(std::lround(clamped * 32767.0f));
	}
	if (bits == 32) {
		return static_cast<int32_t>(std::llround(static_cast<double>(clamped) * 2147483647.0));
	}
	return 0;
}

inline bool is_supported_bits(uint32_t bits)
{
	return bits == 16 || bits == 32;
}

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

	ESP_LOGE(TAG, "Unsupported channel conversion: %u -> %u", input_channels, output_channels);
	return ESP_ERR_NOT_SUPPORTED;
}

template <typename TInt>
void convert_float_to_int_buffer(const float* input,
								 size_t total_samples,
								 uint32_t target_bits,
								 TInt* output)
{
	for (size_t i = 0; i < total_samples; ++i) {
		output[i] = static_cast<TInt>(float_to_int_sample(input[i], target_bits));
	}
}

} // namespace

esp_err_t remix_convert_pcm_to_format(const uint8_t* input_data,
									   size_t input_size,
									   uint32_t input_rate,
									   uint32_t input_channels,
									   uint32_t input_bits,
									   uint32_t target_rate,
									   uint32_t target_channels,
									   uint32_t target_bits,
									   uint8_t** output_data,
									   size_t* output_size)
{
	if (!output_data || !output_size) {
		return ESP_ERR_INVALID_ARG;
	}

	*output_data = nullptr;
	*output_size = 0;

	if (!input_data || input_size == 0 || input_rate == 0 || target_rate == 0 || input_channels == 0 || target_channels == 0) {
		ESP_LOGE(TAG, "Invalid arguments: input=%p size=%zu rate_in=%u rate_out=%u ch_in=%u ch_out=%u",
				 input_data, input_size, input_rate, target_rate, input_channels, target_channels);
		return ESP_ERR_INVALID_ARG;
	}

	if (!is_supported_bits(input_bits) || !is_supported_bits(target_bits)) {
		ESP_LOGE(TAG, "Unsupported bit depth: in=%u, out=%u", input_bits, target_bits);
		return ESP_ERR_NOT_SUPPORTED;
	}

	const size_t input_bytes_per_sample = input_bits / 8;
	if (input_bytes_per_sample == 0 || (input_size % (input_bytes_per_sample * input_channels)) != 0) {
		ESP_LOGE(TAG, "Input PCM size mismatch: bytes=%zu, in_bits=%u, channels=%u", input_size, input_bits, input_channels);
		return ESP_ERR_INVALID_SIZE;
	}

	const bool need_resample = input_rate != target_rate;
	const bool need_channel_convert = input_channels != target_channels;
	const bool need_bit_convert = input_bits != target_bits;

	if (!need_resample && !need_channel_convert && !need_bit_convert) {
		// 完全一致，无需转换
		*output_data = nullptr;
		*output_size = input_size;
		return ESP_OK;
	}

	const size_t samples_per_channel = input_size / (input_bytes_per_sample * input_channels);
	const size_t total_samples = samples_per_channel * input_channels;

	float* float_buffer = static_cast<float*>(calloc_spiram(total_samples, sizeof(float)));
	if (!float_buffer) {
		ESP_LOGE(TAG, "Failed to allocate float conversion buffer (%zu samples)", total_samples);
		return ESP_ERR_NO_MEM;
	}

	if (input_bits == 16) {
		const int16_t* in16 = reinterpret_cast<const int16_t*>(input_data);
		for (size_t i = 0; i < total_samples; ++i) {
			float_buffer[i] = int_sample_to_float(static_cast<int32_t>(in16[i]), 16);
		}
	} else {
		const int32_t* in32 = reinterpret_cast<const int32_t*>(input_data);
		for (size_t i = 0; i < total_samples; ++i) {
			float_buffer[i] = int_sample_to_float(in32[i], 32);
		}
	}

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

	const size_t channel_samples_per_channel = current_samples_per_channel;

	if (!need_channel_convert && need_resample) {
		final_float_buffer = current_buffer;
	}

	const uint32_t final_channels = current_channels;
	const size_t final_total_samples = channel_samples_per_channel * final_channels;

	if (final_total_samples == 0) {
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Final sample count is zero");
		return ESP_ERR_INVALID_SIZE;
	}

	const size_t target_bytes_per_sample = target_bits / 8;
	if (target_bytes_per_sample == 0) {
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Invalid target bit depth: %u", target_bits);
		return ESP_ERR_INVALID_ARG;
	}

	void* final_buffer = calloc_spiram(final_total_samples, target_bytes_per_sample);
	if (!final_buffer) {
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Failed to allocate final buffer (%zu samples)", final_total_samples);
		return ESP_ERR_NO_MEM;
	}

	if (target_bits == 16) {
		auto* out16 = reinterpret_cast<int16_t*>(final_buffer);
		convert_float_to_int_buffer(final_float_buffer, final_total_samples, 16, out16);
	} else if (target_bits == 32) {
		auto* out32 = reinterpret_cast<int32_t*>(final_buffer);
		convert_float_to_int_buffer(final_float_buffer, final_total_samples, 32, out32);
	} else {
		// 理论上不会走到这里，前面已经过滤
		heap_caps_free(final_buffer);
		if (channel_buffer) heap_caps_free(channel_buffer);
		if (resampled_buffer) heap_caps_free(resampled_buffer);
		heap_caps_free(float_buffer);
		ESP_LOGE(TAG, "Unsupported target bits: %u", target_bits);
		return ESP_ERR_NOT_SUPPORTED;
	}

	*output_data = reinterpret_cast<uint8_t*>(final_buffer);
	*output_size = final_total_samples * target_bytes_per_sample;

	if (channel_buffer) heap_caps_free(channel_buffer);
	if (resampled_buffer) heap_caps_free(resampled_buffer);
	heap_caps_free(float_buffer);

	return ESP_OK;
}

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_recorder.h"
#include "audio_tools.h"
#include "audio_playback.h"
#include "audio_sr_afe.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <sys/stat.h>
#include <math.h>
#include <cctype>

static const char *TAG = "audio_recorder";

// ============================================================================
// 工具函数 (anonymous namespace)
// ============================================================================

namespace {

static inline int16_t load_int16_le(const uint8_t* ptr)
{
    int16_t value = 0;
    memcpy(&value, ptr, sizeof(int16_t));
    return value;
}

static inline uint32_t load_uint32_le(const uint8_t* ptr)
{
    uint32_t value = 0;
    memcpy(&value, ptr, sizeof(uint32_t));
    return value;
}

} // namespace

// 确保输出文件的父目录存在，如果不存在则递归创建
static esp_err_t ensure_parent_directories(const char* filepath)
{
    if (!filepath) {
        return ESP_ERR_INVALID_ARG;
    }

    const char* last_slash = strrchr(filepath, '/');
    if (!last_slash) {
        return ESP_OK; // 没有目录部分
    }

    size_t dir_len = static_cast<size_t>(last_slash - filepath);
    if (dir_len == 0) {
        return ESP_OK; // 根目录，无需创建
    }

    std::string dir_path(filepath, dir_len);
    if (dir_path.empty() || dir_path == "." || dir_path == "/") {
        return ESP_OK;
    }

    std::string current_path;
    size_t index = 0;
    if (!dir_path.empty() && dir_path[0] == '/') {
        current_path = "/";
        index = 1;
    }

    while (index <= dir_path.size()) {
        size_t next_sep = dir_path.find('/', index);
        size_t fragment_len = (next_sep == std::string::npos) ? (dir_path.size() - index) : (next_sep - index);
        std::string fragment = dir_path.substr(index, fragment_len);

        if (!fragment.empty()) {
            if (!current_path.empty() && current_path.back() != '/') {
                current_path += '/';
            }
            current_path += fragment;

            struct stat st {};
            if (stat(current_path.c_str(), &st) == 0) {
                if (!S_ISDIR(st.st_mode)) {
                    ESP_LOGE(TAG, "%s exists but is not a directory", current_path.c_str());
                    return ESP_ERR_INVALID_STATE;
                }
            } else {
                if (mkdir(current_path.c_str(), 0775) != 0) {
                    if (errno == EEXIST) {
                        continue;
                    }

                    if (stat(current_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
                        continue;
                    }

                    ESP_LOGE(TAG, "Failed to create directory %s (errno=%d, %s)",
                             current_path.c_str(), errno, strerror(errno));
                    return ESP_FAIL;
                }

                ESP_LOGI(TAG, "Created directory: %s", current_path.c_str());
            }
        }

        if (next_sep == std::string::npos) {
            break;
        }
        index = next_sep + 1;
    }

    return ESP_OK;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

audio_recorder::audio_recorder(audio_tools* parent)
    : parent_(parent)
{
    ESP_LOGI(TAG, "audio_recorder sub-object created");
}

audio_recorder::~audio_recorder()
{
    if (record_session_task_handle_) {
        ESP_LOGW(TAG, "Stopping active record session before destruction");
        record_session_stop();
    }
    ESP_LOGI(TAG, "audio_recorder sub-object destroyed");
}

// ============================================================================
// 静态通道拆分工具
// ============================================================================

void audio_recorder::free_channel_split_result(channel_split_result_t& result)
{
    for (int i = 0; i < 4; ++i) {
        if (result.mic_buffers[i]) {
            free(result.mic_buffers[i]);
            result.mic_buffers[i] = nullptr;
        }
    }

    result.samples_per_channel = 0;
    result.bytes_per_sample = 0;
    result.status = ESP_OK;
}

channel_split_result_t audio_recorder::split_recorded_channels(const uint8_t* record_buffer,
                                                                size_t bytes_read,
                                                                const esp_codec_dev_sample_info_t& fs,
                                                                bool is_tdm_mode,
                                                                audio_mic_channel_t mic_channels)
{
    channel_split_result_t result{};
    result.status = ESP_FAIL;
    result.enabled_mask = mic_channels;
    result.is_tdm_mode = is_tdm_mode;
    result.bytes_per_sample = (fs.bits_per_sample >> 3);

    if (!record_buffer || bytes_read == 0) {
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    if (result.bytes_per_sample == 0) {
        ESP_LOGE(TAG, "Invalid bits_per_sample: %u", static_cast<unsigned>(fs.bits_per_sample));
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    uint8_t enabled_indices[4] = {0};
    uint8_t mic_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (mic_channels & (1 << i)) {
            enabled_indices[mic_count++] = static_cast<uint8_t>(i);
        }
    }

    if (mic_count == 0) {
        ESP_LOGE(TAG, "No microphone channels enabled for splitting (mask=0x%02X)",
                 static_cast<unsigned>(mic_channels));
        result.status = ESP_ERR_INVALID_ARG;
        return result;
    }

    size_t samples_per_channel = 0;

    if (is_tdm_mode) {
        const size_t stereo_frame_bytes = result.bytes_per_sample * 2;
        if (result.bytes_per_sample != sizeof(uint32_t)) {
            ESP_LOGW(TAG, "Unexpected slot width %zu bytes in TDM mode; assuming 32-bit packing",
                     result.bytes_per_sample);
        }
        if (stereo_frame_bytes == 0) {
            ESP_LOGE(TAG, "Invalid TDM frame size calculation (slot bytes=%zu)", result.bytes_per_sample);
            result.status = ESP_ERR_INVALID_STATE;
            return result;
        }
        if (bytes_read % stereo_frame_bytes != 0) {
            ESP_LOGW(TAG, "Recorded byte count (%zu) not aligned with TDM frame size (%zu); trailing data ignored",
                     bytes_read, stereo_frame_bytes);
        }
        samples_per_channel = bytes_read / stereo_frame_bytes;
    } else {
        size_t channel_count = fs.channel ? static_cast<size_t>(fs.channel) : 1;
        const size_t frame_bytes = result.bytes_per_sample * channel_count;
        if (frame_bytes == 0) {
            ESP_LOGE(TAG, "Invalid STD frame size calculation (slot bytes=%zu, channels=%u)",
                     result.bytes_per_sample, static_cast<unsigned>(fs.channel));
            result.status = ESP_ERR_INVALID_STATE;
            return result;
        }
        samples_per_channel = bytes_read / frame_bytes;
    }

    if (samples_per_channel == 0) {
        ESP_LOGE(TAG, "Channel splitting failed: no samples detected (bytes=%zu)", bytes_read);
        result.status = ESP_ERR_INVALID_SIZE;
        return result;
    }

    result.samples_per_channel = samples_per_channel;

    for (int i = 0; i < 4; ++i) {
        if (mic_channels & (1 << i)) {
            result.mic_buffers[i] = static_cast<int16_t*>(malloc(samples_per_channel * sizeof(int16_t)));
            if (!result.mic_buffers[i]) {
                ESP_LOGE(TAG, "Failed to allocate channel buffer for MIC%u (%zu samples)",
                         static_cast<unsigned>(i + 1), samples_per_channel);
                for (int j = 0; j < 4; ++j) {
                    if (result.mic_buffers[j]) {
                        free(result.mic_buffers[j]);
                        result.mic_buffers[j] = nullptr;
                    }
                }
                result.status = ESP_ERR_NO_MEM;
                return result;
            }
            memset(result.mic_buffers[i], 0, samples_per_channel * sizeof(int16_t));
        }
    }

    if (is_tdm_mode) {
        const size_t stereo_frame_bytes = result.bytes_per_sample * 2;
        const size_t total_frames = bytes_read / stereo_frame_bytes;

        if (result.bytes_per_sample != sizeof(uint32_t)) {
            ESP_LOGW(TAG, "TDM splitting operating without 32-bit slot width validation (slot=%zu)",
                     result.bytes_per_sample * 8);
        }

        for (size_t frame = 0; frame < total_frames; ++frame) {
            size_t base = frame * stereo_frame_bytes;
            int16_t left_high = 0;
            int16_t left_low = 0;
            int16_t right_high = 0;
            int16_t right_low = 0;

            if (result.bytes_per_sample == sizeof(uint32_t)) {
                uint32_t left_word = load_uint32_le(record_buffer + base);
                uint32_t right_word = load_uint32_le(record_buffer + base + result.bytes_per_sample);
                left_high = static_cast<int16_t>(left_word >> 16);
                left_low = static_cast<int16_t>(left_word & 0xFFFF);
                right_high = static_cast<int16_t>(right_word >> 16);
                right_low = static_cast<int16_t>(right_word & 0xFFFF);
            } else {
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = load_int16_le(record_buffer + base + result.bytes_per_sample);
                left_high = left_sample;
                left_low = left_sample;
                right_high = right_sample;
                right_low = right_sample;
            }

            if (result.mic_buffers[0]) {                 // MIC1 (LRCK Low, 第一槽)
                result.mic_buffers[0][frame] = left_high;
            }
            if (result.mic_buffers[2]) {                 // MIC3 (LRCK Low, 第二槽 - 奇数通道)
                result.mic_buffers[2][frame] = left_low;
            }
            if (result.mic_buffers[1]) {                 // MIC2 (LRCK High, 第一槽 - 偶数通道)
                result.mic_buffers[1][frame] = right_high;
            }
            if (result.mic_buffers[3]) {                 // MIC4 (LRCK High, 第二槽)
                result.mic_buffers[3][frame] = right_low;
            }
        }
    } else {
        const size_t channel_count = fs.channel ? static_cast<size_t>(fs.channel) : 1;
        const size_t frame_bytes = result.bytes_per_sample * channel_count;

        if (channel_count > 2) {
            ESP_LOGW(TAG, "STD mode detected with %u channels; using first two for MIC mapping",
                     static_cast<unsigned>(fs.channel));
        }

        if (channel_count == 1 && mic_count > 1) {
            ESP_LOGW(TAG, "Multiple microphones enabled with mono STD stream; duplicating mono data across channels");
        }

        if (mic_count == 1) {
            const uint8_t mic_index = enabled_indices[0];
            const bool is_left_channel = (mic_index == 0 || mic_index == 2);
            const uint8_t channel_offset = is_left_channel ? 0 : 1;

            if (channel_count == 1) {
                for (size_t i = 0; i < samples_per_channel; ++i) {
                    int16_t sample = load_int16_le(record_buffer + i * result.bytes_per_sample);
                    if (result.mic_buffers[mic_index]) {
                        result.mic_buffers[mic_index][i] = sample;
                    }
                }
            } else {
                for (size_t i = 0; i < samples_per_channel; ++i) {
                    size_t sample_pos = (i * channel_count + channel_offset) * result.bytes_per_sample;
                    int16_t sample = load_int16_le(record_buffer + sample_pos);
                    if (result.mic_buffers[mic_index]) {
                        result.mic_buffers[mic_index][i] = sample;
                    }
                }
            }
        } else if (mic_count == 2) {
            const uint8_t first_mic = enabled_indices[0];
            const uint8_t second_mic = enabled_indices[1];

            for (size_t i = 0; i < samples_per_channel; ++i) {
                size_t base = i * frame_bytes;
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = (channel_count >= 2)
                                           ? load_int16_le(record_buffer + base + result.bytes_per_sample)
                                           : left_sample;

                if (result.mic_buffers[first_mic]) {
                    result.mic_buffers[first_mic][i] = left_sample;
                }
                if (result.mic_buffers[second_mic]) {
                    result.mic_buffers[second_mic][i] = right_sample;
                }
            }
        } else {
            if (mic_count == 3) {
                ESP_LOGW(TAG, "STD mode with 3 microphones enabled; one slot may carry zero samples");
            } else {
                ESP_LOGW(TAG, "STD mode with 4 microphones enabled; consider using TDM for optimal layout");
            }

            for (size_t i = 0; i < samples_per_channel; ++i) {
                size_t base = i * frame_bytes;
                int16_t left_sample = load_int16_le(record_buffer + base);
                int16_t right_sample = (channel_count >= 2)
                                           ? load_int16_le(record_buffer + base + result.bytes_per_sample)
                                           : left_sample;

                if (result.mic_buffers[0]) {
                    result.mic_buffers[0][i] = left_sample;
                }
                if (result.mic_buffers[1]) {
                    result.mic_buffers[1][i] = right_sample;
                }
                if (result.mic_buffers[2]) {
                    result.mic_buffers[2][i] = left_sample;
                }
                if (result.mic_buffers[3]) {
                    result.mic_buffers[3][i] = right_sample;
                }
            }
        }
    }

    result.status = ESP_OK;
    return result;
}

void audio_recorder::compute_split_channel_quality(const channel_split_result_t& split_result,
                                                    mic_channel_quality_t quality[4])
{
    const size_t samples = split_result.samples_per_channel;

    for (int i = 0; i < 4; ++i) {
        mic_channel_quality_t& out = quality[i];
        out.available = (split_result.mic_buffers[i] != nullptr);
        out.sample_count = out.available ? samples : 0;
        out.min_value = out.available ? INT16_MAX : 0;
        out.max_value = out.available ? INT16_MIN : 0;
        out.average_abs_amplitude = 0;
        out.rms_db = -96.0;
        out.zero_percent = 0.0;
        out.clipped_percent = 0.0;

        if (!out.available || samples == 0) {
            if (out.available) {
                out.min_value = 0;
                out.max_value = 0;
            }
            continue;
        }

        const int16_t* data = split_result.mic_buffers[i];
        int64_t sum_abs = 0;
        size_t zero_count = 0;
        size_t clipped_count = 0;
        long double sum_square = 0.0;

        for (size_t sample_index = 0; sample_index < samples; ++sample_index) {
            int16_t sample = data[sample_index];
            if (sample == 0) {
                zero_count++;
            }
            if (sample == INT16_MAX || sample == INT16_MIN) {
                clipped_count++;
            }
            sum_abs += llabs(static_cast<long long>(sample));
            if (sample < out.min_value) {
                out.min_value = sample;
            }
            if (sample > out.max_value) {
                out.max_value = sample;
            }
            long double v = static_cast<long double>(sample);
            sum_square += v * v;
        }

        out.average_abs_amplitude = static_cast<int32_t>(sum_abs / static_cast<int64_t>(samples));
        const long double mean_square = sum_square / static_cast<long double>(samples);
        const double mean_square_double = static_cast<double>(mean_square);
        const double rms_linear = (mean_square_double > 0.0) ? sqrt(mean_square_double) : 0.0;
        out.rms_db = (rms_linear > 0.0) ? (20.0 * log10(rms_linear / 32768.0)) : -96.0;
        out.zero_percent = (samples > 0) ? (static_cast<double>(zero_count) * 100.0 / static_cast<double>(samples)) : 0.0;
        out.clipped_percent = (samples > 0) ? (static_cast<double>(clipped_count) * 100.0 / static_cast<double>(samples)) : 0.0;
    }
}

// ============================================================================
// 流式录音会话 (TASK-1 stubs)
// ============================================================================

esp_err_t audio_recorder::sample_mic_channels(uint32_t duration_ms,
                                               mic_channel_quality_t quality[4],
                                               size_t* bytes_read_out,
                                               esp_codec_dev_sample_info_t* sample_info_out,
                                               mic_channel_sample_window_t* sample_window_out)
{
    if (!quality) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(quality, 0, sizeof(mic_channel_quality_t) * 4);
    for (int i = 0; i < 4; ++i) {
        quality[i].rms_db = -96.0;
    }
    if (bytes_read_out) {
        *bytes_read_out = 0;
    }
    if (sample_info_out) {
        *sample_info_out = {};
    }
    if (sample_window_out) {
        memset(sample_window_out, 0, sizeof(*sample_window_out));
    }

    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot sample microphones while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }
    if (!parent_->es7210_initialized || !parent_->record_dev) {
        ESP_LOGE(TAG, "ES7210 record path not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_ms == 0) {
        duration_ms = 100;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(parent_->sample_rate);
    fs.bits_per_sample = static_cast<uint32_t>(parent_->bits_per_sample);
    const bool is_tdm_mode = parent_->es7210_use_tdm;
    fs.channel = is_tdm_mode
                     ? 2
                     : static_cast<uint32_t>((parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2);
    if (fs.sample_rate == 0 || fs.bits_per_sample == 0 || fs.channel == 0) {
        ESP_LOGE(TAG, "Invalid sample format for mic sampling: sr=%u bits=%u ch=%u",
                 static_cast<unsigned>(fs.sample_rate),
                 static_cast<unsigned>(fs.bits_per_sample),
                 static_cast<unsigned>(fs.channel));
        return ESP_ERR_INVALID_STATE;
    }

    const size_t bytes_per_sample = fs.bits_per_sample >> 3;
    const size_t frame_size = bytes_per_sample * fs.channel;
    if (bytes_per_sample == 0 || frame_size == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint64_t target_bytes_64 = static_cast<uint64_t>(fs.sample_rate) *
                               static_cast<uint64_t>(frame_size) *
                               static_cast<uint64_t>(duration_ms) / 1000ULL;
    if (target_bytes_64 < frame_size) {
        target_bytes_64 = frame_size;
    }
    target_bytes_64 = (target_bytes_64 / frame_size) * frame_size;
    if (target_bytes_64 == 0 || target_bytes_64 > SIZE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t target_bytes = static_cast<size_t>(target_bytes_64);
    uint8_t* record_buffer = static_cast<uint8_t*>(malloc(target_bytes));
    if (!record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate mic sample buffer: %zu bytes", target_bytes);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = 0;
    esp_err_t read_ret = ESP_OK;
    const size_t block_size = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(duration_ms + 300);
    const TickType_t start_tick = xTaskGetTickCount();
    while (bytes_read < target_bytes) {
        const size_t remaining = target_bytes - bytes_read;
        size_t to_read = remaining > block_size ? block_size : remaining;
        to_read = (to_read / frame_size) * frame_size;
        if (to_read == 0) {
            to_read = frame_size;
        }
        read_ret = esp_codec_dev_read(parent_->record_dev,
                                      record_buffer + bytes_read,
                                      static_cast<int>(to_read));
        if (read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Mic sample read failed at %zu/%zu: %s",
                     bytes_read,
                     target_bytes,
                     esp_err_to_name(read_ret));
            break;
        }
        bytes_read += to_read;
        if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
            ESP_LOGW(TAG, "Mic sample timeout at %zu/%zu bytes", bytes_read, target_bytes);
            break;
        }
    }

    if (bytes_read_out) {
        *bytes_read_out = bytes_read;
    }
    if (sample_info_out) {
        *sample_info_out = fs;
    }

    if (read_ret != ESP_OK && bytes_read == 0) {
        free(record_buffer);
        return read_ret;
    }
    if (bytes_read == 0) {
        free(record_buffer);
        return ESP_ERR_INVALID_SIZE;
    }

    channel_split_result_t split_result = split_recorded_channels(record_buffer,
                                                                  bytes_read,
                                                                  fs,
                                                                  is_tdm_mode,
                                                                  parent_->mic_channels);
    free(record_buffer);
    if (split_result.status != ESP_OK) {
        ESP_LOGE(TAG, "Mic sample channel split failed: %s", esp_err_to_name(split_result.status));
        free_channel_split_result(split_result);
        return split_result.status;
    }

    compute_split_channel_quality(split_result, quality);
    if (sample_window_out) {
        sample_window_out->sample_rate_hz = fs.sample_rate;
        const size_t sample_count = split_result.samples_per_channel > AUDIO_MIC_SAMPLE_WINDOW_MAX_SAMPLES
                                        ? AUDIO_MIC_SAMPLE_WINDOW_MAX_SAMPLES
                                        : split_result.samples_per_channel;
        sample_window_out->sample_count = sample_count;
        for (int ch = 0; ch < 4; ++ch) {
            sample_window_out->available[ch] = split_result.mic_buffers[ch] != nullptr && sample_count > 0;
            if (sample_window_out->available[ch]) {
                memcpy(sample_window_out->samples[ch],
                       split_result.mic_buffers[ch],
                       sample_count * sizeof(int16_t));
            }
        }
    }
    free_channel_split_result(split_result);
    return ESP_OK;
}

esp_err_t audio_recorder::capture_mic_channels(
    uint32_t duration_ms,
    channel_split_result_t* split_result_out,
    mic_channel_quality_t quality[4],
    size_t* bytes_read_out,
    esp_codec_dev_sample_info_t* sample_info_out)
{
    if (!split_result_out || !quality) {
        return ESP_ERR_INVALID_ARG;
    }
    *split_result_out = {};
    memset(quality, 0, sizeof(mic_channel_quality_t) * 4);
    for (int i = 0; i < 4; ++i) {
        quality[i].rms_db = -96.0;
    }
    if (bytes_read_out) *bytes_read_out = 0;
    if (sample_info_out) *sample_info_out = {};

    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot capture microphones while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }
    if (!parent_->es7210_initialized || !parent_->record_dev) {
        ESP_LOGE(TAG, "ES7210 record path not ready");
        return ESP_ERR_INVALID_STATE;
    }
    if (duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(parent_->sample_rate);
    fs.bits_per_sample = static_cast<uint32_t>(parent_->bits_per_sample);
    const bool is_tdm_mode = parent_->es7210_use_tdm;
    fs.channel = is_tdm_mode
                     ? 2
                     : static_cast<uint32_t>((parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2);
    const size_t bytes_per_sample = fs.bits_per_sample >> 3;
    const size_t frame_size = bytes_per_sample * fs.channel;
    if (fs.sample_rate == 0 || bytes_per_sample == 0 || frame_size == 0) {
        ESP_LOGE(TAG, "Invalid sample format for mic capture: sr=%u bits=%u ch=%u",
                 static_cast<unsigned>(fs.sample_rate),
                 static_cast<unsigned>(fs.bits_per_sample),
                 static_cast<unsigned>(fs.channel));
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t target_bytes_64 = static_cast<uint64_t>(fs.sample_rate) * frame_size *
                               static_cast<uint64_t>(duration_ms) / 1000ULL;
    target_bytes_64 = (target_bytes_64 / frame_size) * frame_size;
    if (target_bytes_64 == 0 || target_bytes_64 > SIZE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t target_bytes = static_cast<size_t>(target_bytes_64);
    uint8_t* record_buffer = static_cast<uint8_t*>(
        heap_caps_malloc(target_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate mic capture buffer: %zu bytes", target_bytes);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = 0;
    esp_err_t read_ret = ESP_OK;
    const size_t block_size = 1024;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(duration_ms + 1000);
    const TickType_t start_tick = xTaskGetTickCount();
    while (bytes_read < target_bytes) {
        size_t to_read = std::min(block_size, target_bytes - bytes_read);
        to_read = (to_read / frame_size) * frame_size;
        if (to_read == 0) to_read = frame_size;
        read_ret = esp_codec_dev_read(parent_->record_dev,
                                      record_buffer + bytes_read,
                                      static_cast<int>(to_read));
        if (read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Mic capture read failed at %zu/%zu: %s",
                     bytes_read, target_bytes, esp_err_to_name(read_ret));
            break;
        }
        bytes_read += to_read;
        if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
            ESP_LOGW(TAG, "Mic capture timeout at %zu/%zu bytes", bytes_read, target_bytes);
            break;
        }
    }

    if (bytes_read_out) *bytes_read_out = bytes_read;
    if (sample_info_out) *sample_info_out = fs;
    if (read_ret != ESP_OK && bytes_read == 0) {
        heap_caps_free(record_buffer);
        return read_ret;
    }
    if (bytes_read == 0) {
        heap_caps_free(record_buffer);
        return ESP_ERR_INVALID_SIZE;
    }

    channel_split_result_t split_result = split_recorded_channels(record_buffer,
                                                                  bytes_read,
                                                                  fs,
                                                                  is_tdm_mode,
                                                                  parent_->mic_channels);
    heap_caps_free(record_buffer);
    if (split_result.status != ESP_OK) {
        free_channel_split_result(split_result);
        return split_result.status;
    }
    compute_split_channel_quality(split_result, quality);
    *split_result_out = split_result;
    return read_ret == ESP_OK ? ESP_OK : read_ret;
}

esp_err_t audio_recorder::record_session_start(const record_session_config_t& config)
{
    // TODO: TASK-1 实现
    ESP_LOGW(TAG, "record_session_start() not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

int audio_recorder::record_session_read(void* buf, size_t len, uint32_t timeout_ms)
{
    // TODO: TASK-1 实现
    ESP_LOGW(TAG, "record_session_read() not yet implemented");
    return -1;
}

esp_err_t audio_recorder::record_session_stop()
{
    // TODO: TASK-1 实现
    ESP_LOGW(TAG, "record_session_stop() not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

bool audio_recorder::record_session_is_running() const
{
    return record_session_task_handle_ != nullptr;
}

// ============================================================================
// 批处理录音 - record_to_file
// ============================================================================

esp_err_t audio_recorder::record_to_file(const char* filepath, uint32_t record_duration_seconds, size_t chunk_size)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    const bool es8311_adc_ready = parent_->es8311_initialized && parent_->es8311_has_adc_path() && (parent_->record_dev == parent_->es8311_dev_handle);
    const bool es7210_adc_ready = parent_->es7210_initialized;

    if (!parent_->system_initialized || !parent_->record_dev || (!es7210_adc_ready && !es8311_adc_ready)) {
        ESP_LOGE(TAG, "Audio recording path not ready (system:%d, es7210:%d, es8311_adc:%d, dev:%p)",
                 parent_->system_initialized, parent_->es7210_initialized, es8311_adc_ready, parent_->record_dev);
        return ESP_ERR_INVALID_STATE;
    }

    if (filepath == nullptr || filepath[0] == '\0' || record_duration_seconds == 0) {
        ESP_LOGE(TAG, "Invalid arguments: filepath=%p, duration=%lu",
                 filepath,
                 static_cast<unsigned long>(record_duration_seconds));
        return ESP_ERR_INVALID_ARG;
    }

    // 使用录音设备的声道配置
    uint32_t channels = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1u : 2u;
    uint32_t bits = static_cast<uint32_t>(parent_->bits_per_sample);
    uint32_t sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    uint32_t bytes_per_sample = (bits * channels) / 8;

    if (bytes_per_sample == 0 || sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid audio format: sr=%u, bits=%u, channels=%u",
                 static_cast<unsigned>(sample_rate_hz),
                 static_cast<unsigned>(bits),
                 static_cast<unsigned>(channels));
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t total_bytes_64 = static_cast<uint64_t>(sample_rate_hz) * record_duration_seconds * bytes_per_sample;
    if (total_bytes_64 == 0) {
        ESP_LOGE(TAG, "Calculated recording size is zero");
        return ESP_ERR_INVALID_STATE;
    }

    if (total_bytes_64 > SIZE_MAX) {
        ESP_LOGE(TAG, "Requested recording exceeds size_t capacity: %llu bytes", (unsigned long long)total_bytes_64);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t total_bytes = static_cast<size_t>(total_bytes_64);
    size_t frame_size = bytes_per_sample;
    if (chunk_size < frame_size) {
        ESP_LOGW(TAG, "chunk_size %zu too small, adjusting to frame size %zu", chunk_size, frame_size);
        chunk_size = frame_size;
    }

    size_t aligned_chunk = (chunk_size / frame_size) * frame_size;
    if (aligned_chunk == 0) {
        aligned_chunk = frame_size;
    }
    if (aligned_chunk != chunk_size) {
        ESP_LOGW(TAG, "Aligning chunk size from %zu to %zu to match frame size", chunk_size, aligned_chunk);
        chunk_size = aligned_chunk;
    }

    esp_err_t dir_ret = ensure_parent_directories(filepath);
    if (dir_ret != ESP_OK) {
        return dir_ret;
    }

    uint8_t* chunk_buffer = static_cast<uint8_t*>(malloc(chunk_size));
    if (!chunk_buffer) {
        ESP_LOGE(TAG, "Failed to allocate chunk buffer: %zu bytes", chunk_size);
        return ESP_ERR_NO_MEM;
    }

    FILE* file = fopen(filepath, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open %s for writing (errno=%d, %s)", filepath, errno, strerror(errno));
        free(chunk_buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Recording %u Hz, %u-bit, %u-ch audio for %lu s -> %zu bytes into %s",
             static_cast<unsigned>(sample_rate_hz),
             static_cast<unsigned>(bits),
             static_cast<unsigned>(channels),
             static_cast<unsigned long>(record_duration_seconds),
             total_bytes, filepath);

    size_t bytes_written = 0;
    esp_err_t ret = ESP_OK;
    TickType_t start_ticks = xTaskGetTickCount();

    while (bytes_written < total_bytes) {
        size_t remaining = total_bytes - bytes_written;
        size_t to_read = (chunk_size < remaining) ? chunk_size : remaining;
        ret = esp_codec_dev_read(parent_->record_dev, chunk_buffer, static_cast<int>(to_read));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu/%zu bytes: %s", bytes_written, total_bytes, esp_err_to_name(ret));
            break;
        }

        size_t written = fwrite(chunk_buffer, 1, to_read, file);
        if (written != to_read) {
            ESP_LOGE(TAG, "File write error at %zu/%zu bytes (written %zu)", bytes_written, total_bytes, written);
            ret = ESP_FAIL;
            break;
        }

        bytes_written += written;
    }

    fflush(file);
    fclose(file);
    free(chunk_buffer);

    TickType_t elapsed_ticks = xTaskGetTickCount() - start_ticks;
    uint32_t elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);
    ESP_LOGI(TAG, "Recording loop elapsed %u ms", static_cast<unsigned>(elapsed_ms));

    if (ret != ESP_OK) {
        return ret;
    }

    if (bytes_written == 0) {
        ESP_LOGE(TAG, "No audio captured, deleting %s", filepath);
        remove(filepath);
        return ESP_FAIL;
    }

    if (bytes_written < total_bytes) {
        ESP_LOGW(TAG, "Recording stopped early: saved %zu/%zu bytes", bytes_written, total_bytes);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Recording saved successfully: %s (%zu bytes)", filepath, bytes_written);
    return ESP_OK;
}

// ============================================================================
// 批处理录音 - record_all_channel_to_files
// ============================================================================

esp_err_t audio_recorder::record_all_channel_to_files(uint32_t record_duration_seconds,
                                                       const char* output_directory,
                                                       const char* file_prefix)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }

    if (record_duration_seconds == 0) {
        ESP_LOGE(TAG, "Record duration must be greater than zero");
        return ESP_ERR_INVALID_ARG;
    }

    if (!output_directory || output_directory[0] == '\0') {
        ESP_LOGE(TAG, "Output directory is empty");
        return ESP_ERR_INVALID_ARG;
    }

    const char* prefix_arg = (file_prefix && file_prefix[0] != '\0') ? file_prefix : "MIC";

    const uint8_t enabled_mask = static_cast<uint8_t>(parent_->mic_channels);
    if (enabled_mask == 0) {
        ESP_LOGE(TAG, "No microphone channels enabled");
        return ESP_ERR_INVALID_STATE;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(parent_->sample_rate);
    fs.bits_per_sample = static_cast<uint32_t>(parent_->bits_per_sample);

    const bool is_tdm_mode = parent_->es7210_use_tdm;
    if (is_tdm_mode) {
        fs.channel = parent_->rx_tdm_slot_count ? parent_->rx_tdm_slot_count : 4;
    } else {
        fs.channel = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    }

    if (fs.sample_rate == 0 || fs.bits_per_sample == 0 || fs.channel == 0) {
        ESP_LOGE(TAG, "Invalid audio format: sr=%u, bits=%u, ch=%u",
                 static_cast<unsigned>(fs.sample_rate),
                 static_cast<unsigned>(fs.bits_per_sample),
                 static_cast<unsigned>(fs.channel));
        return ESP_ERR_INVALID_STATE;
    }

    const size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    if (bytes_per_sample == 0) {
        ESP_LOGE(TAG, "bits_per_sample %u results in zero byte samples",
                 static_cast<unsigned>(fs.bits_per_sample));
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t buffer_size_64 = static_cast<uint64_t>(fs.sample_rate) * fs.channel * bytes_per_sample * record_duration_seconds;
    if (buffer_size_64 == 0 || buffer_size_64 > SIZE_MAX) {
        ESP_LOGE(TAG, "Requested recording size invalid or too large: %llu bytes",
                 static_cast<unsigned long long>(buffer_size_64));
        return ESP_ERR_INVALID_SIZE;
    }

    size_t buffer_size = static_cast<size_t>(buffer_size_64);

    ESP_LOGI(TAG, "=== Record All Channels (%lu s) ===", static_cast<unsigned long>(record_duration_seconds));
    ESP_LOGI(TAG, "Format: %u Hz, %u bits, %u channels (%s)",
             static_cast<unsigned>(fs.sample_rate),
             static_cast<unsigned>(fs.bits_per_sample),
             static_cast<unsigned>(fs.channel),
             is_tdm_mode ? "TDM" : "Standard");
    ESP_LOGI(TAG, "Buffer size: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);

    uint8_t* record_buffer = static_cast<uint8_t*>(malloc(buffer_size));
    if (!record_buffer) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate recording buffer (%zu bytes, free %zu bytes)", buffer_size, free_heap);
        return ESP_ERR_NO_MEM;
    }

    memset(record_buffer, 0, buffer_size);

    const size_t BLOCK_SIZE = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);
    TickType_t start_tick = xTaskGetTickCount();
    size_t bytes_read = 0;
    size_t last_progress = 0;
    esp_err_t read_result = ESP_OK;

    while (bytes_read < buffer_size) {
        size_t remaining = buffer_size - bytes_read;
        size_t to_read = (remaining > BLOCK_SIZE) ? BLOCK_SIZE : remaining;

        read_result = esp_codec_dev_read(parent_->record_dev, record_buffer + bytes_read, static_cast<int>(to_read));
        if (read_result != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu/%zu bytes: %s",
                     bytes_read, buffer_size, esp_err_to_name(read_result));
            break;
        }

        bytes_read += to_read;

        size_t progress = (bytes_read * 100) / buffer_size;
        if (progress >= last_progress + 25) {
            ESP_LOGI(TAG, "Recording progress: %zu%%", progress);
            last_progress = progress;
        }

        if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
            ESP_LOGW(TAG, "Recording timeout reached at %zu/%zu bytes", bytes_read, buffer_size);
            break;
        }
    }

    TickType_t end_tick = xTaskGetTickCount();
    uint32_t elapsed_ms = pdTICKS_TO_MS(end_tick - start_tick);

    if (read_result != ESP_OK && bytes_read == 0) {
        free(record_buffer);
        return read_result;
    }

    if (bytes_read == 0) {
        ESP_LOGE(TAG, "No audio captured");
        free(record_buffer);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Recording finished: %zu bytes in %.2f s", bytes_read, elapsed_ms / 1000.0f);

    channel_split_result_t split_result = split_recorded_channels(
        record_buffer,
        bytes_read,
        fs,
        is_tdm_mode,
        parent_->mic_channels);

    free(record_buffer);
    record_buffer = nullptr;

    if (split_result.status != ESP_OK) {
        ESP_LOGE(TAG, "Channel splitting failed: %s", esp_err_to_name(split_result.status));
        free_channel_split_result(split_result);
        return split_result.status;
    }

    mic_channel_quality_t channel_quality[4] = {};
    compute_split_channel_quality(split_result, channel_quality);

    ESP_LOGI(TAG, "=== Channel Quality Summary ===");
    for (int mic_index = 0; mic_index < 4; ++mic_index) {
        const mic_channel_quality_t& q = channel_quality[mic_index];
        if (!q.available) {
            continue;
        }

        ESP_LOGI(TAG, "  MIC%u -> samples:%zu rms:%.1f dB peak:[%d,%d] zero:%.1f%% clip:%.2f%%",
                 static_cast<unsigned>(mic_index + 1),
                 q.sample_count,
                 q.rms_db,
                 q.min_value,
                 q.max_value,
                 q.zero_percent,
                 q.clipped_percent);
    }

    std::string base_dir(output_directory);
    for (char& c : base_dir) {
        if (c == '\\') {
            c = '/';
        }
    }
    if (!base_dir.empty() && (base_dir.back() != '/' && base_dir.back() != '\\')) {
        base_dir.push_back('/');
    }

    std::string dir_probe = base_dir;
    if (!dir_probe.empty() && dir_probe.back() != '/') {
        dir_probe.push_back('/');
    }
    dir_probe += "probe.tmp";

    esp_err_t dir_status = ensure_parent_directories(dir_probe.c_str());
    if (dir_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare output directory %s", output_directory);
        free_channel_split_result(split_result);
        return dir_status;
    }

    // Remove probe filename, we just needed directories
    if (!dir_probe.empty()) {
        size_t pos = dir_probe.find_last_of('/');
        if (pos != std::string::npos) {
            base_dir = dir_probe.substr(0, pos + 1);
        }
    }

#if defined(CONFIG_FATFS_LFN_NONE)
    std::string prefix_str(prefix_arg);
    auto sanitize_token = [](const std::string& input) {
        std::string token;
        token.reserve(input.size());
        for (char c : input) {
            if (std::isalnum(static_cast<unsigned char>(c))) {
                token.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
            } else if (c == '_' || c == '-') {
                token.push_back('_');
            }
        }
        if (token.empty()) {
            token = "REC";
        }
        return token;
    };

    std::string token = sanitize_token(prefix_str);
    auto make_short_name = [&](const std::string& suffix) {
        std::string base = token;
        if (base.size() > 3) {
            base = base.substr(0, 3);
        }
        std::string result = base + suffix;
        if (result.size() > 8) {
            result = result.substr(0, 8);
        }
        return result;
    };
#else
    auto normalize_prefix = [](const char* raw) {
        std::string cleaned;
        for (const char* p = raw; p && *p; ++p) {
            unsigned char c = static_cast<unsigned char>(*p);
            if (std::isalnum(c)) {
                cleaned.push_back(static_cast<char>(c));
            } else if (*p == '_' || *p == '-') {
                cleaned.push_back('_');
            }
        }
        if (cleaned.empty()) {
            cleaned = "record";
        }
        return cleaned;
    };

    std::string token = normalize_prefix(prefix_arg);
#endif

    esp_err_t write_status = ESP_OK;
    size_t files_written = 0;

    for (int mic_index = 0; mic_index < 4 && write_status == ESP_OK; ++mic_index) {
        int16_t* samples = split_result.mic_buffers[mic_index];
        const mic_channel_quality_t& quality = channel_quality[mic_index];

        if (!samples || !quality.available || quality.sample_count == 0) {
            continue;
        }

        std::string file_path;
#if defined(CONFIG_FATFS_LFN_NONE)
        std::string suffix = "M" + std::to_string(mic_index + 1);
        file_path = base_dir + make_short_name(suffix) + ".PCM";
#else
        file_path = base_dir + token + "_mic_ch" + std::to_string(mic_index + 1) + ".pcm";
#endif

        write_status = ensure_parent_directories(file_path.c_str());
        if (write_status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create directories for %s", file_path.c_str());
            break;
        }

        FILE* file = fopen(file_path.c_str(), "wb");
        if (!file) {
            ESP_LOGE(TAG, "Failed to open %s: %s", file_path.c_str(), strerror(errno));
            write_status = ESP_FAIL;
            break;
        }

        size_t written = fwrite(samples, sizeof(int16_t), quality.sample_count, file);
        fflush(file);
        fclose(file);

        if (written != quality.sample_count) {
            ESP_LOGE(TAG, "Incomplete write for %s (%zu/%zu samples)",
                     file_path.c_str(), written, quality.sample_count);
            remove(file_path.c_str());
            write_status = ESP_FAIL;
            break;
        }

        ++files_written;
        float channel_duration = static_cast<float>(quality.sample_count) / static_cast<float>(fs.sample_rate);
        ESP_LOGI(TAG, "Saved MIC%u to %s (%.2f s, rms %.1f dB)",
                 static_cast<unsigned>(mic_index + 1), file_path.c_str(), channel_duration, quality.rms_db);
    }

    free_channel_split_result(split_result);

    if (write_status != ESP_OK) {
        return write_status;
    }

    if (files_written == 0) {
        ESP_LOGW(TAG, "No channel data available to write");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Recorded %zu channel files to %s", files_written, output_directory);
    return ESP_OK;
}

// ============================================================================
// 废弃测试函数 - record_test
// ============================================================================

esp_err_t audio_recorder::record_test(uint32_t record_duration_ms)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    const bool using_es8311_adc = parent_->es8311_initialized && parent_->es8311_has_adc_path() && (parent_->record_dev == parent_->es8311_dev_handle);
    const bool using_es7210_adc = parent_->es7210_initialized && !using_es8311_adc;

    if (!using_es8311_adc && !using_es7210_adc) {
        ESP_LOGE(TAG, "No active recording codec available (ES8311/ES7210)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting %lu ms record test using %s...",
             static_cast<unsigned long>(record_duration_ms),
             using_es8311_adc ? "ES8311 ADC" : "ES7210 ADC");

    // 分配录音缓冲区（动态参数）
    const size_t sr = (size_t)parent_->sample_rate;
    size_t channels = 0;
    uint8_t mic_mask = static_cast<uint8_t>(parent_->mic_channels);
    if (using_es8311_adc) {
        channels = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
        mic_mask = 0;
    } else {
        for (int i = 0; i < 4; ++i) {
            if (mic_mask & (1 << i)) {
                channels++;
            }
        }
        if (channels == 0) {
            channels = 1;
        }
    }
    const size_t bps = (size_t)parent_->bits_per_sample;
    const size_t bytes_per_sample = bps / 8;
    const size_t buffer_size = (sr * channels * bytes_per_sample * record_duration_ms) / 1000;
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate record buffer (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Record buffer allocated: %zu bytes", buffer_size);
    ESP_LOGI(TAG, "Sample rate: %zu Hz, Channels: %zu, Bits: %zu", sr, channels, bps);

    // 使用官方推荐的固定块大小读取方式
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    const size_t BLOCK_SIZE = 512; // 使用固定块大小
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_ms + 300); // 适度超时余量
    
    while (bytes_read < buffer_size) {
        // 计算本次读取的块大小
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(parent_->record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error: %s", esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 检查是否超时
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached (%lu ms), stop early",
                     static_cast<unsigned long>(record_duration_ms));
            break;
        }
    }
    TickType_t end_time = xTaskGetTickCount();
    
    uint32_t actual_duration = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Record completed successfully");
    ESP_LOGI(TAG, "Requested duration: %lu ms, Actual duration: %lu ms",
             static_cast<unsigned long>(record_duration_ms),
             static_cast<unsigned long>(actual_duration));
    ESP_LOGI(TAG, "Expected bytes: %zu, Actual bytes read: %zu (%.1f%%)", buffer_size, bytes_read, (buffer_size? (bytes_read * 100.0 / buffer_size):0.0));
    
    // 简单的音频数据分析
    if (bytes_read > 0) {
    int16_t *samples = (int16_t *)record_buffer;
    size_t total_samples = bytes_read / sizeof(int16_t); // 所有通道样本点
    size_t frame_count = (channels > 0) ? total_samples / channels : total_samples;
        
        // 计算音频信号的基本统计信息
        int32_t sum = 0;
        int16_t min_val = INT16_MAX;
        int16_t max_val = INT16_MIN;
        
        for (size_t i = 0; i < total_samples; i++) {
            int16_t sample = samples[i];
            sum += abs(sample);
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        int32_t avg_amplitude = (total_samples > 0) ? (sum / (int32_t)total_samples) : 0;
        ESP_LOGI(TAG, "Audio analysis - Frames: %zu, Total samples: %zu, Avg amplitude: %ld, Range: [%d, %d]", 
                 frame_count, total_samples, avg_amplitude, min_val, max_val);
        
        // 检查是否捕获到有效音频信号
        if (avg_amplitude > 100) {
            ESP_LOGI(TAG, "Valid audio signal detected!");
        } else {
            ESP_LOGW(TAG, "Low audio signal detected - check microphone connection");
        }
    }

    // 清理资源
    free(record_buffer);
    
    ESP_LOGI(TAG, "Record test completed successfully");
    return ESP_OK;
}

// ============================================================================
// 废弃测试函数 - record_and_play_test
// ============================================================================

esp_err_t audio_recorder::record_and_play_test(uint32_t record_duration_seconds)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    const bool capture_ready = (parent_->es7210_initialized && parent_->record_dev != nullptr) ||
                               (parent_->es8311_initialized && parent_->es8311_has_adc_path() && parent_->record_dev == parent_->es8311_dev_handle);
    const bool playback_ready = parent_->es8311_initialized && parent_->es8311_has_dac_path() && (parent_->play_dev != nullptr);

    if (!capture_ready || !playback_ready) {
        ESP_LOGE(TAG, "Audio pipeline not ready (capture=%s, playback=%s)",
                 capture_ready ? "OK" : "NO",
                 playback_ready ? "OK" : "NO");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Record and Play Test (Single Shot) ===");
    ESP_LOGI(TAG, "Duration: %lu seconds", static_cast<unsigned long>(record_duration_seconds));
    
    // 获取当前音频格式信息（使用录音设备的声道配置）
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)parent_->sample_rate;
    fs.channel = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    fs.bits_per_sample = (uint32_t)parent_->bits_per_sample;
    
    ESP_LOGI(TAG, "Audio format: %u Hz, %u channels, %u bits",
             static_cast<unsigned>(fs.sample_rate),
             static_cast<unsigned>(fs.channel),
             static_cast<unsigned>(fs.bits_per_sample));
    
    // 计算缓冲区大小
    size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    size_t buffer_size = (size_t)fs.sample_rate * fs.channel * bytes_per_sample * record_duration_seconds;
    
    ESP_LOGI(TAG, "Allocating buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate memory for recording");
        ESP_LOGE(TAG, "Required: %zu bytes (%.2f KB), Free: %zu bytes (%.2f KB)",
                buffer_size, buffer_size / 1024.0f, free_heap, free_heap / 1024.0f);
        return ESP_ERR_NO_MEM;
    }
    
    memset(record_buffer, 0, buffer_size);
    
    // ========== Phase 1: Recording ==========
    ESP_LOGI(TAG, "Phase 1: Recording %lu seconds...", static_cast<unsigned long>(record_duration_seconds));
    
    const size_t BLOCK_SIZE = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    
    while (bytes_read < buffer_size) {
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(parent_->record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu bytes: %s", bytes_read, esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 进度提示 (每25%打印一次)
        static size_t last_progress = 0;
        size_t progress = (bytes_read * 100) / buffer_size;
        if (progress >= last_progress + 25) {
            ESP_LOGI(TAG, "Recording progress: %zu%%", progress);
            last_progress = progress;
        }
        
        // 超时检查
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached, stopping early");
            break;
        }
    }
    
    TickType_t end_time = xTaskGetTickCount();
    uint32_t actual_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Recording completed!");
    ESP_LOGI(TAG, "Requested: %lu sec, Actual: %.2f sec",
             static_cast<unsigned long>(record_duration_seconds),
             actual_duration_ms / 1000.0f);
    ESP_LOGI(TAG, "Bytes read: %zu / %zu (%.1f%%)",
             bytes_read, buffer_size, (buffer_size > 0) ? (bytes_read * 100.0f / buffer_size) : 0.0f);
    
    // 简单音频分析
    if (bytes_read > 0) {
        int16_t *samples = (int16_t *)record_buffer;
        size_t sample_count = bytes_read / sizeof(int16_t);
        
        int32_t sum = 0;
        int16_t min_val = INT16_MAX;
        int16_t max_val = INT16_MIN;
        
        for (size_t i = 0; i < sample_count; i++) {
            int16_t sample = samples[i];
            sum += abs(sample);
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        
        int32_t avg_amplitude = (sample_count > 0) ? (sum / (int32_t)sample_count) : 0;
        
        ESP_LOGI(TAG, "Audio analysis - Samples: %zu, Avg amplitude: %ld, Range: [%d, %d]",
                 sample_count, avg_amplitude, min_val, max_val);
        
        if (avg_amplitude > 100) {
            ESP_LOGI(TAG, "Valid audio signal detected [OK]");
        } else {
            ESP_LOGW(TAG, "Low audio signal - check microphone");
        }
    }
    
    // 录音和播放之间短暂延迟
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // ========== Phase 2: Playback ==========
    ESP_LOGI(TAG, "Phase 2: Playing recorded audio...");
    
    start_time = xTaskGetTickCount();
    
    // 使用 parent_->get_playback()->play_audio_buffer 播放录音
    esp_err_t ret = parent_->get_playback()->play_audio_buffer(
        record_buffer,
        bytes_read,
        fs.sample_rate,
        (fs.channel == 1) ? AUDIO_CHANNELS_MONO : AUDIO_CHANNELS_STEREO,
        (i2s_data_bit_width_t)fs.bits_per_sample,
        AUDIO_PLAYBACK_BLOCKING
    );
    
    end_time = xTaskGetTickCount();
    uint32_t playback_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        free(record_buffer);
        return ret;
    }
    
    ESP_LOGI(TAG, "Playback completed!");
    ESP_LOGI(TAG, "Duration: %.2f sec, Bytes played: %zu", 
             playback_duration_ms / 1000.0f, bytes_read);
    
    // ========== Phase 3: 生成详细统计报告 ==========
    ESP_LOGI(TAG, "=== Analysis Results ===");
    
    if (bytes_read > 0 && fs.channel > 0) {
        int16_t *samples = (int16_t *)record_buffer;
        size_t total_samples = bytes_read / sizeof(int16_t);
        size_t frames = total_samples / fs.channel;
        
        // 为每个通道统计信息
        for (uint32_t ch = 0; ch < fs.channel; ch++) {
            // 通道标识
            const char* ch_name = (ch == 0) ? "Microphone" : "Loopback";
            ESP_LOGI(TAG, "=== CH%u (%s) Statistics ===", static_cast<unsigned>(ch + 1), ch_name);
            
            // 统计变量
            int64_t sum_squares = 0;  // 用于计算RMS
            int16_t min_val = INT16_MAX;
            int16_t max_val = INT16_MIN;
            size_t zero_count = 0;
            size_t clipped_count = 0;
            
            // 遍历该通道的所有样本
            for (size_t frame = 0; frame < frames; frame++) {
                size_t idx = frame * fs.channel + ch;
                if (idx >= total_samples) break;
                
                int16_t sample = samples[idx];
                
                // RMS 计算
                int32_t sample_32 = (int32_t)sample;
                sum_squares += (int64_t)(sample_32 * sample_32);
                
                // 峰值检测
                if (sample < min_val) min_val = sample;
                if (sample > max_val) max_val = sample;
                
                // 零样本计数 (绝对值小于等于 1)
                if (sample >= -1 && sample <= 1) {
                    zero_count++;
                }
                
                // 削波检测 (接近最大值)
                if (sample <= -32767 || sample >= 32767) {
                    clipped_count++;
                }
            }
            
            // 计算 RMS Level (dB)
            double rms_linear = 0.0;
            if (frames > 0) {
                double mean_square = (double)sum_squares / (double)frames;
                rms_linear = sqrt(mean_square);
            }
            // 转换为 dB (参考值为 32768)
            double rms_db = (rms_linear > 0) ? (20.0 * log10(rms_linear / 32768.0)) : -96.0;
            
            // 计算百分比
            double zero_percent = (frames > 0) ? (zero_count * 100.0 / frames) : 0.0;
            double clipped_percent = (frames > 0) ? (clipped_count * 100.0 / frames) : 0.0;
            
            // 打印统计结果 (格式类似图片)
            ESP_LOGI(TAG, "  RMS Level: %.1f dB", rms_db);
            ESP_LOGI(TAG, "  Peak: %d to %d", min_val, max_val);
            ESP_LOGI(TAG, "  Zero samples: %zu (%.0f%%)", zero_count, zero_percent);
            ESP_LOGI(TAG, "  Clipped samples: %zu (%.2f%%)", clipped_count, clipped_percent);
        }
    }
    
    ESP_LOGI(TAG, "=== Analysis Results ===");
    
    // 清理资源
    free(record_buffer);
    
    // 清空播放管线，避免残留音频
    esp_err_t clr_ret = parent_->get_playback()->clear_audio_pipeline(120);
    if (clr_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear playback pipeline: %s", esp_err_to_name(clr_ret));
    }
    
    ESP_LOGI(TAG, "=== Record and Play Test Completed ===");
    return ESP_OK;
}

// ============================================================================
// 废弃测试函数 - record_and_play_test_with_channel_select
// ============================================================================

esp_err_t audio_recorder::record_and_play_test_with_channel_select(uint32_t record_duration_seconds, audio_mic_channel_t target_mic_channel, bool analysis_only)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    // 仅分析模式只需要ES7210初始化
    if (!parent_->es7210_initialized) {
        ESP_LOGE(TAG, "ES7210 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 正常播放模式需要两个设备都初始化
    if (!analysis_only && !parent_->es8311_initialized) {
        ESP_LOGE(TAG, "ES8311 not initialized (required for playback mode)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->record_dev) {
        ESP_LOGE(TAG, "Record device not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!analysis_only && !parent_->play_dev) {
        ESP_LOGE(TAG, "Playback device not available (required for playback mode)");
        return ESP_ERR_INVALID_STATE;
    }

    // 验证目标麦克风通道是否为单一通道（不能是组合通道）
    uint8_t target_mic_value = (uint8_t)target_mic_channel;
    bool is_single_channel = (target_mic_value == 0x01 || target_mic_value == 0x02 || 
                              target_mic_value == 0x04 || target_mic_value == 0x08);
    
    if (!is_single_channel) {
        ESP_LOGE(TAG, "target_mic_channel must be a single microphone (AUDIO_MIC_CHANNEL_1/2/3/4), got: 0x%02X", 
                 static_cast<unsigned>(target_mic_value));
        return ESP_ERR_INVALID_ARG;
    }

    // 将 audio_mic_channel_t 转换为索引 (0-3)
    uint8_t target_channel = 0;
    switch (target_mic_channel) {
        case AUDIO_MIC_CHANNEL_1: target_channel = 0; break;
        case AUDIO_MIC_CHANNEL_2: target_channel = 1; break;
        case AUDIO_MIC_CHANNEL_3: target_channel = 2; break;
        case AUDIO_MIC_CHANNEL_4: target_channel = 3; break;
        default:
            ESP_LOGE(TAG, "Invalid target_mic_channel: 0x%02X", static_cast<unsigned>(target_mic_value));
            return ESP_ERR_INVALID_ARG;
    }

    // 检查目标通道是否已启用
    if (!(parent_->mic_channels & target_mic_value)) {
        ESP_LOGE(TAG, "Target MIC%u (0x%02X) is not enabled in current config (0x%02X)", 
                 static_cast<unsigned>(target_channel + 1),
                 static_cast<unsigned>(target_mic_value),
                 static_cast<unsigned>(parent_->mic_channels));
        ESP_LOGE(TAG, "Please initialize ES7210 with the target microphone enabled");
        return ESP_ERR_INVALID_ARG;
    }

    // 计算启用的麦克风数量
    uint8_t mic_count = 0;
    for (int i = 0; i < 4; i++) {
        if (parent_->mic_channels & (1 << i)) {
            mic_count++;
        }
    }

    // 直接参考ES7210的TDM状态
    bool is_tdm_mode = parent_->es7210_use_tdm;
    
    ESP_LOGI(TAG, "=== Record and %s Test (Channel Select) ===", analysis_only ? "Analysis" : "Play");
    ESP_LOGI(TAG, "Duration: %lu seconds, Target: MIC%u (0x%02X)", 
             static_cast<unsigned long>(record_duration_seconds),
             static_cast<unsigned>(target_channel + 1),
             static_cast<unsigned>(target_mic_value));
    ESP_LOGI(TAG, "Mode: %s (%u mics enabled: 0x%02X), Analysis only: %s", 
             is_tdm_mode ? "TDM" : "Standard I2S",
             static_cast<unsigned>(mic_count),
             static_cast<unsigned>(parent_->mic_channels),
             analysis_only ? "YES" : "NO");
    
    // 获取当前音频格式信息
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = (uint32_t)parent_->sample_rate;
    fs.bits_per_sample = (uint32_t)parent_->bits_per_sample;
    
    // 确定I2S通道数配置
    if (is_tdm_mode) {
        fs.channel = parent_->rx_tdm_slot_count ? parent_->rx_tdm_slot_count : 4;  // TDM使用实际配置的slot数量（默认4）
    } else {
        fs.channel = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    }
    
    size_t bytes_per_sample = (fs.bits_per_sample >> 3);
    
    ESP_LOGI(TAG, "Audio format: %u Hz, %u channels, %u bits per sample",
             static_cast<unsigned>(fs.sample_rate),
             static_cast<unsigned>(fs.channel),
             static_cast<unsigned>(fs.bits_per_sample));
    
    // 计算录音缓冲区大小
    size_t buffer_size = (size_t)fs.sample_rate * fs.channel * bytes_per_sample * record_duration_seconds;
    
    if (is_tdm_mode) {
        ESP_LOGI(TAG, "TDM buffer calculation:");
        ESP_LOGI(TAG, "  - Total buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
        ESP_LOGI(TAG, "  - TDM slots: %u", static_cast<unsigned>(fs.channel));
        ESP_LOGI(TAG, "  - Active mics: %u (0x%02X)",
                 static_cast<unsigned>(mic_count),
                 static_cast<unsigned>(parent_->mic_channels));
        ESP_LOGI(TAG, "  - Data rate: %.2f KB/s", 
                 (float)(fs.sample_rate * fs.channel * bytes_per_sample) / 1024.0f);
    } else {
        ESP_LOGI(TAG, "Standard I2S buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    }
    
    ESP_LOGI(TAG, "Allocating buffer: %zu bytes (%.2f KB)", buffer_size, buffer_size / 1024.0f);
    
    uint8_t *record_buffer = (uint8_t *)malloc(buffer_size);
    if (!record_buffer) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate memory for recording");
        ESP_LOGE(TAG, "Required: %zu bytes (%.2f KB), Free: %zu bytes (%.2f KB)",
                buffer_size, buffer_size / 1024.0f, free_heap, free_heap / 1024.0f);
        return ESP_ERR_NO_MEM;
    }
    
    memset(record_buffer, 0, buffer_size);
    
    // ========== Phase 1: Recording ==========
    ESP_LOGI(TAG, "Phase 1: Recording all TDM channels...");
    
    const size_t BLOCK_SIZE = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);
    TickType_t start_time = xTaskGetTickCount();
    size_t bytes_read = 0;
    
    while (bytes_read < buffer_size) {
        size_t read_size = (buffer_size - bytes_read > BLOCK_SIZE) ? BLOCK_SIZE : (buffer_size - bytes_read);
        
        esp_err_t ret = esp_codec_dev_read(parent_->record_dev, record_buffer + bytes_read, (int)read_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record read error at %zu bytes: %s", bytes_read, esp_err_to_name(ret));
            free(record_buffer);
            return ESP_FAIL;
        }
        
        bytes_read += read_size;
        
        // 进度提示
        static size_t last_progress = 0;
        size_t progress = (bytes_read * 100) / buffer_size;
        if (progress >= last_progress + 25) {
            ESP_LOGI(TAG, "Recording progress: %zu%%", progress);
            last_progress = progress;
        }
        
        // 超时检查
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Record timeout reached, stopping early");
            break;
        }
    }
    
    TickType_t end_time = xTaskGetTickCount();
    uint32_t actual_duration_ms = pdTICKS_TO_MS(end_time - start_time);
    
    ESP_LOGI(TAG, "Recording completed! Actual: %.2f sec, Bytes: %zu",
             actual_duration_ms / 1000.0f, bytes_read);
    
    // ========== Phase 2: Channel Extraction ==========
    ESP_LOGI(TAG, "Phase 2: Splitting MIC data from %s stream...",
             is_tdm_mode ? "TDM" : "Standard I2S");

    channel_split_result_t split_result = split_recorded_channels(
        record_buffer,
        bytes_read,
        fs,
        is_tdm_mode,
        parent_->mic_channels);

    free(record_buffer);
    record_buffer = nullptr;

    if (split_result.status != ESP_OK) {
        ESP_LOGE(TAG, "Channel splitting failed: %s", esp_err_to_name(split_result.status));
        free_channel_split_result(split_result);
        return split_result.status;
    }

    int16_t* target_samples = split_result.mic_buffers[target_channel];
    if (!target_samples) {
        ESP_LOGE(TAG, "Target MIC%u buffer unavailable after splitting",
                 static_cast<unsigned>(target_channel + 1));
        free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    mic_channel_quality_t channel_quality[4] = {};
    compute_split_channel_quality(split_result, channel_quality);
    const mic_channel_quality_t& target_quality = channel_quality[target_channel];
    const size_t extracted_samples = target_quality.sample_count;
    const float target_duration_sec = extracted_samples
                                          ? (static_cast<float>(extracted_samples) / static_cast<float>(fs.sample_rate))
                                          : 0.0f;

    ESP_LOGI(TAG, "Extracted %zu samples from MIC%u (%.2f seconds)",
             extracted_samples,
             static_cast<unsigned>(target_channel + 1),
             target_duration_sec);

    ESP_LOGI(TAG, "=== MIC%u Audio Analysis Report ===", static_cast<unsigned>(target_channel + 1));
    ESP_LOGI(TAG, "  Total samples: %zu (%.2f seconds)", extracted_samples, target_duration_sec);
    ESP_LOGI(TAG, "  Average amplitude: %ld", static_cast<long>(target_quality.average_abs_amplitude));
    ESP_LOGI(TAG, "  Peak range: [%d, %d]", target_quality.min_value, target_quality.max_value);
    ESP_LOGI(TAG, "  RMS Level: %.1f dB", target_quality.rms_db);
    ESP_LOGI(TAG, "  Signal quality: %s", target_quality.average_abs_amplitude > 100 ? "GOOD" : "LOW");

    if (target_quality.average_abs_amplitude > 100) {
        ESP_LOGI(TAG, "Valid audio signal detected from MIC%u", static_cast<unsigned>(target_channel + 1));
    } else {
        ESP_LOGW(TAG, "Low audio signal from MIC%u - check microphone connection/gain",
                 static_cast<unsigned>(target_channel + 1));
    }
    ESP_LOGI(TAG, "====================================");

    ESP_LOGI(TAG, "=== All Channel Quality Summary ===");
    for (int mic_index = 0; mic_index < 4; ++mic_index) {
        const mic_channel_quality_t& mic_quality = channel_quality[mic_index];
        if (!mic_quality.available) {
            continue;
        }

        const char* marker = (mic_index == target_channel) ? " (target)" : "";
        ESP_LOGI(TAG, "  MIC%u%s -> avg:%ld peak:[%d,%d] rms:%.1f dB signal:%s",
                 static_cast<unsigned>(mic_index + 1),
                 marker,
                 static_cast<long>(mic_quality.average_abs_amplitude),
                 mic_quality.min_value,
                 mic_quality.max_value,
                 mic_quality.rms_db,
                 mic_quality.average_abs_amplitude > 100 ? "GOOD" : "LOW");
    }
    ESP_LOGI(TAG, "====================================");

    if (analysis_only) {
        ESP_LOGI(TAG, "Analysis-only mode: Skipping playback phase");
        free_channel_split_result(split_result);
        ESP_LOGI(TAG, "=== Channel Analysis Completed ===");
        return ESP_OK;
    }

    // 短暂延迟
    vTaskDelay(pdMS_TO_TICKS(500));

    // ========== Phase 3: Playback ==========
    ESP_LOGI(TAG, "Phase 3: Playing extracted MIC%u audio...", static_cast<unsigned>(target_channel + 1));

    start_time = xTaskGetTickCount();

    const size_t bytes_to_write = extracted_samples * sizeof(int16_t);
    esp_err_t ret = parent_->get_playback()->play_audio_buffer(
        reinterpret_cast<const uint8_t*>(target_samples),
        bytes_to_write,
        static_cast<uint32_t>(fs.sample_rate),
        AUDIO_CHANNELS_MONO,
        I2S_DATA_BIT_WIDTH_16BIT,
        AUDIO_PLAYBACK_BLOCKING);

    end_time = xTaskGetTickCount();
    uint32_t playback_duration_ms = pdTICKS_TO_MS(end_time - start_time);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        free_channel_split_result(split_result);
        return ret;
    }

    ESP_LOGI(TAG, "Playback completed! Duration: %.2f sec, Bytes: %zu",
             playback_duration_ms / 1000.0f, bytes_to_write);

    free_channel_split_result(split_result);

    // 清空播放管线
    esp_err_t clr_ret = parent_->get_playback()->clear_audio_pipeline(120);
    if (clr_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear playback pipeline: %s", esp_err_to_name(clr_ret));
    }

    ESP_LOGI(TAG, "=== Channel Select Test Completed ===");
    return ESP_OK;
}

// ============================================================================
// 废弃测试函数 - record_and_playback_test
// ============================================================================

esp_err_t audio_recorder::record_and_playback_test(uint32_t record_duration_seconds,
                                                    bool loop_playback,
                                                    audio_mic_channel_t target_mic_channel)
{
    // S6: AEC 流式会话互斥保护
    if (parent_->sr_afe_ && parent_->sr_afe_->aec_session_is_running()) {
        ESP_LOGE(TAG, "Cannot record while AEC streaming session is active");
        return ESP_ERR_INVALID_STATE;
    }

    const bool capture_ready = (parent_->es7210_initialized && parent_->record_dev != nullptr) ||
                               (parent_->es8311_initialized && parent_->es8311_has_adc_path() && parent_->record_dev == parent_->es8311_dev_handle);
    const bool playback_ready = parent_->es8311_initialized && parent_->es8311_has_dac_path() && (parent_->play_dev != nullptr);

    if (!capture_ready || !playback_ready) {
        ESP_LOGE(TAG, "Audio devices not initialized properly (capture=%s, playback=%s)",
                 capture_ready ? "OK" : "NO",
                 playback_ready ? "OK" : "NO");
        return ESP_ERR_INVALID_STATE;
    }

    if (record_duration_seconds == 0) {
        ESP_LOGE(TAG, "Record duration must be greater than zero");
        return ESP_ERR_INVALID_ARG;
    }

    const bool is_tdm_mode = parent_->es7210_use_tdm;
    const audio_mic_channel_t enabled_mics = parent_->mic_channels;

    auto is_single_mic = [](audio_mic_channel_t mic) {
        uint8_t value = static_cast<uint8_t>(mic);
        return value == 0x01 || value == 0x02 || value == 0x04 || value == 0x08;
    };

    uint8_t selected_channel_index = 0;
    audio_mic_channel_t effective_target = target_mic_channel;

    if (is_tdm_mode) {
        if (enabled_mics == AUDIO_MIC_NONE) {
            ESP_LOGE(TAG, "No microphone channels enabled for TDM mode");
            return ESP_ERR_INVALID_STATE;
        }

        if (effective_target == AUDIO_MIC_NONE) {
            bool found = false;
            for (uint8_t idx = 0; idx < 4; ++idx) {
                if (static_cast<uint8_t>(enabled_mics) & (1u << idx)) {
                    selected_channel_index = idx;
                    effective_target = static_cast<audio_mic_channel_t>(1u << idx);
                    found = true;
                    break;
                }
            }

            if (!found) {
                ESP_LOGE(TAG, "Failed to auto-select TDM microphone channel (mask=0x%02X)",
                         static_cast<unsigned>(enabled_mics));
                return ESP_ERR_INVALID_STATE;
            }

            ESP_LOGI(TAG, "TDM auto-selected MIC%u for playback",
                     static_cast<unsigned>(selected_channel_index + 1));
        } else {
            if (!is_single_mic(effective_target)) {
                ESP_LOGE(TAG, "TDM mode requires single mic selection, got 0x%02X",
                         static_cast<unsigned>(effective_target));
                return ESP_ERR_INVALID_ARG;
            }

            uint8_t mask = static_cast<uint8_t>(effective_target);
            if (!(mask & static_cast<uint8_t>(enabled_mics))) {
                ESP_LOGE(TAG, "Selected MIC (0x%02X) not enabled (mask=0x%02X)",
                         static_cast<unsigned>(mask),
                         static_cast<unsigned>(enabled_mics));
                return ESP_ERR_INVALID_ARG;
            }

            switch (effective_target) {
                case AUDIO_MIC_CHANNEL_1:
                    selected_channel_index = 0;
                    break;
                case AUDIO_MIC_CHANNEL_2:
                    selected_channel_index = 1;
                    break;
                case AUDIO_MIC_CHANNEL_3:
                    selected_channel_index = 2;
                    break;
                case AUDIO_MIC_CHANNEL_4:
                    selected_channel_index = 3;
                    break;
                default:
                    ESP_LOGE(TAG, "Unsupported mic channel selection 0x%02X",
                             static_cast<unsigned>(effective_target));
                    return ESP_ERR_INVALID_ARG;
            }
        }
    } else if (target_mic_channel != AUDIO_MIC_NONE) {
        ESP_LOGW(TAG, "Standard I2S mode ignores target_mic_channel (0x%02X)",
                 static_cast<unsigned>(target_mic_channel));
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(parent_->sample_rate);
    fs.bits_per_sample = static_cast<uint32_t>(parent_->bits_per_sample);

    if (fs.sample_rate == 0 || fs.bits_per_sample == 0) {
        ESP_LOGE(TAG, "Invalid audio format configuration (sr=%u, bits=%u)",
                 static_cast<unsigned>(fs.sample_rate),
                 static_cast<unsigned>(fs.bits_per_sample));
        return ESP_ERR_INVALID_STATE;
    }

    if (is_tdm_mode) {
        fs.channel = parent_->rx_tdm_slot_count ? parent_->rx_tdm_slot_count : 4;
    } else {
        fs.channel = (parent_->rx_channels == AUDIO_CHANNELS_MONO) ? 1 : 2;
    }

    if (fs.channel == 0) {
        ESP_LOGE(TAG, "Invalid channel configuration (mode=%s)", is_tdm_mode ? "TDM" : "STD");
        return ESP_ERR_INVALID_STATE;
    }

    const size_t bytes_per_sample = fs.bits_per_sample >> 3;
    if (bytes_per_sample == 0) {
        ESP_LOGE(TAG, "bits_per_sample %u produces zero-sized samples",
                 static_cast<unsigned>(fs.bits_per_sample));
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t bytes_needed = static_cast<uint64_t>(fs.sample_rate) * fs.channel * bytes_per_sample * record_duration_seconds;
    if (bytes_needed == 0 || bytes_needed > SIZE_MAX) {
        ESP_LOGE(TAG, "Requested recording buffer too large: %llu bytes",
                 static_cast<unsigned long long>(bytes_needed));
        return ESP_ERR_INVALID_SIZE;
    }

    size_t record_bytes = static_cast<size_t>(bytes_needed);
    uint8_t* data = static_cast<uint8_t*>(malloc(record_bytes));
    if (!data) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGE(TAG, "Failed to allocate recording buffer (%zu bytes, free %zu bytes)",
                 record_bytes,
                 free_heap);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "=== Record-playback test (%lu s, loop: %s, mode: %s) ===",
             static_cast<unsigned long>(record_duration_seconds),
             loop_playback ? "YES" : "NO",
             is_tdm_mode ? "TDM" : "STD");

    if (is_tdm_mode) {
        ESP_LOGI(TAG, "Enabled MIC mask: 0x%02X, target MIC%u",
                 static_cast<unsigned>(enabled_mics),
                 static_cast<unsigned>(selected_channel_index + 1));
    }

    const size_t block_size = 512;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(record_duration_seconds * 1000 + 500);

    auto run_cycle = [&](int cycle_index) -> esp_err_t {
        memset(data, 0, record_bytes);

        ESP_LOGI(TAG, "Cycle #%d: Recording %lu seconds... (buffer %zu bytes)",
                 cycle_index,
                 static_cast<unsigned long>(record_duration_seconds),
                 record_bytes);

        TickType_t start_tick = xTaskGetTickCount();
        size_t bytes_read = 0;

        while (bytes_read < record_bytes) {
            size_t to_read = (record_bytes - bytes_read > block_size) ? block_size : (record_bytes - bytes_read);
            esp_err_t ret = esp_codec_dev_read(parent_->record_dev, data + bytes_read, static_cast<int>(to_read));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Cycle #%d: Record read failed at %zu bytes: %s",
                         cycle_index,
                         bytes_read,
                         esp_err_to_name(ret));
                return ret;
            }

            bytes_read += to_read;

            if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
                ESP_LOGW(TAG, "Cycle #%d: Record timeout reached at %zu/%zu bytes",
                         cycle_index,
                         bytes_read,
                         record_bytes);
                break;
            }
        }

        if (bytes_read == 0) {
            ESP_LOGE(TAG, "Cycle #%d: No audio captured", cycle_index);
            return ESP_ERR_INVALID_STATE;
        }

        float approx_duration = static_cast<float>(bytes_read) /
                                static_cast<float>(fs.sample_rate * fs.channel * bytes_per_sample);
        ESP_LOGI(TAG, "Cycle #%d: Recording completed (bytes=%zu, approx %.2f s)",
                 cycle_index,
                 bytes_read,
                 approx_duration);

        vTaskDelay(pdMS_TO_TICKS(200));

        esp_err_t playback_status = ESP_OK;

        if (is_tdm_mode) {
            channel_split_result_t split_result = split_recorded_channels(
                data,
                bytes_read,
                fs,
                true,
                enabled_mics);

            if (split_result.status != ESP_OK) {
                ESP_LOGE(TAG, "Cycle #%d: Channel split failed: %s",
                         cycle_index,
                         esp_err_to_name(split_result.status));
                free_channel_split_result(split_result);
                return split_result.status;
            }

            mic_channel_quality_t quality[4] = {};
            compute_split_channel_quality(split_result, quality);
            const mic_channel_quality_t& target_quality = quality[selected_channel_index];

            if (!target_quality.available || target_quality.sample_count == 0) {
                ESP_LOGE(TAG, "Cycle #%d: Target MIC%u has no samples",
                          cycle_index,
                          static_cast<unsigned>(selected_channel_index + 1));
                free_channel_split_result(split_result);
                return ESP_ERR_INVALID_STATE;
            }

            ESP_LOGI(TAG, "Cycle #%d: MIC%u stats -> samples:%zu rms:%.1f dB peak:[%d,%d]",
                      cycle_index,
                      static_cast<unsigned>(selected_channel_index + 1),
                      target_quality.sample_count,
                     target_quality.rms_db,
                     target_quality.min_value,
                     target_quality.max_value);

            playback_status = parent_->get_playback()->play_audio_buffer(
                reinterpret_cast<const uint8_t*>(split_result.mic_buffers[selected_channel_index]),
                target_quality.sample_count * sizeof(int16_t),
                fs.sample_rate,
                AUDIO_CHANNELS_MONO,
                I2S_DATA_BIT_WIDTH_16BIT,
                AUDIO_PLAYBACK_BLOCKING);

            free_channel_split_result(split_result);
        } else {
            playback_status = parent_->get_playback()->play_audio_buffer(
                data,
                bytes_read,
                fs.sample_rate,
                (fs.channel == 1) ? AUDIO_CHANNELS_MONO : AUDIO_CHANNELS_STEREO,
                static_cast<i2s_data_bit_width_t>(fs.bits_per_sample),
                AUDIO_PLAYBACK_BLOCKING);
        }

        if (playback_status != ESP_OK) {
            ESP_LOGE(TAG, "Cycle #%d: Playback failed: %s",
                     cycle_index,
                     esp_err_to_name(playback_status));
            return playback_status;
        }

        ESP_LOGI(TAG, "Cycle #%d: Playback completed", cycle_index);

        esp_err_t clear_ret = parent_->get_playback()->clear_audio_pipeline(is_tdm_mode ? 120 : (loop_playback ? 50 : 80));
        if (clear_ret != ESP_OK) {
            ESP_LOGW(TAG, "Cycle #%d: Failed to clear audio pipeline: %s",
                     cycle_index,
                     esp_err_to_name(clear_ret));
        }

        return ESP_OK;
    };

    int cycle_index = 0;
    esp_err_t overall_status = ESP_OK;

    do {
        cycle_index++;
        ESP_LOGI(TAG, "=== Cycle #%d START ===", cycle_index);

        overall_status = run_cycle(cycle_index);
        if (overall_status != ESP_OK) {
            break;
        }

        ESP_LOGI(TAG, "=== Cycle #%d COMPLETED ===", cycle_index);

        if (loop_playback) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } while (loop_playback);

    free(data);

    if (overall_status == ESP_OK) {
        ESP_LOGI(TAG, "=== Record and playback test completed ===");
    }

    return overall_status;
}

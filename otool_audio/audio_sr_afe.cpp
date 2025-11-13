/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_sr_afe.h"
#include "audio_tools.h"
#include "audio_remix_tools.h"
#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_TASK_WDT
#include "esp_task_wdt.h"
#endif

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cctype>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

static const char *TAG_AEC = "audio_sr_afe";

struct audio_sr_afe::aec_capture_output {
    channel_split_result_t split_result{};
    int16_t* mic_data = nullptr;
    int16_t* ref_data = nullptr;
    int16_t* aec_output = nullptr;
    size_t sample_count = 0;
    esp_codec_dev_sample_info_t fs{};
    bool reference_is_silent = true;
};

namespace {

#if CONFIG_TASK_WDT
class TaskWdtGuard {
public:
    TaskWdtGuard()
    {
        esp_err_t res = esp_task_wdt_add(nullptr);
        if (res == ESP_OK) {
            added_here_ = true;
            can_reset_ = true;
        } else if (res == ESP_ERR_INVALID_STATE) {
            // 当前任务可能已经注册，允许复位
            can_reset_ = true;
        } else {
            ESP_LOGW(TAG_AEC, "Failed to add current task to WDT: %s", esp_err_to_name(res));
        }
    }

    ~TaskWdtGuard()
    {
        if (added_here_) {
            esp_err_t res = esp_task_wdt_delete(nullptr);
            if (res != ESP_OK) {
                ESP_LOGW(TAG_AEC, "Failed to remove task from WDT: %s", esp_err_to_name(res));
            }
        }
    }

    bool can_reset() const { return can_reset_; }

private:
    bool added_here_ = false;
    bool can_reset_ = false;
};
#endif

/**
 * @brief Ensure directory exists, creating intermediate folders if needed.
 */
bool ensure_directory_exists(const std::string& original_path)
{
    if (original_path.empty()) {
        return false;
    }

    std::string path = original_path;
    std::replace(path.begin(), path.end(), '\\', '/');

    // Trim trailing slash (except root)
    while (path.size() > 1 && path.back() == '/') {
        path.pop_back();
    }

    size_t start = 0;
    std::string current;
    if (!path.empty() && path[0] == '/') {
        current = "/";
        start = 1;
    }

    while (start <= path.size()) {
        size_t next = path.find('/', start);
        std::string segment = (next == std::string::npos)
                                ? path.substr(start)
                                : path.substr(start, next - start);

        if (!segment.empty()) {
            if (!current.empty() && current.back() != '/') {
                current += '/';
            }
            current += segment;

            struct stat st {};
            if (stat(current.c_str(), &st) != 0) {
                if (errno == ENOENT) {
                    if (mkdir(current.c_str(), 0775) != 0) {
                        ESP_LOGE(TAG_AEC, "Failed to create directory %s: %s", current.c_str(), strerror(errno));
                        return false;
                    }
                } else {
                    ESP_LOGE(TAG_AEC, "stat(%s) failed: %s", current.c_str(), strerror(errno));
                    return false;
                }
            } else if (!S_ISDIR(st.st_mode)) {
                ESP_LOGE(TAG_AEC, "Path %s exists but is not a directory", current.c_str());
                return false;
            }
        }

        if (next == std::string::npos) {
            break;
        }
        start = next + 1;
    }

    return true;
}

static std::string sanitize_token(const std::string& input)
{
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
        token = "AEC";
    }
    return token;
}

static std::string make_short_name(const std::string& base, const std::string& suffix)
{
    std::string token = sanitize_token(base);
    if (token.size() > 3) {
        token = token.substr(0, 3);
    }
    std::string result = token + suffix;
    if (result.size() > 8) {
        result = result.substr(0, 8);
    }
    return result;
}

} // namespace

// ========== 构造与析构 ==========

audio_sr_afe::audio_sr_afe(audio_tools* parent)
    : parent_(parent)
{
    // 初始化 AEC 上下文
    aec_ctx_.initialized = false;
    aec_ctx_.handle = nullptr;
    aec_ctx_.filter_length = 0;
    aec_ctx_.mode = AFE_MODE_LOW_COST;
    aec_ctx_.mic_channel = AUDIO_MIC_NONE;
    aec_ctx_.reference_channel = AUDIO_MIC_NONE;
}

audio_sr_afe::~audio_sr_afe()
{
    aec_deinit();
}

// ========== 辅助函数 ==========

int audio_sr_afe::mic_channel_to_index(audio_mic_channel_t channel)
{
    switch (channel) {
        case AUDIO_MIC_CHANNEL_1: return 0;
        case AUDIO_MIC_CHANNEL_2: return 1;
        case AUDIO_MIC_CHANNEL_3: return 2;
        case AUDIO_MIC_CHANNEL_4: return 3;
        default: return -1;
    }
}

// ========== AEC 核心接口 ==========

esp_err_t audio_sr_afe::aec_init(audio_mic_channel_t mic_channel,
                                 audio_mic_channel_t reference_channel,
                                 int filter_length,
                                 afe_mode_t aec_mode)
{
    if (!parent_) {
        ESP_LOGE(TAG_AEC, "Parent audio_tools object is null");
        return ESP_ERR_INVALID_STATE;
    }

    if (filter_length <= 0) {
        ESP_LOGE(TAG_AEC, "Invalid filter length: %d", filter_length);
        return ESP_ERR_INVALID_ARG;
    }

    if (!parent_->is_es7210_initialized() || !parent_->get_record_device_handle()) {
        ESP_LOGE(TAG_AEC, "Recording path not ready (ES7210 or record_dev)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->is_es8311_initialized() || !parent_->get_play_device_handle()) {
        ESP_LOGE(TAG_AEC, "Playback path not ready (ES8311 or play_dev)");
        return ESP_ERR_INVALID_STATE;
    }

    const int mic_index = mic_channel_to_index(mic_channel);
    if (mic_index < 0) {
        ESP_LOGE(TAG_AEC, "Invalid mic channel mask: 0x%02X", static_cast<uint8_t>(mic_channel));
        return ESP_ERR_INVALID_ARG;
    }

    const bool reference_is_silent = (reference_channel == AUDIO_MIC_NONE);
    const int ref_index = reference_is_silent ? -1 : mic_channel_to_index(reference_channel);
    if (!reference_is_silent && ref_index < 0) {
        ESP_LOGE(TAG_AEC, "Invalid reference channel mask: 0x%02X", static_cast<uint8_t>(reference_channel));
        return ESP_ERR_INVALID_ARG;
    }

    audio_mic_channel_t enabled_mics = parent_->get_mic_channels();
    if (!(enabled_mics & mic_channel)) {
        ESP_LOGE(TAG_AEC, "Mic channel 0x%02X not enabled in current mask 0x%02X",
                 static_cast<uint8_t>(mic_channel), static_cast<uint8_t>(enabled_mics));
        return ESP_ERR_INVALID_ARG;
    }

    if (!reference_is_silent && !(enabled_mics & reference_channel)) {
        ESP_LOGE(TAG_AEC, "Reference channel 0x%02X not enabled in current mask 0x%02X",
                 static_cast<uint8_t>(reference_channel), static_cast<uint8_t>(enabled_mics));
        return ESP_ERR_INVALID_ARG;
    }

    if (aec_ctx_.initialized) {
        ESP_LOGW(TAG_AEC, "Re-initialising AEC context");
        aec_deinit();
    }

    afe_aec_handle_t* handle = nullptr;
    if (!reference_is_silent) {
        const char* fmt = "MR"; // microphone + reference channel
        handle = afe_aec_create(fmt, filter_length, AFE_TYPE_SR, aec_mode);
        if (!handle) {
            ESP_LOGE(TAG_AEC, "Failed to create AEC handle (fmt=%s, filter=%d)", fmt, filter_length);
            return ESP_ERR_NO_MEM;
        }
    } else {
        ESP_LOGW(TAG_AEC, "AEC initialised with silent reference channel");
    }

    aec_ctx_.handle = handle;
    aec_ctx_.initialized = true;
    aec_ctx_.filter_length = filter_length;
    aec_ctx_.mode = aec_mode;
    aec_ctx_.mic_channel = mic_channel;
    aec_ctx_.reference_channel = reference_channel;

    char ref_desc[16];
    if (reference_is_silent) {
        std::snprintf(ref_desc, sizeof(ref_desc), "SILENT");
    } else {
        std::snprintf(ref_desc, sizeof(ref_desc), "MIC%u", static_cast<unsigned>(ref_index + 1));
    }

    const char *mode_str = (aec_mode == AFE_MODE_LOW_COST) ? "LOW_COST" : "HIGH_PERF";
    ESP_LOGI(TAG_AEC, "AEC init: mic=MIC%u, ref=%s, filter=%d, mode=%s",
             mic_index + 1, ref_desc, filter_length, mode_str);

    return ESP_OK;
}

esp_err_t audio_sr_afe::aec_deinit()
{
    if (!aec_ctx_.initialized) {
        return ESP_OK;
    }

    if (aec_ctx_.handle) {
        afe_aec_destroy(aec_ctx_.handle);
        aec_ctx_.handle = nullptr;
    }

    ESP_LOGI(TAG_AEC, "AEC context released");

    aec_ctx_.initialized = false;
    aec_ctx_.mic_channel = AUDIO_MIC_NONE;
    aec_ctx_.reference_channel = AUDIO_MIC_NONE;
    aec_ctx_.filter_length = 0;
    aec_ctx_.mode = AFE_MODE_LOW_COST;

    return ESP_OK;
}

esp_err_t audio_sr_afe::capture_aec_buffers(uint32_t record_duration_seconds,
                                            aec_capture_output& output)
{
    output = {};

    if (!parent_) {
        ESP_LOGE(TAG_AEC, "Parent audio_tools object is null");
        return ESP_ERR_INVALID_STATE;
    }

    if (!aec_ctx_.initialized) {
        ESP_LOGE(TAG_AEC, "AEC context not initialised. Call aec_init() first.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!aec_ctx_.handle && aec_ctx_.reference_channel != AUDIO_MIC_NONE) {
        ESP_LOGE(TAG_AEC, "AEC handle missing for non-silent reference channel");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->is_es7210_initialized() || !parent_->get_record_device_handle()) {
        ESP_LOGE(TAG_AEC, "Recording path not ready");
        return ESP_ERR_INVALID_STATE;
    }

    const int mic_index = mic_channel_to_index(aec_ctx_.mic_channel);
    const bool reference_is_silent = (aec_ctx_.reference_channel == AUDIO_MIC_NONE);
    const int ref_index = reference_is_silent ? -1 : mic_channel_to_index(aec_ctx_.reference_channel);

    ESP_LOGI(TAG_AEC, "=== AEC Capture Start ===");
    ESP_LOGI(TAG_AEC, "Duration: %lu seconds", record_duration_seconds);
    ESP_LOGI(TAG_AEC, "Mic channel: MIC%d", mic_index + 1);
    if (reference_is_silent) {
        ESP_LOGI(TAG_AEC, "Reference: SILENT (no echo cancellation)");
    } else {
        ESP_LOGI(TAG_AEC, "Reference channel: MIC%d", ref_index + 1);
    }

    // 获取录音设备和采样信息
    esp_codec_dev_handle_t record_dev = parent_->get_record_device_handle();
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = static_cast<uint32_t>(parent_->get_sample_rate());
    fs.channel = (parent_->get_rx_channels() == AUDIO_CHANNELS_MONO) ? 1 : 
                 (parent_->get_rx_channels() == AUDIO_CHANNELS_STEREO) ? 2 :
                 (parent_->get_rx_channels() == AUDIO_CHANNELS_3CHs) ? 3 : 4;
    fs.bits_per_sample = static_cast<uint32_t>(parent_->get_bits_per_sample());

    const bool is_tdm_mode = parent_->is_es7210_tdm_mode();
    const size_t bytes_per_sample = (fs.bits_per_sample / 8) * fs.channel;
    const size_t samples_per_sec = fs.sample_rate;
    const size_t total_samples = samples_per_sec * record_duration_seconds;
    const size_t buffer_size = total_samples * bytes_per_sample;

    ESP_LOGI(TAG_AEC, "Sample rate: %lu Hz, Channels: %d, Bits: %d, TDM: %s",
             fs.sample_rate, fs.channel, fs.bits_per_sample, is_tdm_mode ? "YES" : "NO");
    ESP_LOGI(TAG_AEC, "Allocating %zu bytes for recording buffer", buffer_size);

    // 分配录音缓冲区
    uint8_t* record_buffer = static_cast<uint8_t*>(heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM));
    if (!record_buffer) {
        ESP_LOGE(TAG_AEC, "Failed to allocate recording buffer");
        return ESP_ERR_NO_MEM;
    }

    // 录音
    ESP_LOGI(TAG_AEC, "Recording %lu seconds...", record_duration_seconds);
    const int64_t record_start_time = esp_timer_get_time();
    
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    const size_t chunk_size = 4096;
    while (bytes_read < buffer_size) {
        const size_t remaining = buffer_size - bytes_read;
        const size_t to_read = (remaining < chunk_size) ? remaining : chunk_size;
        ret = esp_codec_dev_read(record_dev, record_buffer + bytes_read, static_cast<int>(to_read));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_AEC, "Recording failed at %zu bytes: %s", bytes_read, esp_err_to_name(ret));
            heap_caps_free(record_buffer);
            return ret;
        }
        bytes_read += to_read;
    }
    
    const int64_t record_end_time = esp_timer_get_time();
    const float actual_duration = (record_end_time - record_start_time) / 1000000.0f;

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AEC, "Recording failed: %s", esp_err_to_name(ret));
        heap_caps_free(record_buffer);
        return ret;
    }

    ESP_LOGI(TAG_AEC, "Recording complete: %zu bytes in %.2f seconds", bytes_read, actual_duration);

    // 拆分通道
    audio_mic_channel_t enabled_mics = parent_->get_mic_channels();
    channel_split_result_t split_result = audio_tools::split_recorded_channels(
        record_buffer, bytes_read, fs, is_tdm_mode, enabled_mics);

    // 释放原始录音缓冲区
    heap_caps_free(record_buffer);
    record_buffer = nullptr;

    if (split_result.status != ESP_OK) {
        ESP_LOGE(TAG_AEC, "Failed to split channels: %s", esp_err_to_name(split_result.status));
        audio_tools::free_channel_split_result(split_result);
        return split_result.status;
    }

    // 获取麦克风和参考信号
    int16_t* mic_data = split_result.mic_buffers[mic_index];
    int16_t* ref_data = reference_is_silent ? nullptr : split_result.mic_buffers[ref_index];

    if (!mic_data) {
        ESP_LOGE(TAG_AEC, "Mic channel buffer is null");
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    if (!reference_is_silent && !ref_data) {
        ESP_LOGE(TAG_AEC, "Reference channel buffer is null");
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    const size_t sample_count = split_result.samples_per_channel;
    ESP_LOGI(TAG_AEC, "Channel split successful: %zu samples per channel", sample_count);

    // 分配 AEC 输出缓冲区
    int16_t* aec_output = static_cast<int16_t*>(heap_caps_aligned_alloc(
        16, sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!aec_output) {
        ESP_LOGE(TAG_AEC, "Failed to allocate AEC output buffer");
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_NO_MEM;
    }

    // AEC 处理
    ESP_LOGI(TAG_AEC, "Processing AEC (filter_length=%d)...", aec_ctx_.filter_length);
    const int64_t aec_start_time = esp_timer_get_time();

#if CONFIG_TASK_WDT
    TaskWdtGuard wdt_guard;
#endif

    size_t frame_size = 160;  // 默认 10ms 帧
    if (!reference_is_silent && aec_ctx_.handle) {
        const int chunksize = afe_aec_get_chunksize(aec_ctx_.handle);
        if (chunksize > 0) {
            frame_size = static_cast<size_t>(chunksize);
        } else {
            ESP_LOGW(TAG_AEC, "Failed to query AEC frame size, fallback to %zu samples", frame_size);
        }
    }
    size_t processed_samples = 0;
    size_t frames_processed = 0;

    int16_t* aec_input = nullptr;
    if (!reference_is_silent && aec_ctx_.handle) {
        const int nch = 2;  // 麦克风 + 参考
        aec_input = static_cast<int16_t*>(heap_caps_aligned_alloc(
            16, frame_size * nch * sizeof(int16_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT));
        if (!aec_input) {
            ESP_LOGE(TAG_AEC, "Failed to allocate AEC input buffer");
            heap_caps_free(aec_output);
            audio_tools::free_channel_split_result(split_result);
            return ESP_ERR_NO_MEM;
        }
    }

    for (size_t offset = 0; offset + frame_size <= sample_count; offset += frame_size) {
        int16_t* mic_frame = mic_data + offset;
        int16_t* ref_frame = reference_is_silent ? nullptr : (ref_data + offset);
        int16_t* out_frame = aec_output + offset;

        if (reference_is_silent) {
            // 无参考信号，直接复制
            memcpy(out_frame, mic_frame, frame_size * sizeof(int16_t));
        } else {
            // 准备交错格式输入
            const int nch = 2;
            for (size_t i = 0; i < frame_size; i++) {
                aec_input[i * nch + 0] = mic_frame[i];  // 麦克风
                aec_input[i * nch + 1] = ref_frame[i];  // 参考
            }
            
            // 执行 AEC 处理
            afe_aec_process(aec_ctx_.handle, aec_input, out_frame);
        }

        processed_samples += frame_size;
        ++frames_processed;

#if CONFIG_TASK_WDT
        if (wdt_guard.can_reset() && ((frames_processed & 0x7) == 0)) {
            esp_err_t reset_ret = esp_task_wdt_reset();
            if (reset_ret != ESP_OK) {
                ESP_LOGW(TAG_AEC, "Failed to reset task WDT: %s", esp_err_to_name(reset_ret));
            }
        }
#endif
        if ((frames_processed & 0xF) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));  // 让 Idle 任务运行以满足 WDT
        }
    }

    if (aec_input) {
        heap_caps_free(aec_input);
    }

    // 处理剩余样本（如果有）
    if (processed_samples < sample_count) {
        const size_t remaining = sample_count - processed_samples;
        memcpy(aec_output + processed_samples, mic_data + processed_samples, 
               remaining * sizeof(int16_t));
    }

    const int64_t aec_end_time = esp_timer_get_time();
    const float aec_duration = (aec_end_time - aec_start_time) / 1000000.0f;
    ESP_LOGI(TAG_AEC, "AEC processing complete: %zu samples in %.3f seconds",
             processed_samples, aec_duration);

    output.split_result = split_result;
    output.mic_data = mic_data;
    output.ref_data = reference_is_silent ? nullptr : ref_data;
    output.aec_output = aec_output;
    output.sample_count = sample_count;
    output.fs = fs;
    output.reference_is_silent = reference_is_silent;

    return ESP_OK;
}

void audio_sr_afe::release_aec_buffers(aec_capture_output& output)
{
    if (output.aec_output) {
        heap_caps_free(output.aec_output);
        output.aec_output = nullptr;
    }
    audio_tools::free_channel_split_result(output.split_result);
    output.split_result = {};
    output.mic_data = nullptr;
    output.ref_data = nullptr;
    output.sample_count = 0;
    output.fs = {};
    output.reference_is_silent = true;
}

esp_err_t audio_sr_afe::aec_test_loopback(uint32_t record_duration_seconds,
                                          bool play_original_audio,
                                          bool play_processed_audio)
{
    if ((play_original_audio || play_processed_audio) &&
        (!parent_->is_es8311_initialized() || !parent_->get_play_device_handle())) {
        ESP_LOGE(TAG_AEC, "Playback path not ready");
        return ESP_ERR_INVALID_STATE;
    }

    aec_capture_output capture{};
    esp_err_t ret = capture_aec_buffers(record_duration_seconds, capture);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG_AEC, "=== AEC Loopback Test Start ===");

    esp_err_t result = ESP_OK;

    // 播放原始音频（可选）
    if (play_original_audio) {
        ESP_LOGI(TAG_AEC, "Playing original audio (mic input)...");
        
        // 使用 audio_remix_tools 进行单声道转立体声
        uint8_t* stereo_buffer = nullptr;
        size_t stereo_size = 0;
        ret = remix_convert_pcm_to_format(
            reinterpret_cast<const uint8_t*>(capture.mic_data),
            capture.sample_count * sizeof(int16_t),
            capture.fs.sample_rate,
            1,  // 单声道
            AUDIO_TYPE_INT16,
            capture.fs.sample_rate,
            2,  // 立体声
            AUDIO_TYPE_INT16,
            &stereo_buffer,
            &stereo_size);
        
        if (ret == ESP_OK && stereo_buffer) {
            ret = parent_->play_audio_buffer(
                stereo_buffer, stereo_size,
                capture.fs.sample_rate, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT,
                AUDIO_PLAYBACK_BLOCKING, 0.0f);
            heap_caps_free(stereo_buffer);
            
            if (ret == ESP_OK) {
                parent_->clear_audio_pipeline(100);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if (result == ESP_OK) {
                result = ret;
            }
        } else if (result == ESP_OK && ret != ESP_OK) {
            result = ret;
        }
    }

    // 播放处理后的音频（可选）
    if (play_processed_audio) {
        ESP_LOGI(TAG_AEC, "Playing AEC-processed audio...");
        
        // 使用 audio_remix_tools 进行单声道转立体声
        uint8_t* stereo_buffer = nullptr;
        size_t stereo_size = 0;
        ret = remix_convert_pcm_to_format(
            reinterpret_cast<const uint8_t*>(capture.aec_output),
            capture.sample_count * sizeof(int16_t),
            capture.fs.sample_rate,
            1,  // 单声道
            AUDIO_TYPE_INT16,
            capture.fs.sample_rate,
            2,  // 立体声
            AUDIO_TYPE_INT16,
            &stereo_buffer,
            &stereo_size);
        
        if (ret == ESP_OK && stereo_buffer) {
            ret = parent_->play_audio_buffer(
                stereo_buffer, stereo_size,
                capture.fs.sample_rate, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT,
                AUDIO_PLAYBACK_BLOCKING, 0.0f);
            heap_caps_free(stereo_buffer);
            
            if (ret == ESP_OK) {
                parent_->clear_audio_pipeline(100);
            } else if (result == ESP_OK) {
                result = ret;
            }
        } else if (result == ESP_OK && ret != ESP_OK) {
            result = ret;
        }
    }

    release_aec_buffers(capture);

    ESP_LOGI(TAG_AEC, "=== AEC Loopback Test Complete ===");
    return result;
}

esp_err_t audio_sr_afe::aec_test_loopback_to_files(uint32_t record_duration_seconds,
                                                   const char* mount_point,
                                                   const char* file_prefix)
{
    if (!mount_point || mount_point[0] == '\0') {
        ESP_LOGE(TAG_AEC, "Mount point is empty");
        return ESP_ERR_INVALID_ARG;
    }

    if (!file_prefix || file_prefix[0] == '\0') {
        ESP_LOGE(TAG_AEC, "File prefix is empty");
        return ESP_ERR_INVALID_ARG;
    }

    if (!ensure_directory_exists(mount_point)) {
        return ESP_ERR_INVALID_STATE;
    }

    struct stat st {};
    if (stat(mount_point, &st) != 0 || !S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG_AEC, "Mount point %s not accessible", mount_point);
        return ESP_ERR_INVALID_STATE;
    }

    aec_capture_output capture{};
    esp_err_t ret = capture_aec_buffers(record_duration_seconds, capture);
    if (ret != ESP_OK) {
        return ret;
    }

    std::string base_path(mount_point);
    if (!base_path.empty() && base_path.back() != '/' && base_path.back() != '\\') {
        base_path += '/';
    }

    std::string prefix(file_prefix);
    const int mic_index = mic_channel_to_index(aec_ctx_.mic_channel);
    const bool reference_is_silent = capture.reference_is_silent;
    const int ref_index = reference_is_silent ? -1 : mic_channel_to_index(aec_ctx_.reference_channel);

    std::string mic_path;
    std::string ref_path;
    std::string out_path;

#if defined(CONFIG_FATFS_LFN_NONE)
    const std::string mic_suffix = "M" + std::to_string(mic_index + 1);
    const std::string ref_suffix = reference_is_silent ? "RS" : ("R" + std::to_string(ref_index + 1));
    const std::string out_suffix = "OUT";

    mic_path = base_path + make_short_name(prefix, mic_suffix) + ".PCM";
    ref_path = base_path + make_short_name(prefix, ref_suffix) + ".PCM";
    out_path = base_path + make_short_name(prefix, out_suffix) + ".PCM";
#else
    mic_path = base_path + prefix + "_mic_ch" + std::to_string(mic_index + 1) + ".pcm";
    ref_path = base_path + prefix + "_ref" + (reference_is_silent ? std::string("_silent") : ("_ch" + std::to_string(ref_index + 1))) + ".pcm";
    out_path = base_path + prefix + "_aec_out.pcm";
#endif

    auto write_pcm = [&](const std::string& path, const int16_t* data, bool fill_zero) -> esp_err_t {
        if (!fill_zero && !data) {
            ESP_LOGE(TAG_AEC, "No data for %s", path.c_str());
            return ESP_ERR_INVALID_STATE;
        }

        const size_t pos = path.find_last_of("/\\");
        if (pos != std::string::npos) {
            const std::string dir = path.substr(0, pos);
            if (!dir.empty() && !ensure_directory_exists(dir)) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        FILE* file = fopen(path.c_str(), "wb");
        if (!file) {
            ESP_LOGE(TAG_AEC, "Failed to open %s: %s", path.c_str(), strerror(errno));
            return ESP_FAIL;
        }

        esp_err_t write_result = ESP_OK;
        if (fill_zero) {
            std::vector<int16_t> zeros(1024, 0);
            size_t remaining = capture.sample_count;
            while (remaining > 0) {
                size_t chunk = std::min(remaining, zeros.size());
                if (fwrite(zeros.data(), sizeof(int16_t), chunk, file) != chunk) {
                    write_result = ESP_FAIL;
                    break;
                }
                remaining -= chunk;
            }
        } else {
            if (fwrite(data, sizeof(int16_t), capture.sample_count, file) != capture.sample_count) {
                write_result = ESP_FAIL;
            }
        }

        fflush(file);
        fclose(file);

        if (write_result == ESP_OK) {
            ESP_LOGI(TAG_AEC, "Saved %s (%zu samples @ %lu Hz)", path.c_str(),
                     capture.sample_count, static_cast<unsigned long>(capture.fs.sample_rate));
        } else {
            ESP_LOGE(TAG_AEC, "Failed while writing %s", path.c_str());
        }
        return write_result;
    };

    esp_err_t result = write_pcm(mic_path, capture.mic_data, false);
    if (result == ESP_OK) {
        result = write_pcm(ref_path, capture.ref_data, reference_is_silent);
    }
    if (result == ESP_OK) {
        result = write_pcm(out_path, capture.aec_output, false);
    }

    release_aec_buffers(capture);
    return result;
}

esp_err_t audio_sr_afe::aec_test(uint32_t record_duration_seconds,
                                 audio_mic_channel_t mic_channel,
                                 audio_mic_channel_t reference_channel,
                                 int filter_length,
                                 afe_mode_t aec_mode,
                                 bool play_original_audio,
                                 bool play_processed_audio)
{
    esp_err_t ret = aec_init(mic_channel, reference_channel, filter_length, aec_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    return aec_test_loopback(record_duration_seconds, play_original_audio, play_processed_audio);
}

esp_err_t audio_sr_afe::test_aec_loopback(uint32_t record_duration_seconds, uint8_t play_channels)
{
    ESP_LOGW(TAG_AEC, "test_aec_loopback() is deprecated. Use aec_init()/aec_test_loopback() instead.");
    if (!aec_ctx_.initialized) {
        ESP_LOGE(TAG_AEC, "AEC context not initialised. Call aec_init() first.");
        return ESP_ERR_INVALID_STATE;
    }
    const bool play_original = (play_channels & 0x01) != 0;
    const bool play_processed = (play_channels & 0x02) != 0;
    return aec_test_loopback(record_duration_seconds, play_original, play_processed);
}

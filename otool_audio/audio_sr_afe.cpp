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
    , session_task_handle_(nullptr)
    , session_ringbuf_(nullptr)
    , session_stop_flag_(false)
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

    // S7: 若流式会话正在运行，先自动停止
    if (aec_session_is_running()) {
        ESP_LOGW(TAG_AEC, "AEC session still running during deinit, stopping...");
        aec_session_stop();
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

    // S6: 批处理模式与流式会话互斥
    if (aec_session_is_running()) {
        ESP_LOGE(TAG_AEC, "Cannot use batch capture while AEC streaming session is active");
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
    int16_t* original_mic_data = split_result.mic_buffers[mic_index];
    int16_t* original_ref_data = reference_is_silent ? nullptr : split_result.mic_buffers[ref_index];

    if (!original_mic_data) {
        ESP_LOGE(TAG_AEC, "Mic channel buffer is null");
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    if (!reference_is_silent && !original_ref_data) {
        ESP_LOGE(TAG_AEC, "Reference channel buffer is null");
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_INVALID_STATE;
    }

    size_t sample_count = split_result.samples_per_channel;
    ESP_LOGI(TAG_AEC, "Channel split successful: %zu samples per channel", sample_count);

    // ================== 重采样适配 (如果当前系统采样率不是 16kHz) ==================
    int16_t* mic_data_16k = nullptr;
    int16_t* ref_data_16k = nullptr;
    const uint32_t aec_required_rate = 16000;
    const bool need_resample = (fs.sample_rate != aec_required_rate);

    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Resampling from %lu Hz to %lu Hz for AEC processing...", fs.sample_rate, aec_required_rate);
        
        // 1. 转为 float，这是 resample_linear 的输入要求
        float* mic_float = static_cast<float*>(heap_caps_malloc(sample_count * sizeof(float), MALLOC_CAP_SPIRAM));
        if (!mic_float) {
            ESP_LOGE(TAG_AEC, "Failed to allocate float buffer for MIC");
            audio_tools::free_channel_split_result(split_result);
            return ESP_ERR_NO_MEM;
        }
        for (size_t i = 0; i < sample_count; ++i) {
            mic_float[i] = static_cast<float>(original_mic_data[i]);
        }

        // 调用重采样
        float* resampled_mic = nullptr;
        size_t new_sample_count = 0;
        esp_err_t ret_res = resample_linear(mic_float, sample_count, 1, fs.sample_rate, aec_required_rate, &resampled_mic, &new_sample_count);
        heap_caps_free(mic_float);
        
        if (ret_res != ESP_OK || !resampled_mic) {
            ESP_LOGE(TAG_AEC, "Mic resample failed");
            audio_tools::free_channel_split_result(split_result);
            return ret_res;
        }

        // 转回 int16_t
        mic_data_16k = static_cast<int16_t*>(heap_caps_malloc(new_sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM));
        for (size_t i = 0; i < new_sample_count; ++i) {
            mic_data_16k[i] = static_cast<int16_t>(resampled_mic[i]); // 忽略极端的钳位由于这里原本就是int16上来
        }
        heap_caps_free(resampled_mic);

        // 如果包含参考信号，也要对其进行相同的重采样
        if (!reference_is_silent) {
            float* ref_float = static_cast<float*>(heap_caps_malloc(sample_count * sizeof(float), MALLOC_CAP_SPIRAM));
            for (size_t i = 0; i < sample_count; ++i) {
                ref_float[i] = static_cast<float>(original_ref_data[i]);
            }
            float* resampled_ref = nullptr;
            size_t new_ref_count = 0;
            esp_err_t ref_res = resample_linear(ref_float, sample_count, 1, fs.sample_rate, aec_required_rate, &resampled_ref, &new_ref_count);
            heap_caps_free(ref_float);
            
            if (ref_res != ESP_OK || !resampled_ref) {
                ESP_LOGE(TAG_AEC, "Ref resample failed");
                heap_caps_free(mic_data_16k);
                audio_tools::free_channel_split_result(split_result);
                return ref_res;
            }

            ref_data_16k = static_cast<int16_t*>(heap_caps_malloc(new_ref_count * sizeof(int16_t), MALLOC_CAP_SPIRAM));
            for (size_t i = 0; i < new_ref_count; ++i) {
                ref_data_16k[i] = static_cast<int16_t>(resampled_ref[i]);
            }
            heap_caps_free(resampled_ref);
        }

        sample_count = new_sample_count; // 更新后样本数
    } else {
        // 不需要重采样，直接使用拆分出来的缓冲区
        mic_data_16k = original_mic_data;
        ref_data_16k = original_ref_data;
    }
    // ==============================================================================================

    // 分配 AEC 输出缓冲区 (现在永远是 16000Hz 频率的大小了)
    int16_t* aec_output_16k = static_cast<int16_t*>(heap_caps_aligned_alloc(
        16, sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!aec_output_16k) {
        ESP_LOGE(TAG_AEC, "Failed to allocate AEC output buffer");
        if (need_resample) {
            heap_caps_free(mic_data_16k);
            if (ref_data_16k) heap_caps_free(ref_data_16k);
        }
        audio_tools::free_channel_split_result(split_result);
        return ESP_ERR_NO_MEM;
    }

    // AEC 处理
    ESP_LOGI(TAG_AEC, "Processing AEC (filter_length=%d)...", aec_ctx_.filter_length);
    const int64_t aec_start_time = esp_timer_get_time();

#if CONFIG_TASK_WDT
    TaskWdtGuard wdt_guard;
#endif

    size_t frame_size = 160;  // 默认对16kHz来说，10ms帧是160个采样
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
            heap_caps_free(aec_output_16k);
            if (need_resample) {
                heap_caps_free(mic_data_16k);
                if (ref_data_16k) heap_caps_free(ref_data_16k);
            }
            audio_tools::free_channel_split_result(split_result);
            return ESP_ERR_NO_MEM;
        }
    }

    for (size_t offset = 0; offset + frame_size <= sample_count; offset += frame_size) {
        int16_t* mic_frame = mic_data_16k + offset;
        int16_t* ref_frame = reference_is_silent ? nullptr : (ref_data_16k + offset);
        int16_t* out_frame = aec_output_16k + offset;

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
        memcpy(aec_output_16k + processed_samples, mic_data_16k + processed_samples,
               remaining * sizeof(int16_t));
    }

    const int64_t aec_end_time = esp_timer_get_time();
    const float aec_duration = (aec_end_time - aec_start_time) / 1000000.0f;
    ESP_LOGI(TAG_AEC, "AEC processing complete: %zu samples in %.3f seconds",
             processed_samples, aec_duration);

    // ================== 如果存在重采样，将处理完的干声转回原始系统采样率 ==================
    int16_t* final_aec_output = aec_output_16k;
    size_t final_sample_count = sample_count;

    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Resampling AEC output back from %lu Hz to %lu Hz...", aec_required_rate, fs.sample_rate);
        float* aec_float = static_cast<float*>(heap_caps_malloc(sample_count * sizeof(float), MALLOC_CAP_SPIRAM));
        for (size_t i = 0; i < sample_count; ++i) {
            aec_float[i] = static_cast<float>(aec_output_16k[i]);
        }
        
        float* resampled_aec = nullptr;
        esp_err_t back_res = resample_linear(aec_float, sample_count, 1, aec_required_rate, fs.sample_rate, &resampled_aec, &final_sample_count);
        heap_caps_free(aec_float);

        if (back_res != ESP_OK || !resampled_aec) {
            ESP_LOGE(TAG_AEC, "AEC output resample back failed");
            // 作为fallback，还是返回16k的就算了，外层可能播放会快一点
        } else {
            final_aec_output = static_cast<int16_t*>(heap_caps_aligned_alloc(16, final_sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            for (size_t i = 0; i < final_sample_count; ++i) {
                final_aec_output[i] = static_cast<int16_t>(resampled_aec[i]);
            }
            heap_caps_free(resampled_aec);
            heap_caps_free(aec_output_16k); // 释放降采过的
        }

        // 清理因为输入重采样而中途分配的临时缓冲
        heap_caps_free(mic_data_16k);
        if (ref_data_16k) {
            heap_caps_free(ref_data_16k);
        }
    }
    // ==================================================================================

    // 此时 output 里存的是系统原始要求的那些通道信息，便于播放和保存
    output.split_result = split_result;
    output.mic_data = original_mic_data;
    output.ref_data = reference_is_silent ? nullptr : original_ref_data;
    output.aec_output = final_aec_output;
    output.sample_count = final_sample_count;
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

// ========== 流式 AEC 会话 ==========

void audio_sr_afe::aec_stream_task(void* param)
{
    audio_sr_afe* self = static_cast<audio_sr_afe*>(param);
    audio_tools* parent = self->parent_;

    ESP_LOGI(TAG_AEC, "AEC stream task started");

    // --- 获取 AEC 帧大小 ---
    const int chunksize = afe_aec_get_chunksize(self->aec_ctx_.handle);
    if (chunksize <= 0) {
        ESP_LOGE(TAG_AEC, "Invalid AEC chunksize: %d", chunksize);
        self->session_task_handle_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG_AEC, "AEC chunksize: %d samples (%.1f ms)", chunksize, chunksize * 1000.0f / 16000.0f);

    // --- 硬件参数 ---
    esp_codec_dev_handle_t record_dev = parent->get_record_device_handle();
    const bool is_tdm = parent->is_es7210_tdm_mode();
    const uint32_t hw_bits = static_cast<uint32_t>(parent->get_bits_per_sample());
    const uint32_t hw_channels = (parent->get_rx_channels() == AUDIO_CHANNELS_MONO) ? 1 : 2;
    const size_t bytes_per_hw_frame = (hw_bits / 8) * hw_channels;

    const int mic_idx = mic_channel_to_index(self->aec_ctx_.mic_channel);
    const bool ref_silent = (self->aec_ctx_.reference_channel == AUDIO_MIC_NONE);
    const int ref_idx = ref_silent ? -1 : mic_channel_to_index(self->aec_ctx_.reference_channel);

    // --- 分配缓冲区 ---
    // I2S 读取缓冲: chunksize 个 TDM/STD 帧
    const size_t read_buf_size = static_cast<size_t>(chunksize) * bytes_per_hw_frame;
    uint8_t* read_buf = static_cast<uint8_t*>(heap_caps_malloc(read_buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!read_buf) {
        // DMA 内存不足时回退到 SPIRAM
        read_buf = static_cast<uint8_t*>(heap_caps_malloc(read_buf_size, MALLOC_CAP_SPIRAM));
    }

    // AEC 交织输入缓冲 (mic + ref 交错, 每帧 2 个 int16_t)
    const size_t aec_nch = ref_silent ? 1 : 2;
    int16_t* aec_input = static_cast<int16_t*>(heap_caps_aligned_alloc(
        16, chunksize * aec_nch * sizeof(int16_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT));

    // AEC 输出缓冲
    int16_t* aec_output = static_cast<int16_t*>(heap_caps_aligned_alloc(
        16, chunksize * sizeof(int16_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT));

    if (!read_buf || !aec_input || !aec_output) {
        ESP_LOGE(TAG_AEC, "Failed to allocate stream buffers (read=%p, input=%p, output=%p)",
                 read_buf, aec_input, aec_output);
        if (read_buf) heap_caps_free(read_buf);
        if (aec_input) heap_caps_free(aec_input);
        if (aec_output) heap_caps_free(aec_output);
        self->session_task_handle_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    size_t total_frames_processed = 0;
    size_t ringbuf_overflow_count = 0;

    // --- 主循环 ---
    while (!self->session_stop_flag_) {
        // 1) 从 I2S 读取一帧数据 (阻塞式)
        esp_err_t ret = esp_codec_dev_read(record_dev, read_buf, static_cast<int>(read_buf_size));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG_AEC, "I2S read failed: %s, retrying...", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // 2) 拆通道: 从 TDM/STD 数据中提取 mic 和 ref 通道
        if (is_tdm && (hw_bits == 32) && (hw_channels == 2)) {
            // TDM 32-bit 双槽模式:
            // left_word:  MIC1(high16) | MIC3(low16)
            // right_word: MIC2(high16) | MIC4(low16)
            const uint32_t* frames = reinterpret_cast<const uint32_t*>(read_buf);
            for (int i = 0; i < chunksize; ++i) {
                uint32_t left_word  = frames[i * 2];
                uint32_t right_word = frames[i * 2 + 1];

                int16_t mic_samples[4] = {
                    static_cast<int16_t>(left_word >> 16),      // MIC1
                    static_cast<int16_t>(right_word >> 16),     // MIC2
                    static_cast<int16_t>(left_word & 0xFFFF),   // MIC3
                    static_cast<int16_t>(right_word & 0xFFFF)   // MIC4
                };

                if (ref_silent) {
                    aec_input[i] = mic_samples[mic_idx];
                } else {
                    aec_input[i * 2]     = mic_samples[mic_idx];
                    aec_input[i * 2 + 1] = mic_samples[ref_idx];
                }
            }
        } else if (!is_tdm && (hw_channels == 2)) {
            // 标准立体声模式: [left][right] 交错
            const size_t slot_bytes = hw_bits / 8;
            for (int i = 0; i < chunksize; ++i) {
                const uint8_t* frame_ptr = read_buf + i * bytes_per_hw_frame;
                int16_t left_sample, right_sample;

                if (slot_bytes == 4) {
                    // 32-bit 槽: 取高 16 位作为有效数据
                    int32_t left_raw  = *reinterpret_cast<const int32_t*>(frame_ptr);
                    int32_t right_raw = *reinterpret_cast<const int32_t*>(frame_ptr + 4);
                    left_sample  = static_cast<int16_t>(left_raw >> 16);
                    right_sample = static_cast<int16_t>(right_raw >> 16);
                } else {
                    left_sample  = *reinterpret_cast<const int16_t*>(frame_ptr);
                    right_sample = *reinterpret_cast<const int16_t*>(frame_ptr + slot_bytes);
                }

                int16_t ch_data[2] = { left_sample, right_sample };
                if (ref_silent) {
                    aec_input[i] = ch_data[mic_idx < 2 ? mic_idx : 0];
                } else {
                    aec_input[i * 2]     = ch_data[mic_idx < 2 ? mic_idx : 0];
                    aec_input[i * 2 + 1] = ch_data[ref_idx < 2 ? ref_idx : 0];
                }
            }
        } else {
            // 不支持的配置: 单声道或其他,仅复制零
            memset(aec_input, 0, chunksize * aec_nch * sizeof(int16_t));
            if (total_frames_processed == 0) {
                ESP_LOGW(TAG_AEC, "Unsupported I2S config for streaming (tdm=%d, bits=%lu, ch=%lu)",
                         is_tdm, static_cast<unsigned long>(hw_bits), static_cast<unsigned long>(hw_channels));
            }
        }

        // 3) AEC 处理
        if (!ref_silent && self->aec_ctx_.handle) {
            afe_aec_process(self->aec_ctx_.handle, aec_input, aec_output);
        } else {
            // 无参考信号: 直接透传 mic 数据
            if (ref_silent) {
                memcpy(aec_output, aec_input, chunksize * sizeof(int16_t));
            } else {
                // handle 为 null（理论不应出现）
                for (int i = 0; i < chunksize; ++i) {
                    aec_output[i] = aec_input[i * 2];
                }
            }
        }

        // 4) 写入 RingBuffer
        BaseType_t rb_ret = xRingbufferSend(self->session_ringbuf_, aec_output,
                                            chunksize * sizeof(int16_t), pdMS_TO_TICKS(50));
        if (rb_ret != pdTRUE) {
            ++ringbuf_overflow_count;
            if ((ringbuf_overflow_count & 0x3F) == 1) {
                ESP_LOGW(TAG_AEC, "RingBuffer overflow (count=%zu), reader too slow",
                         ringbuf_overflow_count);
            }
        }

        ++total_frames_processed;
    }

    // --- 退出清理 ---
    heap_caps_free(read_buf);
    heap_caps_free(aec_input);
    heap_caps_free(aec_output);

    ESP_LOGI(TAG_AEC, "AEC stream task exiting (processed %zu frames, %zu overflows)",
             total_frames_processed, ringbuf_overflow_count);

    self->session_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t audio_sr_afe::aec_session_start(size_t output_ringbuf_size,
                                           UBaseType_t task_priority,
                                           uint32_t task_stack_size)
{
    if (aec_session_is_running()) {
        ESP_LOGW(TAG_AEC, "AEC session already running");
        return ESP_ERR_INVALID_STATE;
    }

    if (!aec_ctx_.initialized) {
        ESP_LOGE(TAG_AEC, "AEC not initialized. Call aec_init() first.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_) {
        ESP_LOGE(TAG_AEC, "Parent audio_tools is null");
        return ESP_ERR_INVALID_STATE;
    }

    if (!parent_->is_es7210_initialized() || !parent_->get_record_device_handle()) {
        ESP_LOGE(TAG_AEC, "Recording path not ready");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查采样率: AEC 硬性要求 16kHz
    const uint32_t hw_rate = static_cast<uint32_t>(parent_->get_sample_rate());
    if (hw_rate != 16000) {
        ESP_LOGE(TAG_AEC, "Streaming AEC requires 16kHz sample rate (current: %lu Hz). "
                 "Use batch mode (capture_aec_buffers) for non-16kHz configurations.",
                 static_cast<unsigned long>(hw_rate));
        return ESP_ERR_INVALID_STATE;
    }

    // 检查与 record() 的互斥
    if (parent_->is_async_playback_running()) {
        // 异步播放不影响录音通道，可以共存（不阻止）
    }

    // 创建 RingBuffer
    session_ringbuf_ = xRingbufferCreate(output_ringbuf_size, RINGBUF_TYPE_BYTEBUF);
    if (!session_ringbuf_) {
        ESP_LOGE(TAG_AEC, "Failed to create RingBuffer (%zu bytes)", output_ringbuf_size);
        return ESP_ERR_NO_MEM;
    }

    // 启动后台任务
    session_stop_flag_ = false;
    BaseType_t ret = xTaskCreatePinnedToCore(
        aec_stream_task,
        "aec_stream",
        task_stack_size,
        this,
        task_priority,
        &session_task_handle_,
        1  // 固定在 CPU Core 1，避免影响主线程和 LVGL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG_AEC, "Failed to create AEC stream task");
        vRingbufferDelete(session_ringbuf_);
        session_ringbuf_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG_AEC, "AEC streaming session started (ringbuf=%zuKB, priority=%u, stack=%luB)",
             output_ringbuf_size / 1024, task_priority, static_cast<unsigned long>(task_stack_size));
    return ESP_OK;
}

esp_err_t audio_sr_afe::aec_session_stop()
{
    if (!aec_session_is_running()) {
        ESP_LOGW(TAG_AEC, "AEC session not running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG_AEC, "Stopping AEC streaming session...");

    // 通知任务退出
    session_stop_flag_ = true;

    // 等待任务自行退出 (最多等 3 秒)
    const int max_wait_ms = 3000;
    const int poll_interval_ms = 50;
    int waited_ms = 0;
    while (session_task_handle_ != nullptr && waited_ms < max_wait_ms) {
        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
        waited_ms += poll_interval_ms;
    }

    if (session_task_handle_ != nullptr) {
        ESP_LOGW(TAG_AEC, "AEC stream task did not exit within %d ms, force deleting", max_wait_ms);
        vTaskDelete(session_task_handle_);
        session_task_handle_ = nullptr;
    }

    // 释放 RingBuffer
    if (session_ringbuf_) {
        vRingbufferDelete(session_ringbuf_);
        session_ringbuf_ = nullptr;
    }

    session_stop_flag_ = false;

    ESP_LOGI(TAG_AEC, "AEC streaming session stopped (AEC filter coefficients preserved)");
    return ESP_OK;
}

int audio_sr_afe::aec_session_read(void* buf, size_t len, uint32_t timeout_ms)
{
    if (!aec_session_is_running() || !session_ringbuf_) {
        return -1;
    }

    if (!buf || len == 0) {
        return 0;
    }

    // RingBuffer BYTEBUF 模式: 一次性读取最多 len 字节
    size_t item_size = 0;
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    void* item = xRingbufferReceiveUpTo(session_ringbuf_, &item_size, ticks, len);

    if (!item || item_size == 0) {
        return 0;
    }

    memcpy(buf, item, item_size);
    vRingbufferReturnItem(session_ringbuf_, item);

    return static_cast<int>(item_size);
}

bool audio_sr_afe::aec_session_is_running() const
{
    return session_task_handle_ != nullptr;
}

// ========== 批处理 AEC 测试接口 ==========

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

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_playback.h"
#include "audio_tools.h"
#include "audio_remix_tools.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "audio_playback";
static constexpr size_t SILENCE_CHUNK_CAPACITY = 1024;
static uint8_t g_silence_chunk[SILENCE_CHUNK_CAPACITY] = {0};

// ============================================================================
// 音频文件嵌入声明 - 统一命名规范
// ============================================================================
#ifdef USE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_end[];
#endif

#ifdef USE_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_end[];
#endif

#ifdef USE_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start[];
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end[];
#endif

#ifdef USE_AUDIO_STARTUP_1CH_16K_16BIT_4S
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end[];
#endif

#ifdef USE_AUDIO_STARTUP_2CH_16K_16BIT_4S
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_end[];
#endif

// ============================================================================
// 音频文件元数据表
// ============================================================================
typedef bool (*AudioDataGetter)(const uint8_t*& start, size_t& len);

#ifdef USE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
static bool get_candy_wind_1ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
    len = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
static bool get_candy_wind_1ch_44k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start;
    len = _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_end - _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
static bool get_candy_wind_2ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
    len = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
static bool get_candy_wind_2ch_44k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start;
    len = _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_end - _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
static bool get_sine_440hz_2ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
    len = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end - _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_STARTUP_1CH_16K_16BIT_4S
static bool get_startup_1ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
    len = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end - _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
    return true;
}
#endif

#ifdef USE_AUDIO_STARTUP_2CH_16K_16BIT_4S
static bool get_startup_2ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start;
    len = _binary_startup_pcm_2ch_16k_16bit_4s_pcm_end - _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start;
    return true;
}
#endif

struct AudioFileMetadata {
    audio_file_type_t type;
    const char* filename;
    uint32_t sample_rate;
    audio_channels_t channels;
    i2s_data_bit_width_t bits;
    AudioDataGetter data_getter;
};

static const AudioFileMetadata AUDIO_FILE_TABLE[] = {
#ifdef USE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
    {AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S, "candy_wind_pcm_1ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_1ch_16k_data},
#endif
#ifdef USE_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
    {AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S, "candy_wind_pcm_1ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_1ch_44k_data},
#endif
#ifdef USE_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
    {AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S, "candy_wind_pcm_2ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_2ch_16k_data},
#endif
#ifdef USE_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
    {AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S, "candy_wind_pcm_2ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_2ch_44k_data},
#endif
#ifdef USE_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
    {AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S, "sine_440Hz_pcm_2ch_16k_16bit_10s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_sine_440hz_2ch_16k_data},
#endif
#ifdef USE_AUDIO_STARTUP_1CH_16K_16BIT_4S
    {AUDIO_FILE_STARTUP_1CH_16K_16B_4S, "startup_pcm_1ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_startup_1ch_16k_data},
#endif
#ifdef USE_AUDIO_STARTUP_2CH_16K_16BIT_4S
    {AUDIO_FILE_STARTUP_2CH_16K_16B_4S, "startup_pcm_2ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_startup_2ch_16k_data},
#endif
};

static constexpr size_t AUDIO_FILE_COUNT = sizeof(AUDIO_FILE_TABLE) / sizeof(AUDIO_FILE_TABLE[0]);

static const AudioFileMetadata* find_audio_metadata(audio_file_type_t type) {
    for (size_t i = 0; i < AUDIO_FILE_COUNT; ++i) {
        if (AUDIO_FILE_TABLE[i].type == type) {
            return &AUDIO_FILE_TABLE[i];
        }
    }
    return nullptr;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

audio_playback::audio_playback(audio_tools* parent)
    : parent_(parent)
{
    ESP_LOGI(TAG, "audio_playback sub-object created");
}

audio_playback::~audio_playback()
{
    if (playback_task_handle_) {
        ESP_LOGW(TAG, "Waiting for playback task to finish before destruction");
        while (playback_task_handle_) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    ESP_LOGI(TAG, "audio_playback sub-object destroyed");
}

// ============================================================================
// 音频文件信息
// ============================================================================

int audio_playback::get_available_pcm_count() const
{
    int count = 0;
    for (size_t i = 0; i < AUDIO_FILE_COUNT; ++i) {
        if (is_audio_file_available(AUDIO_FILE_TABLE[i].type)) {
            count++;
        }
    }
    return count;
}

const char* audio_playback::get_audio_file_name(audio_file_type_t audio_type) const
{
    const AudioFileMetadata* meta = find_audio_metadata(audio_type);
    return meta ? meta->filename : "unknown";
}

bool audio_playback::is_audio_file_available(audio_file_type_t audio_type) const
{
    const AudioFileMetadata* meta = find_audio_metadata(audio_type);
    return (meta != nullptr);
}

// ============================================================================
// 获取PCM数据和格式参数
// ============================================================================

esp_err_t audio_playback::get_pcm_data_and_format(audio_file_type_t audio_type,
                                                    const uint8_t*& pcm_start,
                                                    size_t& pcm_len,
                                                    uint32_t& file_sample_rate_hz,
                                                    audio_channels_t& file_channels,
                                                    i2s_data_bit_width_t& file_bits)
{
    pcm_start = nullptr;
    pcm_len = 0;

    const AudioFileMetadata* meta = find_audio_metadata(audio_type);
    if (!meta) {
        ESP_LOGE(TAG, "Invalid audio file type: %d", audio_type);
        return ESP_ERR_INVALID_ARG;
    }

    file_sample_rate_hz = meta->sample_rate;
    file_channels = meta->channels;
    file_bits = meta->bits;

    if (!meta->data_getter) {
        ESP_LOGE(TAG, "%s not compiled in (data_getter is null)", meta->filename);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!meta->data_getter(pcm_start, pcm_len)) {
        ESP_LOGE(TAG, "Failed to get PCM data for %s", meta->filename);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!pcm_start || pcm_len == 0) {
        ESP_LOGE(TAG, "Invalid PCM data for %s", meta->filename);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

// ============================================================================
// 播放音频文件（内部实现）
// ============================================================================

esp_err_t audio_playback::play_audio_file_impl(audio_file_type_t audio_type, bool check_stop_signal, float duration_limit_seconds)
{
    if (audio_type < 0 || audio_type >= AUDIO_FILE_MAX) {
        ESP_LOGE(TAG, "Invalid audio file type: %d (valid range: 0-%d)",
                 audio_type, AUDIO_FILE_MAX - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (!parent_->play_dev || !parent_->es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_audio_file_available(audio_type)) {
        ESP_LOGE(TAG, "Audio file %s is not available", get_audio_file_name(audio_type));
        return ESP_ERR_NOT_FOUND;
    }

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Playing audio file: %s (limited to %.1f seconds)", get_audio_file_name(audio_type), duration_limit_seconds);
    } else {
        ESP_LOGI(TAG, "Playing audio file: %s (full file)", get_audio_file_name(audio_type));
    }

    const uint8_t *pcm_start = nullptr;
    size_t pcm_len = 0;
    uint32_t file_sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    audio_channels_t file_channels = parent_->tx_channels;
    i2s_data_bit_width_t file_bits = parent_->bits_per_sample;

    esp_err_t ret = get_pcm_data_and_format(audio_type, pcm_start, pcm_len,
                                             file_sample_rate_hz, file_channels, file_bits);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "PCM data size: %zu bytes", pcm_len);

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(parent_->bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(parent_->tx_channels);

    uint8_t* converted_buffer = nullptr;
    size_t converted_size = 0;

    ret = remix_convert_pcm_to_format(pcm_start, pcm_len,
                                      file_sample_rate_hz,
                                      static_cast<uint32_t>(file_channels),
                                      bits_to_audio_data_type(static_cast<uint32_t>(file_bits)),
                                      system_sample_rate_hz, system_channels,
                                      bits_to_audio_data_type(system_bits),
                                      &converted_buffer, &converted_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert audio format: %s", esp_err_to_name(ret));
        return ret;
    }

    const bool using_converted = converted_buffer != nullptr;
    uint8_t* playback_buffer = using_converted ? converted_buffer : const_cast<uint8_t*>(pcm_start);
    const size_t playback_size = using_converted ? converted_size : pcm_len;

    if (using_converted) {
        ESP_LOGI(TAG, "Audio format converted: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch (%zu -> %zu bytes)",
                 file_sample_rate_hz, static_cast<uint32_t>(file_bits), static_cast<uint32_t>(file_channels),
                 system_sample_rate_hz, system_bits, system_channels, pcm_len, playback_size);
    } else {
        ESP_LOGI(TAG, "Audio format matches system configuration");
    }

    size_t bytes_to_play = playback_size;
    if (duration_limit_seconds > 0) {
        const uint32_t bytes_per_frame = (system_bits * system_channels) / 8;
        if (bytes_per_frame == 0 || system_sample_rate_hz == 0) {
            ESP_LOGW(TAG, "Duration limit skipped due to invalid system format");
        } else {
            const size_t bytes_per_second = static_cast<size_t>(system_sample_rate_hz) * bytes_per_frame;
            const size_t limited_bytes = static_cast<size_t>(bytes_per_second * duration_limit_seconds);
            if (limited_bytes < bytes_to_play) {
                bytes_to_play = limited_bytes;
                ESP_LOGI(TAG, "Duration limit: will play %zu bytes out of %zu bytes", bytes_to_play, playback_size);
            }
        }
    }

    const size_t CHUNK_SIZE = 4096;
    size_t bytes_written = 0;
    ret = ESP_OK;

    while (bytes_written < bytes_to_play) {
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Received stop signal during playback, stopping gracefully...");
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
        }

        size_t remaining = bytes_to_play - bytes_written;
        size_t to_write = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        ret = esp_codec_dev_write(parent_->play_dev, playback_buffer + bytes_written, to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write audio chunk at offset %zu: %s", bytes_written, esp_err_to_name(ret));
            if (using_converted) heap_caps_free(converted_buffer);
            return ret;
        }

        bytes_written += to_write;
    }

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Playback interrupted by stop signal at %zu/%zu bytes", bytes_written, bytes_to_play);
        float saved_volume = parent_->volume;
        parent_->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(20));
        clear_audio_pipeline(150);
        parent_->set_volume(saved_volume);
        if (using_converted) heap_caps_free(converted_buffer);
        return ret;
    }

    ESP_LOGI(TAG, "Audio playback completed successfully (%zu bytes)", bytes_written);
    float saved_volume = parent_->volume;
    parent_->set_volume(10.0);
    vTaskDelay(pdMS_TO_TICKS(20));
    clear_audio_pipeline(200);
    parent_->set_volume(saved_volume);

    if (using_converted) heap_caps_free(converted_buffer);
    return ESP_OK;
}

// ============================================================================
// 播放缓冲区（内部实现）
// ============================================================================

esp_err_t audio_playback::play_audio_buffer_impl(const uint8_t* buffer, size_t buffer_size,
                                                  uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels,
                                                  i2s_data_bit_width_t buffer_bits,
                                                  bool check_stop_signal, float duration_limit_seconds)
{
    if (!buffer || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer_sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", buffer_sample_rate_hz);
        return ESP_ERR_INVALID_ARG;
    }

    if (!parent_->play_dev || !parent_->es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - limited to %.1f seconds",
                 buffer_size, buffer_sample_rate_hz,
                 static_cast<uint32_t>(buffer_channels),
                 static_cast<uint32_t>(buffer_bits),
                 duration_limit_seconds);
    } else {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - full buffer",
                 buffer_size, buffer_sample_rate_hz,
                 static_cast<uint32_t>(buffer_channels),
                 static_cast<uint32_t>(buffer_bits));
    }

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(parent_->bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(parent_->tx_channels);

    uint8_t* converted_buffer = nullptr;
    size_t converted_size = 0;
    esp_err_t ret = ESP_OK;

    const bool need_conversion = (buffer_sample_rate_hz != system_sample_rate_hz) ||
                                  (static_cast<uint32_t>(buffer_channels) != system_channels) ||
                                  (static_cast<uint32_t>(buffer_bits) != system_bits);

    if (need_conversion) {
        ret = remix_convert_pcm_to_format(buffer, buffer_size,
                                          buffer_sample_rate_hz,
                                          static_cast<uint32_t>(buffer_channels),
                                          bits_to_audio_data_type(static_cast<uint32_t>(buffer_bits)),
                                          system_sample_rate_hz, system_channels,
                                          bits_to_audio_data_type(system_bits),
                                          &converted_buffer, &converted_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert audio format: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Audio format converted: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch (%zu -> %zu bytes)",
                 buffer_sample_rate_hz, static_cast<uint32_t>(buffer_bits), static_cast<uint32_t>(buffer_channels),
                 system_sample_rate_hz, system_bits, system_channels, buffer_size, converted_size);
    }

    const bool using_converted = converted_buffer != nullptr;
    const uint8_t* playback_buffer = using_converted ? converted_buffer : buffer;
    size_t playback_size = using_converted ? converted_size : buffer_size;

    size_t bytes_to_play = playback_size;
    if (duration_limit_seconds > 0) {
        const uint32_t bytes_per_frame = (system_bits * system_channels) / 8;
        if (bytes_per_frame > 0 && system_sample_rate_hz > 0) {
            const size_t bytes_per_second = static_cast<size_t>(system_sample_rate_hz) * bytes_per_frame;
            const size_t limited_bytes = static_cast<size_t>(bytes_per_second * duration_limit_seconds);
            if (limited_bytes < bytes_to_play) {
                bytes_to_play = limited_bytes;
                ESP_LOGI(TAG, "Duration limit: will play %zu bytes out of %zu bytes", bytes_to_play, playback_size);
            }
        }
    }

    const size_t CHUNK_SIZE = 4096;
    size_t bytes_written = 0;
    ret = ESP_OK;

    while (bytes_written < bytes_to_play) {
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Received stop signal during playback, stopping gracefully...");
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
        }

        size_t remaining = bytes_to_play - bytes_written;
        size_t to_write = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        ret = esp_codec_dev_write(parent_->play_dev, const_cast<uint8_t*>(playback_buffer + bytes_written), to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write audio chunk at offset %zu: %s", bytes_written, esp_err_to_name(ret));
            if (using_converted) heap_caps_free(converted_buffer);
            return ret;
        }

        bytes_written += to_write;
    }

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Playback interrupted by stop signal at %zu/%zu bytes", bytes_written, bytes_to_play);
        float saved_volume = parent_->volume;
        parent_->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(20));
        clear_audio_pipeline(150);
        parent_->set_volume(saved_volume);
        if (using_converted) heap_caps_free(converted_buffer);
        return ret;
    }

    ESP_LOGI(TAG, "Audio buffer playback completed successfully (%zu bytes)", bytes_written);
    float saved_volume = parent_->volume;
    parent_->set_volume(10.0);
    vTaskDelay(pdMS_TO_TICKS(20));
    clear_audio_pipeline(200);
    parent_->set_volume(saved_volume);

    if (using_converted) heap_caps_free(converted_buffer);
    return ESP_OK;
}

// ============================================================================
// Public 播放方法
// ============================================================================

esp_err_t audio_playback::play_audio_file(audio_file_type_t audio_type, audio_playback_mode_t mode, float duration_limit_seconds)
{
    if (mode == AUDIO_PLAYBACK_BLOCKING) {
        return play_audio_file_impl(audio_type, false, duration_limit_seconds);
    }

    if (playback_task_handle_) {
        ESP_LOGW(TAG, "Playback task already running");
        return ESP_ERR_INVALID_STATE;
    }

    playback_task_args* args = static_cast<playback_task_args*>(calloc(1, sizeof(playback_task_args)));
    if (!args) {
        ESP_LOGE(TAG, "Failed to allocate playback task args");
        return ESP_ERR_NO_MEM;
    }

    args->instance = this;
    args->audio_type = audio_type;
    args->duration_limit_seconds = duration_limit_seconds;

    BaseType_t task_ret = xTaskCreate(playback_task_entry, "audio_play_task", 4096, args, 5, &playback_task_handle_);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create playback task");
        free(args);
        playback_task_handle_ = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Async playback task started for %s", get_audio_file_name(audio_type));
    return ESP_OK;
}

esp_err_t audio_playback::play_audio_buffer(const uint8_t* buffer, size_t buffer_size,
                                             uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels,
                                             i2s_data_bit_width_t buffer_bits,
                                             audio_playback_mode_t mode,
                                             float duration_limit_seconds)
{
    if (!buffer || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer_sample_rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", buffer_sample_rate_hz);
        return ESP_ERR_INVALID_ARG;
    }

    if (mode == AUDIO_PLAYBACK_BLOCKING) {
        return play_audio_buffer_impl(buffer, buffer_size,
                                       buffer_sample_rate_hz, buffer_channels, buffer_bits,
                                       false, duration_limit_seconds);
    }

    if (playback_task_handle_) {
        ESP_LOGW(TAG, "Playback task already running");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t* buffer_copy = static_cast<uint8_t*>(heap_caps_malloc(buffer_size, MALLOC_CAP_8BIT));
    if (!buffer_copy) {
        ESP_LOGE(TAG, "Failed to allocate buffer copy for async playback (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }
    memcpy(buffer_copy, buffer, buffer_size);

    buffer_playback_task_args* args = static_cast<buffer_playback_task_args*>(calloc(1, sizeof(buffer_playback_task_args)));
    if (!args) {
        ESP_LOGE(TAG, "Failed to allocate buffer playback task args");
        heap_caps_free(buffer_copy);
        return ESP_ERR_NO_MEM;
    }

    args->instance = this;
    args->buffer = buffer_copy;
    args->buffer_size = buffer_size;
    args->buffer_sample_rate_hz = buffer_sample_rate_hz;
    args->buffer_channels = buffer_channels;
    args->buffer_bits = buffer_bits;
    args->duration_limit_seconds = duration_limit_seconds;
    args->own_buffer = true;

    BaseType_t task_ret = xTaskCreate(buffer_playback_task_entry, "audio_buf_play", 4096, args, 5, &playback_task_handle_);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create buffer playback task");
        heap_caps_free(buffer_copy);
        free(args);
        playback_task_handle_ = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Async buffer playback task started (%zu bytes, %u Hz, %u ch, %u bit)",
             buffer_size, buffer_sample_rate_hz,
             static_cast<uint32_t>(buffer_channels),
             static_cast<uint32_t>(buffer_bits));
    return ESP_OK;
}

// ============================================================================
// 异步播放任务
// ============================================================================

void audio_playback::playback_task_entry(void* param)
{
    auto* args = static_cast<playback_task_args*>(param);
    audio_playback* instance = args->instance;
    audio_file_type_t audio_type = args->audio_type;
    float duration_limit_seconds = args->duration_limit_seconds;
    free(args);

    ESP_LOGI(TAG, "Playback task started, ready to receive stop signals");

    esp_err_t result = instance->play_audio_file_impl(audio_type, true, duration_limit_seconds);

    if (result == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Playback interrupted by stop request, performing cleanup...");
        float original_volume = instance->parent_->volume;
        instance->parent_->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(30));
        esp_err_t clear_ret = instance->clear_audio_pipeline(200);
        if (clear_ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(30));
            instance->clear_audio_pipeline(100);
        }
        instance->parent_->set_volume(original_volume);
        ESP_LOGI(TAG, "Playback stopped and cleanup completed");
    } else if (result != ESP_OK) {
        ESP_LOGE(TAG, "Async playback failed for %s: %s",
                 instance->get_audio_file_name(audio_type), esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Playback task completed normally");
    }

    instance->playback_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

void audio_playback::buffer_playback_task_entry(void* param)
{
    auto* args = static_cast<buffer_playback_task_args*>(param);
    audio_playback* instance = args->instance;
    const uint8_t* buffer = args->buffer;
    size_t buffer_size = args->buffer_size;
    uint32_t buffer_sample_rate_hz = args->buffer_sample_rate_hz;
    audio_channels_t buffer_channels = args->buffer_channels;
    i2s_data_bit_width_t buffer_bits = args->buffer_bits;
    float duration_limit_seconds = args->duration_limit_seconds;
    bool own_buffer = args->own_buffer;
    free(args);

    ESP_LOGI(TAG, "Buffer playback task started, ready to receive stop signals");

    esp_err_t result = instance->play_audio_buffer_impl(buffer, buffer_size,
                                                         buffer_sample_rate_hz, buffer_channels, buffer_bits,
                                                         true, duration_limit_seconds);

    if (result == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Buffer playback interrupted by stop request, performing cleanup...");
        float original_volume = instance->parent_->volume;
        instance->parent_->set_volume(0.0);
        vTaskDelay(pdMS_TO_TICKS(30));
        esp_err_t clear_ret = instance->clear_audio_pipeline(200);
        if (clear_ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(30));
            instance->clear_audio_pipeline(100);
        }
        instance->parent_->set_volume(original_volume);
        ESP_LOGI(TAG, "Buffer playback stopped and cleanup completed");
    } else if (result != ESP_OK) {
        ESP_LOGE(TAG, "Async buffer playback failed: %s", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Buffer playback task completed normally");
    }

    if (own_buffer && buffer) {
        heap_caps_free(const_cast<uint8_t*>(buffer));
    }

    instance->playback_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// 停止异步播放
// ============================================================================

esp_err_t audio_playback::stop_async_playback()
{
    TaskHandle_t task_to_stop = nullptr;

    if (parent_->audio_mutex && xSemaphoreTake(parent_->audio_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire audio mutex for stop");
        return ESP_ERR_TIMEOUT;
    }

    if (!playback_task_handle_) {
        ESP_LOGI(TAG, "No async playback task running");
        if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);
        return ESP_OK;
    }

    task_to_stop = playback_task_handle_;
    ESP_LOGI(TAG, "Sending stop signal to playback task...");

    BaseType_t notify_result = xTaskNotify(task_to_stop, 1, eSetValueWithOverwrite);
    if (notify_result != pdPASS) {
        ESP_LOGW(TAG, "Failed to send stop signal, notify result=%ld", (long)notify_result);
        if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);
        return ESP_FAIL;
    }

    if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);

    ESP_LOGI(TAG, "Stop signal sent, waiting for task to finish...");

    const TickType_t max_wait = pdMS_TO_TICKS(3000);
    TickType_t start_tick = xTaskGetTickCount();

    while (playback_task_handle_ != nullptr) {
        if ((xTaskGetTickCount() - start_tick) > max_wait) {
            ESP_LOGW(TAG, "Playback task did not exit gracefully within timeout");
            playback_task_handle_ = nullptr;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Playback task exited gracefully");
    return ESP_OK;
}

// ============================================================================
// 清理音频管道
// ============================================================================

esp_err_t audio_playback::clear_audio_pipeline(uint32_t silence_duration_ms)
{
    if (parent_->audio_mutex && xSemaphoreTake(parent_->audio_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire audio mutex for pipeline clear");
        return ESP_ERR_TIMEOUT;
    }

    if (!parent_->play_dev || !parent_->es8311_initialized) {
        ESP_LOGW(TAG, "Playback device not ready, skipping pipeline clear");
        if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Clearing audio pipeline with %lu ms silence...", silence_duration_ms);

    uint32_t sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    uint32_t channels = static_cast<uint32_t>(parent_->tx_channels);
    uint32_t bits_per_sample_val = static_cast<uint32_t>(parent_->bits_per_sample);
    uint32_t bytes_per_sample = (bits_per_sample_val * channels) / 8;
    size_t silence_size = (sample_rate_hz * bytes_per_sample * silence_duration_ms) / 1000;

    if (silence_size == 0) {
        if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);
        ESP_LOGW(TAG, "Silence size calculated as 0, skipping pipeline clear");
        return ESP_OK;
    }

    size_t chunk_capacity = SILENCE_CHUNK_CAPACITY - (SILENCE_CHUNK_CAPACITY % bytes_per_sample);
    if (chunk_capacity == 0) chunk_capacity = bytes_per_sample;

    esp_codec_dev_handle_t local_play_dev = parent_->play_dev;
    size_t remaining = silence_size;
    esp_err_t ret = ESP_OK;

    while (remaining > 0) {
        size_t send_size = remaining < chunk_capacity ? remaining : chunk_capacity;
        ret = esp_codec_dev_write(local_play_dev, g_silence_chunk, send_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write silence chunk (%zu bytes): %s", send_size, esp_err_to_name(ret));
            break;
        }
        remaining -= send_size;
    }

    if (parent_->audio_mutex) xSemaphoreGive(parent_->audio_mutex);

    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(silence_duration_ms + 50));
    ESP_LOGI(TAG, "Audio pipeline cleared successfully");
    return ESP_OK;
}

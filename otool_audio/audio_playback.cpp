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
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "audio_playback";
static constexpr size_t SILENCE_CHUNK_CAPACITY = 1024;
static constexpr size_t EXTERNAL_AUDIO_READ_CHUNK_SIZE = 32 * 1024;
static constexpr TickType_t EXTERNAL_AUDIO_READ_YIELD_TICKS = 1;
static uint8_t g_silence_chunk[SILENCE_CHUNK_CAPACITY] = {0};

// ============================================================================
// 音频文件嵌入声明 - 统一命名规范
// ============================================================================
#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start[];
extern const uint8_t _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start[];
extern const uint8_t _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_STARTUP_1CH_16K_16BIT_4S
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end[];
#endif

#ifdef USE_EMBEDDED_AUDIO_STARTUP_2CH_16K_16BIT_4S
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_start[];
extern const uint8_t _binary_startup_pcm_2ch_16k_16bit_4s_pcm_end[];
#endif

// ============================================================================
// 音频文件元数据表
// ============================================================================
typedef bool (*AudioDataGetter)(const uint8_t*& start, size_t& len);

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
static bool get_candy_wind_1ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
    len = _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_1ch_16k_16bit_9s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
static bool get_candy_wind_1ch_44k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start;
    len = _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_end - _binary_candy_wind_pcm_1ch_44_1k_16bit_45s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
static bool get_candy_wind_2ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
    len = _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_end - _binary_candy_wind_pcm_2ch_16k_16bit_9s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
static bool get_candy_wind_2ch_44k_data(const uint8_t*& start, size_t& len) {
    start = _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start;
    len = _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_end - _binary_candy_wind_pcm_2ch_44_1k_16bit_45s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
static bool get_sine_440hz_2ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
    len = _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_end - _binary_sine_440Hz_pcm_2ch_16k_16bit_10s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_STARTUP_1CH_16K_16BIT_4S
static bool get_startup_1ch_16k_data(const uint8_t*& start, size_t& len) {
    start = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
    len = _binary_startup_pcm_1ch_16k_16bit_4s_pcm_end - _binary_startup_pcm_1ch_16k_16bit_4s_pcm_start;
    return true;
}
#endif

#ifdef USE_EMBEDDED_AUDIO_STARTUP_2CH_16K_16BIT_4S
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
    const char* external_path;
};

static const AudioFileMetadata AUDIO_FILE_TABLE[] = {
#ifdef USE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S
    {AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S, "candy_wind_pcm_1ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_1ch_16k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S)
    {AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S, "candy_wind_pcm_1ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S},
#else
#error "Audio source is not configured for AUDIO_CANDY_WIND_1CH_16K_16BIT_9S"
#endif
#endif
#ifdef USE_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S
    {AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S, "candy_wind_pcm_1ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_1ch_44k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S)
    {AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S, "candy_wind_pcm_1ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S},
#else
#error "Audio source is not configured for AUDIO_CANDY_WIND_1CH_44K_16BIT_45S"
#endif
#endif
#ifdef USE_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S
    {AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S, "candy_wind_pcm_2ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_2ch_16k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S)
    {AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S, "candy_wind_pcm_2ch_16k_16bit_9s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S},
#else
#error "Audio source is not configured for AUDIO_CANDY_WIND_2CH_16K_16BIT_9S"
#endif
#endif
#ifdef USE_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
#ifdef USE_EMBEDDED_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S
    {AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S, "candy_wind_pcm_2ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_candy_wind_2ch_44k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S)
    {AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S, "candy_wind_pcm_2ch_44.1k_16bit_45s.pcm", 44100, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S},
#else
#error "Audio source is not configured for AUDIO_CANDY_WIND_2CH_44K_16BIT_45S"
#endif
#endif
#ifdef USE_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
#ifdef USE_EMBEDDED_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S
    {AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S, "sine_440Hz_pcm_2ch_16k_16bit_10s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_sine_440hz_2ch_16k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S)
    {AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S, "sine_440Hz_pcm_2ch_16k_16bit_10s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S},
#else
#error "Audio source is not configured for AUDIO_SINE_440HZ_2CH_16K_16BIT_10S"
#endif
#endif
#ifdef USE_AUDIO_STARTUP_1CH_16K_16BIT_4S
#ifdef USE_EMBEDDED_AUDIO_STARTUP_1CH_16K_16BIT_4S
    {AUDIO_FILE_STARTUP_1CH_16K_16B_4S, "startup_pcm_1ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, get_startup_1ch_16k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_STARTUP_1CH_16K_16BIT_4S)
    {AUDIO_FILE_STARTUP_1CH_16K_16B_4S, "startup_pcm_1ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_MONO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_STARTUP_1CH_16K_16BIT_4S},
#else
#error "Audio source is not configured for AUDIO_STARTUP_1CH_16K_16BIT_4S"
#endif
#endif
#ifdef USE_AUDIO_STARTUP_2CH_16K_16BIT_4S
#ifdef USE_EMBEDDED_AUDIO_STARTUP_2CH_16K_16BIT_4S
    {AUDIO_FILE_STARTUP_2CH_16K_16B_4S, "startup_pcm_2ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, get_startup_2ch_16k_data, nullptr},
#elif defined(USE_EXTERNAL_AUDIO_STARTUP_2CH_16K_16BIT_4S)
    {AUDIO_FILE_STARTUP_2CH_16K_16B_4S, "startup_pcm_2ch_16k_16bit_4s.pcm", 16000, AUDIO_CHANNELS_STEREO, I2S_DATA_BIT_WIDTH_16BIT, nullptr, OTOOL_AUDIO_FILE_PATH_AUDIO_STARTUP_2CH_16K_16BIT_4S},
#else
#error "Audio source is not configured for AUDIO_STARTUP_2CH_16K_16BIT_4S"
#endif
#endif
};

static constexpr size_t AUDIO_FILE_COUNT = sizeof(AUDIO_FILE_TABLE) / sizeof(AUDIO_FILE_TABLE[0]);

struct WavePcmPayload {
    const uint8_t* data = nullptr;
    size_t size = 0;
    uint32_t sample_rate = 0;
    uint16_t channels = 0;
    uint16_t bits_per_sample = 0;
};

static uint16_t read_u16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

static uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

static bool chunk_id_equals(const uint8_t* data, const char id[5])
{
    return memcmp(data, id, 4) == 0;
}

static esp_err_t extract_wave_pcm_payload(const uint8_t* file_data,
                                          size_t file_size,
                                          bool& is_wave,
                                          WavePcmPayload& payload)
{
    is_wave = false;
    payload = {};

    if (!file_data || file_size < 4 || !chunk_id_equals(file_data, "RIFF")) {
        return ESP_OK;
    }

    is_wave = true;
    if (file_size < 12 || !chunk_id_equals(file_data + 8, "WAVE")) {
        ESP_LOGE(TAG, "RIFF audio data is missing the WAVE signature");
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint32_t riff_size = read_u32_le(file_data + 4);
    if (riff_size < 4 || riff_size > file_size - 8) {
        ESP_LOGE(TAG, "Invalid RIFF size: declared=%u available=%zu",
                 static_cast<unsigned>(riff_size), file_size - 8);
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t riff_end = static_cast<size_t>(riff_size) + 8;
    size_t offset = 12;
    bool fmt_found = false;
    bool data_found = false;
    uint16_t block_align = 0;
    uint32_t byte_rate = 0;

    while (offset <= riff_end && riff_end - offset >= 8) {
        const uint8_t* chunk_header = file_data + offset;
        const uint32_t chunk_size_u32 = read_u32_le(chunk_header + 4);
        const size_t chunk_size = static_cast<size_t>(chunk_size_u32);
        const size_t chunk_data_offset = offset + 8;

        if (chunk_size > riff_end - chunk_data_offset) {
            ESP_LOGE(TAG, "Truncated WAVE chunk %.4s: size=%u remaining=%zu",
                     reinterpret_cast<const char*>(chunk_header),
                     static_cast<unsigned>(chunk_size_u32),
                     riff_end - chunk_data_offset);
            return ESP_ERR_INVALID_SIZE;
        }

        const uint8_t* chunk_data = file_data + chunk_data_offset;
        if (!fmt_found && chunk_id_equals(chunk_header, "fmt ")) {
            if (chunk_size < 16) {
                ESP_LOGE(TAG, "WAVE fmt chunk is too small: %zu", chunk_size);
                return ESP_ERR_INVALID_SIZE;
            }

            const uint16_t format_tag = read_u16_le(chunk_data);
            payload.channels = read_u16_le(chunk_data + 2);
            payload.sample_rate = read_u32_le(chunk_data + 4);
            byte_rate = read_u32_le(chunk_data + 8);
            block_align = read_u16_le(chunk_data + 12);
            payload.bits_per_sample = read_u16_le(chunk_data + 14);

            if (format_tag != 1) {
                ESP_LOGE(TAG, "Unsupported WAVE format tag: 0x%04X", format_tag);
                return ESP_ERR_NOT_SUPPORTED;
            }
            if ((payload.channels != 1 && payload.channels != 2) ||
                payload.sample_rate == 0 ||
                (payload.bits_per_sample != 8 &&
                 payload.bits_per_sample != 16 &&
                 payload.bits_per_sample != 32)) {
                ESP_LOGE(TAG, "Unsupported WAVE PCM format: %u Hz, %u bit, %u ch",
                         static_cast<unsigned>(payload.sample_rate),
                         static_cast<unsigned>(payload.bits_per_sample),
                         static_cast<unsigned>(payload.channels));
                return ESP_ERR_NOT_SUPPORTED;
            }

            const uint32_t expected_block_align =
                static_cast<uint32_t>(payload.channels) * payload.bits_per_sample / 8;
            const uint64_t expected_byte_rate =
                static_cast<uint64_t>(payload.sample_rate) * expected_block_align;
            if (block_align != expected_block_align || byte_rate != expected_byte_rate) {
                ESP_LOGE(TAG,
                         "Inconsistent WAVE PCM format: block_align=%u/%u byte_rate=%u/%llu",
                         static_cast<unsigned>(block_align),
                         static_cast<unsigned>(expected_block_align),
                         static_cast<unsigned>(byte_rate),
                         static_cast<unsigned long long>(expected_byte_rate));
                return ESP_ERR_INVALID_RESPONSE;
            }
            fmt_found = true;
        } else if (!data_found && chunk_id_equals(chunk_header, "data")) {
            payload.data = chunk_data;
            payload.size = chunk_size;
            data_found = true;
        }

        size_t next_offset = chunk_data_offset + chunk_size;
        if ((chunk_size & 1U) != 0U) {
            if (next_offset >= riff_end) {
                ESP_LOGE(TAG, "WAVE chunk %.4s is missing its padding byte",
                         reinterpret_cast<const char*>(chunk_header));
                return ESP_ERR_INVALID_SIZE;
            }
            ++next_offset;
        }
        offset = next_offset;

        if (fmt_found && data_found) {
            break;
        }
    }

    if (!fmt_found || !data_found || !payload.data || payload.size == 0) {
        ESP_LOGE(TAG, "WAVE file is missing a valid fmt or data chunk");
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (block_align == 0 || (payload.size % block_align) != 0) {
        ESP_LOGE(TAG, "WAVE data size is not frame-aligned: size=%zu block_align=%u",
                 payload.size, static_cast<unsigned>(block_align));
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static bool can_stream_convert_i16(audio_data_type_t input_type,
                                   audio_data_type_t target_type,
                                   uint32_t input_channels,
                                   uint32_t target_channels)
{
    return input_type == AUDIO_TYPE_INT16 &&
           (target_type == AUDIO_TYPE_INT16 || target_type == AUDIO_TYPE_INT32) &&
           (input_channels == 1 || input_channels == 2) &&
           (target_channels == 1 || target_channels == 2);
}

static uint8_t* alloc_playback_chunk(size_t bytes)
{
    uint8_t* buffer = static_cast<uint8_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buffer) {
        buffer = static_cast<uint8_t*>(
            heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!buffer) {
        buffer = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
    }
    return buffer;
}

static uint32_t frames_to_ms(uint64_t frames, uint32_t sample_rate)
{
    if (sample_rate == 0) {
        return 0;
    }
    return static_cast<uint32_t>((frames * 1000ULL) / sample_rate);
}

static uint32_t bytes_to_ms(size_t bytes,
                            uint32_t sample_rate,
                            uint32_t channels,
                            audio_data_type_t type)
{
    size_t sample_size = 0;
    switch (type) {
        case AUDIO_TYPE_INT8: sample_size = 1; break;
        case AUDIO_TYPE_INT16: sample_size = 2; break;
        case AUDIO_TYPE_INT32:
        case AUDIO_TYPE_FLOAT32: sample_size = 4; break;
        default: return 0;
    }

    const size_t frame_size = sample_size * channels;
    if (frame_size == 0) {
        return 0;
    }
    return frames_to_ms(bytes / frame_size, sample_rate);
}

static void set_progress_elapsed(volatile uint32_t* elapsed_ms,
                                 uint32_t value_ms,
                                 uint32_t duration_ms)
{
    if (!elapsed_ms) {
        return;
    }
    *elapsed_ms = (duration_ms > 0 && value_ms > duration_ms) ? duration_ms : value_ms;
}

static int32_t read_i16_sample_for_channel(const int16_t* input,
                                           size_t frame,
                                           uint32_t input_channels,
                                           uint32_t target_channels,
                                           uint32_t target_channel)
{
    if (input_channels == 1) {
        return input[frame];
    }

    const int16_t* source_frame = input + frame * input_channels;
    if (target_channels == 1) {
        return (static_cast<int32_t>(source_frame[0]) + static_cast<int32_t>(source_frame[1])) / 2;
    }

    return source_frame[target_channel];
}

static int32_t interpolate_i16_sample(const int16_t* input,
                                      size_t frame0,
                                      size_t frame1,
                                      uint32_t frac,
                                      uint32_t input_channels,
                                      uint32_t target_channels,
                                      uint32_t target_channel)
{
    const int32_t s0 = read_i16_sample_for_channel(input, frame0, input_channels, target_channels, target_channel);
    const int32_t s1 = read_i16_sample_for_channel(input, frame1, input_channels, target_channels, target_channel);
    const int64_t mixed = (static_cast<int64_t>(s0) << 32) +
                          static_cast<int64_t>(s1 - s0) * static_cast<int64_t>(frac);
    return static_cast<int32_t>(mixed >> 32);
}

static esp_err_t write_stream_converted_i16(esp_codec_dev_handle_t play_dev,
                                           const uint8_t* input_data,
                                           size_t input_size,
                                           uint32_t input_rate,
                                           uint32_t input_channels,
                                           uint32_t target_rate,
                                           uint32_t target_channels,
                                           audio_data_type_t target_type,
                                           bool check_stop_signal,
                                           float duration_limit_seconds,
                                           volatile uint32_t* progress_elapsed_ms,
                                           volatile uint32_t* progress_duration_ms,
                                           size_t& bytes_written_out,
                                           size_t& bytes_to_play_out)
{
    bytes_written_out = 0;
    bytes_to_play_out = 0;

    if (!play_dev || !input_data || input_size == 0 || input_rate == 0 || target_rate == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t input_frame_size = sizeof(int16_t) * input_channels;
    if (input_frame_size == 0 || (input_size % input_frame_size) != 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t target_sample_size = (target_type == AUDIO_TYPE_INT32) ? sizeof(int32_t) :
                                      (target_type == AUDIO_TYPE_INT16) ? sizeof(int16_t) : 0;
    if (target_sample_size == 0 || target_channels == 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const size_t input_frames = input_size / input_frame_size;
    if (input_frames == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint64_t output_frames = (static_cast<uint64_t>(input_frames) * target_rate + input_rate / 2) / input_rate;
    if (output_frames == 0) {
        output_frames = 1;
    }

    if (duration_limit_seconds > 0.0f) {
        const uint64_t limited_frames = static_cast<uint64_t>(duration_limit_seconds * static_cast<float>(target_rate));
        if (limited_frames > 0 && limited_frames < output_frames) {
            output_frames = limited_frames;
        }
    }

    const size_t target_frame_size = target_sample_size * target_channels;
    bytes_to_play_out = static_cast<size_t>(output_frames) * target_frame_size;
    const uint32_t duration_ms = frames_to_ms(output_frames, target_rate);
    if (progress_duration_ms) {
        *progress_duration_ms = duration_ms;
    }
    set_progress_elapsed(progress_elapsed_ms, 0, duration_ms);

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Duration limit: will stream %zu bytes", bytes_to_play_out);
    }

    constexpr size_t kOutputFramesPerChunk = 512;
    const size_t chunk_bytes = kOutputFramesPerChunk * target_frame_size;
    uint8_t* chunk = alloc_playback_chunk(chunk_bytes);
    if (!chunk) {
        ESP_LOGE(TAG, "Failed to allocate streaming playback chunk (%zu bytes)", chunk_bytes);
        return ESP_ERR_NO_MEM;
    }

    const int16_t* input_samples = reinterpret_cast<const int16_t*>(input_data);
    const uint64_t phase_step = (static_cast<uint64_t>(input_rate) << 32) / target_rate;
    uint64_t output_frame = 0;
    esp_err_t ret = ESP_OK;

    while (output_frame < output_frames) {
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Received stop signal during streaming conversion, stopping gracefully...");
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
        }

        const uint64_t frames_remaining = output_frames - output_frame;
        const size_t frames_this_chunk = static_cast<size_t>(
            frames_remaining < kOutputFramesPerChunk ? frames_remaining : kOutputFramesPerChunk);

        if (target_type == AUDIO_TYPE_INT32) {
            int32_t* out = reinterpret_cast<int32_t*>(chunk);
            for (size_t i = 0; i < frames_this_chunk; ++i) {
                const uint64_t phase = (output_frame + i) * phase_step;
                size_t frame0 = static_cast<size_t>(phase >> 32);
                if (frame0 >= input_frames) {
                    frame0 = input_frames - 1;
                }
                const size_t frame1 = (frame0 + 1 < input_frames) ? (frame0 + 1) : frame0;
                const uint32_t frac = static_cast<uint32_t>(phase & 0xffffffffu);

                for (uint32_t ch = 0; ch < target_channels; ++ch) {
                    const int32_t sample = interpolate_i16_sample(input_samples, frame0, frame1, frac,
                                                                  input_channels, target_channels, ch);
                    out[i * target_channels + ch] = static_cast<int32_t>(static_cast<int64_t>(sample) * 65536);
                }
            }
        } else {
            int16_t* out = reinterpret_cast<int16_t*>(chunk);
            for (size_t i = 0; i < frames_this_chunk; ++i) {
                const uint64_t phase = (output_frame + i) * phase_step;
                size_t frame0 = static_cast<size_t>(phase >> 32);
                if (frame0 >= input_frames) {
                    frame0 = input_frames - 1;
                }
                const size_t frame1 = (frame0 + 1 < input_frames) ? (frame0 + 1) : frame0;
                const uint32_t frac = static_cast<uint32_t>(phase & 0xffffffffu);

                for (uint32_t ch = 0; ch < target_channels; ++ch) {
                    const int32_t sample = interpolate_i16_sample(input_samples, frame0, frame1, frac,
                                                                  input_channels, target_channels, ch);
                    out[i * target_channels + ch] = static_cast<int16_t>(sample);
                }
            }
        }

        const size_t bytes_this_chunk = frames_this_chunk * target_frame_size;
        ret = esp_codec_dev_write(play_dev, chunk, bytes_this_chunk);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write converted audio chunk at frame %llu: %s",
                     static_cast<unsigned long long>(output_frame), esp_err_to_name(ret));
            break;
        }

        bytes_written_out += bytes_this_chunk;
        output_frame += frames_this_chunk;
        set_progress_elapsed(progress_elapsed_ms, frames_to_ms(output_frame, target_rate), duration_ms);
    }

    heap_caps_free(chunk);
    return ret;
}

static const AudioFileMetadata* find_audio_metadata(audio_file_type_t type) {
    for (size_t i = 0; i < AUDIO_FILE_COUNT; ++i) {
        if (AUDIO_FILE_TABLE[i].type == type) {
            return &AUDIO_FILE_TABLE[i];
        }
    }
    return nullptr;
}

struct ScopedAudioFileBuffer {
    uint8_t* data = nullptr;

    ~ScopedAudioFileBuffer()
    {
        if (data) {
            heap_caps_free(data);
        }
    }
};

static esp_err_t load_external_audio_file(const char* path,
                                          uint8_t*& file_data,
                                          size_t& file_size,
                                          bool check_stop_signal)
{
    file_data = nullptr;
    file_size = 0;
    if (!path || path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    FILE* file = fopen(path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open external audio file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        ESP_LOGE(TAG, "Failed to seek external audio file: %s", path);
        fclose(file);
        return ESP_FAIL;
    }
    const long length = ftell(file);
    if (length <= 0 || fseek(file, 0, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Invalid external audio file size for %s: %ld", path, length);
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t length_bytes = static_cast<size_t>(length);
    uint8_t* buffer = static_cast<uint8_t*>(
        heap_caps_malloc(length_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes in PSRAM for %s",
                 length_bytes, path);
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    size_t total_bytes_read = 0;
    while (total_bytes_read < length_bytes) {
        if (check_stop_signal) {
            uint32_t notification_value = 0;
            if (xTaskNotifyWait(0, 0, &notification_value, 0) == pdTRUE) {
                ESP_LOGI(TAG,
                         "External audio load interrupted at %zu/%zu bytes",
                         total_bytes_read,
                         length_bytes);
                fclose(file);
                heap_caps_free(buffer);
                return ESP_ERR_INVALID_STATE;
            }
        }

        const size_t remaining = length_bytes - total_bytes_read;
        const size_t bytes_to_read =
            remaining < EXTERNAL_AUDIO_READ_CHUNK_SIZE
                ? remaining
                : EXTERNAL_AUDIO_READ_CHUNK_SIZE;
        const size_t chunk_bytes_read =
            fread(buffer + total_bytes_read, 1, bytes_to_read, file);
        total_bytes_read += chunk_bytes_read;
        if (chunk_bytes_read != bytes_to_read) {
            ESP_LOGE(TAG,
                     "Failed to read external audio file %s: %zu/%zu bytes "
                     "(chunk %zu/%zu, error=%d, eof=%d)",
                     path,
                     total_bytes_read,
                     length_bytes,
                     chunk_bytes_read,
                     bytes_to_read,
                     ferror(file) != 0,
                     feof(file) != 0);
            fclose(file);
            heap_caps_free(buffer);
            return ESP_FAIL;
        }

        if (total_bytes_read < length_bytes) {
            // SPIFFS reads are synchronous. Block for one tick between chunks
            // so the core's Idle task can run and service the task watchdog.
            vTaskDelay(EXTERNAL_AUDIO_READ_YIELD_TICKS);
        }
    }

    const int close_result = fclose(file);
    if (close_result != 0) {
        ESP_LOGE(TAG, "Failed to close external audio file after reading: %s",
                 path);
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    file_data = buffer;
    file_size = length_bytes;
    ESP_LOGI(TAG, "Loaded external audio file %s into PSRAM (%zu bytes)",
             path, file_size);
    return ESP_OK;
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

esp_err_t audio_playback::get_audio_file_pcm(audio_file_type_t audio_type,
                                             const uint8_t*& pcm_start,
                                             size_t& pcm_len,
                                             uint32_t& file_sample_rate_hz,
                                             audio_channels_t& file_channels,
                                             i2s_data_bit_width_t& file_bits)
{
    return get_pcm_data_and_format(audio_type,
                                   pcm_start,
                                   pcm_len,
                                   file_sample_rate_hz,
                                   file_channels,
                                   file_bits);
}

bool audio_playback::is_audio_file_available(audio_file_type_t audio_type) const
{
    const AudioFileMetadata* meta = find_audio_metadata(audio_type);
    if (!meta) {
        return false;
    }
    if (!meta->external_path) {
        return meta->data_getter != nullptr;
    }

    struct stat info = {};
    return stat(meta->external_path, &info) == 0 && info.st_size > 0;
}

// ============================================================================
// 获取PCM数据和格式参数
// ============================================================================

esp_err_t audio_playback::get_pcm_data_and_format(audio_file_type_t audio_type,
                                                    const uint8_t*& pcm_start,
                                                    size_t& pcm_len,
                                                    uint32_t& file_sample_rate_hz,
                                                    audio_channels_t& file_channels,
                                                    i2s_data_bit_width_t& file_bits,
                                                    uint8_t** owned_file_buffer,
                                                    bool check_stop_signal)
{
    pcm_start = nullptr;
    pcm_len = 0;
    if (owned_file_buffer) {
        *owned_file_buffer = nullptr;
    }

    const AudioFileMetadata* meta = find_audio_metadata(audio_type);
    if (!meta) {
        ESP_LOGE(TAG, "Invalid audio file type: %d", audio_type);
        return ESP_ERR_INVALID_ARG;
    }

    file_sample_rate_hz = meta->sample_rate;
    file_channels = meta->channels;
    file_bits = meta->bits;

    if (meta->external_path) {
        if (!owned_file_buffer) {
            ESP_LOGE(TAG,
                     "%s is file-backed and cannot expose a persistent PCM pointer",
                     meta->filename);
            return ESP_ERR_NOT_SUPPORTED;
        }

        esp_err_t load_ret =
            load_external_audio_file(meta->external_path,
                                     *owned_file_buffer,
                                     pcm_len,
                                     check_stop_signal);
        if (load_ret != ESP_OK) {
            return load_ret;
        }
        pcm_start = *owned_file_buffer;
    } else if (!meta->data_getter) {
        ESP_LOGE(TAG, "%s not compiled in (data_getter is null)", meta->filename);
        return ESP_ERR_NOT_SUPPORTED;
    } else if (!meta->data_getter(pcm_start, pcm_len)) {
        ESP_LOGE(TAG, "Failed to get PCM data for %s", meta->filename);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!pcm_start || pcm_len == 0) {
        ESP_LOGE(TAG, "Invalid PCM data for %s", meta->filename);
        return ESP_ERR_INVALID_SIZE;
    }

    bool is_wave = false;
    WavePcmPayload wave_payload = {};
    esp_err_t ret = extract_wave_pcm_payload(pcm_start, pcm_len, is_wave, wave_payload);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse RIFF/WAVE data in %s: %s",
                 meta->filename, esp_err_to_name(ret));
        return ret;
    }

    if (is_wave) {
        const uint32_t metadata_rate = file_sample_rate_hz;
        const uint32_t metadata_channels = static_cast<uint32_t>(file_channels);
        const uint32_t metadata_bits = static_cast<uint32_t>(file_bits);

        file_sample_rate_hz = wave_payload.sample_rate;
        file_channels = static_cast<audio_channels_t>(wave_payload.channels);
        file_bits = static_cast<i2s_data_bit_width_t>(wave_payload.bits_per_sample);
        pcm_start = wave_payload.data;
        pcm_len = wave_payload.size;

        if (metadata_rate != file_sample_rate_hz ||
            metadata_channels != static_cast<uint32_t>(file_channels) ||
            metadata_bits != static_cast<uint32_t>(file_bits)) {
            ESP_LOGW(TAG,
                     "WAVE header overrides metadata for %s: %u Hz/%u bit/%u ch -> %u Hz/%u bit/%u ch",
                     meta->filename,
                     static_cast<unsigned>(metadata_rate),
                     static_cast<unsigned>(metadata_bits),
                     static_cast<unsigned>(metadata_channels),
                     static_cast<unsigned>(file_sample_rate_hz),
                     static_cast<unsigned>(file_bits),
                     static_cast<unsigned>(file_channels));
        }

        ESP_LOGI(TAG, "Parsed RIFF/WAVE payload for %s: data=%zu bytes, %u Hz, %u bit, %u ch",
                 meta->filename,
                 pcm_len,
                 static_cast<unsigned>(file_sample_rate_hz),
                 static_cast<unsigned>(file_bits),
                 static_cast<unsigned>(file_channels));
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

    playback_progress_valid_ = false;
    playback_elapsed_ms_ = 0;
    playback_duration_ms_ = 0;

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
    ScopedAudioFileBuffer owned_file_buffer;

    esp_err_t ret = get_pcm_data_and_format(audio_type, pcm_start, pcm_len,
                                             file_sample_rate_hz, file_channels,
                                             file_bits, &owned_file_buffer.data,
                                             check_stop_signal);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "PCM data size: %zu bytes", pcm_len);

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(parent_->bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(parent_->tx_channels);
    const audio_data_type_t file_type = bits_to_audio_data_type(static_cast<uint32_t>(file_bits));
    const audio_data_type_t system_type = bits_to_audio_data_type(system_bits);
    const bool need_conversion = (file_sample_rate_hz != system_sample_rate_hz) ||
                                 (static_cast<uint32_t>(file_channels) != system_channels) ||
                                 (static_cast<uint32_t>(file_bits) != system_bits);

    if (need_conversion && can_stream_convert_i16(file_type, system_type,
                                                  static_cast<uint32_t>(file_channels),
                                                  system_channels)) {
        ESP_LOGI(TAG, "Streaming audio format conversion: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch",
                 static_cast<unsigned>(file_sample_rate_hz),
                 static_cast<unsigned>(file_bits),
                 static_cast<unsigned>(file_channels),
                 static_cast<unsigned>(system_sample_rate_hz),
                 static_cast<unsigned>(system_bits),
                 static_cast<unsigned>(system_channels));

        size_t bytes_written = 0;
        size_t bytes_to_play = 0;
        playback_progress_valid_ = true;
        ret = write_stream_converted_i16(parent_->play_dev,
                                         pcm_start, pcm_len,
                                         file_sample_rate_hz,
                                         static_cast<uint32_t>(file_channels),
                                         system_sample_rate_hz,
                                         system_channels,
                                         system_type,
                                         check_stop_signal,
                                         duration_limit_seconds,
                                         &playback_elapsed_ms_,
                                         &playback_duration_ms_,
                                         bytes_written,
                                         bytes_to_play);

        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "Playback interrupted by stop signal at %zu/%zu bytes", bytes_written, bytes_to_play);
            float saved_volume = parent_->volume;
            parent_->set_volume(0.0);
            vTaskDelay(pdMS_TO_TICKS(20));
            clear_audio_pipeline(150);
            parent_->set_volume(saved_volume);
            return ret;
        }

        if (ret != ESP_OK) {
            return ret;
        }

        ESP_LOGI(TAG, "Audio playback completed successfully (%zu bytes)", bytes_written);
        float saved_volume = parent_->volume;
        parent_->set_volume(10.0);
        vTaskDelay(pdMS_TO_TICKS(20));
        clear_audio_pipeline(200);
        parent_->set_volume(saved_volume);
        return ESP_OK;
    }

    uint8_t* converted_buffer = nullptr;
    size_t converted_size = 0;

    ret = remix_convert_pcm_to_format(pcm_start, pcm_len,
                                      file_sample_rate_hz,
                                      static_cast<uint32_t>(file_channels),
                                      file_type,
                                      system_sample_rate_hz, system_channels,
                                      system_type,
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
                 static_cast<unsigned>(file_sample_rate_hz),
                 static_cast<unsigned>(file_bits),
                 static_cast<unsigned>(file_channels),
                 static_cast<unsigned>(system_sample_rate_hz),
                 static_cast<unsigned>(system_bits),
                 static_cast<unsigned>(system_channels),
                 pcm_len, playback_size);
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

    const uint32_t playback_duration_ms = bytes_to_ms(bytes_to_play,
                                                      system_sample_rate_hz,
                                                      system_channels,
                                                      system_type);
    playback_duration_ms_ = playback_duration_ms;
    playback_elapsed_ms_ = 0;
    playback_progress_valid_ = playback_duration_ms > 0;

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
        set_progress_elapsed(&playback_elapsed_ms_,
                             bytes_to_ms(bytes_written, system_sample_rate_hz,
                                         system_channels, system_type),
                             playback_duration_ms_);
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
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", static_cast<unsigned>(buffer_sample_rate_hz));
        return ESP_ERR_INVALID_ARG;
    }

    if (!parent_->play_dev || !parent_->es8311_initialized) {
        ESP_LOGE(TAG, "Playback device not ready");
        return ESP_ERR_INVALID_STATE;
    }

    playback_progress_valid_ = false;
    playback_elapsed_ms_ = 0;
    playback_duration_ms_ = 0;

    if (duration_limit_seconds > 0.0f) {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - limited to %.1f seconds",
                 buffer_size, static_cast<unsigned>(buffer_sample_rate_hz),
                 static_cast<unsigned>(buffer_channels),
                 static_cast<unsigned>(buffer_bits),
                 duration_limit_seconds);
    } else {
        ESP_LOGI(TAG, "Playing audio buffer (%zu bytes, %u Hz, %u ch, %u bit) - full buffer",
                 buffer_size, static_cast<unsigned>(buffer_sample_rate_hz),
                 static_cast<unsigned>(buffer_channels),
                 static_cast<unsigned>(buffer_bits));
    }

    const uint32_t system_sample_rate_hz = static_cast<uint32_t>(parent_->sample_rate);
    const uint32_t system_bits = static_cast<uint32_t>(parent_->bits_per_sample);
    const uint32_t system_channels = static_cast<uint32_t>(parent_->tx_channels);
    const audio_data_type_t buffer_type = bits_to_audio_data_type(static_cast<uint32_t>(buffer_bits));
    const audio_data_type_t system_type = bits_to_audio_data_type(system_bits);

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
                                           buffer_type,
                                           system_sample_rate_hz, system_channels,
                                           system_type,
                                           &converted_buffer, &converted_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert audio format: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Audio format converted: %u Hz, %u bit, %u ch -> %u Hz, %u bit, %u ch (%zu -> %zu bytes)",
                 static_cast<unsigned>(buffer_sample_rate_hz),
                 static_cast<unsigned>(buffer_bits),
                 static_cast<unsigned>(buffer_channels),
                 static_cast<unsigned>(system_sample_rate_hz),
                 static_cast<unsigned>(system_bits),
                 static_cast<unsigned>(system_channels),
                 buffer_size, converted_size);
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

    const uint32_t playback_duration_ms = bytes_to_ms(bytes_to_play,
                                                      system_sample_rate_hz,
                                                      system_channels,
                                                      system_type);
    playback_duration_ms_ = playback_duration_ms;
    playback_elapsed_ms_ = 0;
    playback_progress_valid_ = playback_duration_ms > 0;

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
        set_progress_elapsed(&playback_elapsed_ms_,
                             bytes_to_ms(bytes_written, system_sample_rate_hz,
                                         system_channels, system_type),
                             playback_duration_ms_);
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

bool audio_playback::get_playback_progress(uint32_t& elapsed_ms, uint32_t& duration_ms) const
{
    if (!playback_progress_valid_ || playback_duration_ms_ == 0) {
        elapsed_ms = 0;
        duration_ms = 0;
        return false;
    }

    duration_ms = playback_duration_ms_;
    elapsed_ms = playback_elapsed_ms_;
    if (elapsed_ms > duration_ms) {
        elapsed_ms = duration_ms;
    }
    return true;
}

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
        ESP_LOGE(TAG, "Invalid sample rate: %u Hz", static_cast<unsigned>(buffer_sample_rate_hz));
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
             buffer_size, static_cast<unsigned>(buffer_sample_rate_hz),
             static_cast<unsigned>(buffer_channels),
             static_cast<unsigned>(buffer_bits));
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

    ESP_LOGI(TAG, "Clearing audio pipeline with %lu ms silence...", static_cast<unsigned long>(silence_duration_ms));

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

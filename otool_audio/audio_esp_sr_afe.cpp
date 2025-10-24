/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "audio_tools.h"
#include "audio_remix_tools.h"

static const char *TAG_AEC = "audio_aec_test";

/**
 * @brief AEC测试函数 - 录音并回声消除后播放（支持自适应采样率）
 * 
 * 此函数实现了完整的AEC测试流程：
 * 1. 同时录制麦克风输入和参考音频（正在播放的音频）
 * 2. 自动检测录音采样率，如果不是16kHz则进行重采样
 * 3. 使用ESP-SR的AEC算法进行回声消除处理（AEC固定工作在16kHz）
 * 4. 将处理后的音频重采样回播放采样率
 * 5. 播放经过AEC处理后的音频
 * 
 * @param record_duration_seconds 录音时长（秒），默认5秒
 * @param filter_length AEC滤波器长度，推荐值：ESP32P4=4, ESP32C5=2，值越大CPU负载越高但效果越好
 * @param aec_mode AEC模式：AFE_MODE_LOW_COST（低功耗）或 AFE_MODE_HIGH_PERF（高性能）
 * @param play_original_audio 是否播放原始录音（用于对比），false则播放AEC处理后的音频
 * 
 * @return esp_err_t 返回操作结果
 *         - ESP_OK: 测试成功
 *         - ESP_ERR_INVALID_STATE: 音频系统未初始化
 *         - ESP_ERR_NO_MEM: 内存分配失败
 *         - 其他: 底层驱动错误
 */
esp_err_t audio_tools::aec_test(uint32_t record_duration_seconds, int filter_length, afe_mode_t aec_mode, bool play_original_audio)
{
    esp_err_t ret = ESP_OK;
    afe_aec_handle_t *aec_handle = nullptr;
    int16_t *mic_data_system = nullptr;
    int16_t *ref_data_system = nullptr;
    int16_t *output_data_system = nullptr;
    int16_t *aec_input = nullptr;
    int16_t *mic_data_aec = nullptr;
    int16_t *ref_data_aec = nullptr;
    int16_t *output_data_aec = nullptr;
    int16_t *resample_temp_mic = nullptr;
    int16_t *resample_temp_ref = nullptr;
    int16_t *resample_temp_output = nullptr;
    int16_t *dual_channel_buffer = nullptr;  // 用于硬件AEC双通道读取

    // 声明所有需要的变量（避免goto跳过初始化）
    int64_t start_time = 0;
    int64_t end_time = 0;
    int64_t processing_time_us = 0;
    float processing_time_ms = 0.0f;
    float real_time_factor = 0.0f;
    size_t samples_recorded_system = 0;
    int total_frames_system = 0;
    bool use_hardware_aec = false;  // 是否使用硬件AEC回采
    
    // AEC参数变量
    int frame_size = 0;
    int nch = 0;
    int mic_idx = 0;
    int ref_idx = 0;
    int frame_bytes_aec = 0;
    int frame_size_system = 0;
    int frame_bytes_system = 0;
    int total_frames_aec = 0;
    size_t total_samples_aec = 0;
    size_t total_samples_system = 0;

    if (!es7210_initialized || !es8311_initialized) {
        ESP_LOGE(TAG_AEC, "Audio devices not initialized (ES7210 or ES8311)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev || !play_dev) {
        ESP_LOGE(TAG_AEC, "Record or playback device not available");
        return ESP_ERR_INVALID_STATE;
    }

    // 获取当前系统的采样率配置
    uint32_t system_sample_rate = static_cast<uint32_t>(sample_rate);
    const uint32_t aec_sample_rate = 16000;  // AEC固定工作在16kHz
    bool need_resample = (system_sample_rate != aec_sample_rate);

    ESP_LOGI(TAG_AEC, "=== Starting AEC Test (Adaptive Sample Rate) ===");
    ESP_LOGI(TAG_AEC, "System Sample Rate: %lu Hz, AEC Sample Rate: %lu Hz", 
             system_sample_rate, aec_sample_rate);
    ESP_LOGI(TAG_AEC, "Duration: %lu seconds, Filter Length: %d, Mode: %s", 
             record_duration_seconds, filter_length, 
             aec_mode == AFE_MODE_LOW_COST ? "LOW_COST" : "HIGH_PERF");
    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Resampling enabled: %lu Hz -> %lu Hz (recording), %lu Hz -> %lu Hz (playback)",
                 system_sample_rate, aec_sample_rate, aec_sample_rate, system_sample_rate);
    }

    // 创建AEC实例
    // 输入格式: "MR" - M=麦克风通道, R=参考通道（播放的音频）
    aec_handle = afe_aec_create("MR", filter_length, AFE_TYPE_SR, aec_mode);
    if (!aec_handle) {
        ESP_LOGE(TAG_AEC, "Failed to create AEC handle");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // 获取AEC帧大小和配置参数
    frame_size = aec_handle->frame_size;  // 每帧的采样点数（16kHz）
    nch = aec_handle->pcm_config.total_ch_num;  // 总通道数（麦克风+参考）
    mic_idx = aec_handle->pcm_config.mic_ids[0];  // 麦克风通道索引
    ref_idx = aec_handle->pcm_config.ref_ids[0];  // 参考通道索引
    frame_bytes_aec = frame_size * sizeof(int16_t);

    ESP_LOGI(TAG_AEC, "AEC Config: frame_size=%d, channels=%d, mic_idx=%d, ref_idx=%d", 
             frame_size, nch, mic_idx, ref_idx);

    // 计算系统采样率下的帧大小
    frame_size_system = need_resample ? 
        (frame_size * system_sample_rate + aec_sample_rate / 2) / aec_sample_rate : frame_size;
    frame_bytes_system = frame_size_system * sizeof(int16_t);

    // 计算总帧数和缓冲区大小（基于AEC采样率）
    total_frames_aec = (record_duration_seconds * aec_sample_rate) / frame_size;
    total_samples_aec = total_frames_aec * frame_size;
    
    // 计算系统采样率下的总采样数
    total_samples_system = need_resample ?
        (total_samples_aec * system_sample_rate + aec_sample_rate / 2) / aec_sample_rate : total_samples_aec;

    ESP_LOGI(TAG_AEC, "Buffer allocation (AEC): total_frames=%d, total_samples=%zu", 
             total_frames_aec, total_samples_aec);
    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Buffer allocation (System): frame_size=%d, total_samples=%zu", 
                 frame_size_system, total_samples_system);
    }

    // 检查并分配音频缓冲区（使用SPIRAM以节省内部RAM）
    {
        size_t required_spiram = (total_samples_system * 3 + frame_bytes_aec * nch);
        if (need_resample) {
            required_spiram += total_samples_aec * 3;
        }
        required_spiram *= sizeof(int16_t);

        size_t available_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        if (available_spiram < required_spiram) {
            ESP_LOGE(TAG_AEC, "Insufficient SPIRAM. Required: %zu, Available: %zu", required_spiram, available_spiram);
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        mic_data_system = (int16_t *)heap_caps_aligned_calloc(16, total_samples_system, sizeof(int16_t), MALLOC_CAP_SPIRAM);
        ref_data_system = (int16_t *)heap_caps_aligned_calloc(16, total_samples_system, sizeof(int16_t), MALLOC_CAP_SPIRAM);
        output_data_system = (int16_t *)heap_caps_aligned_calloc(16, total_samples_system, sizeof(int16_t), MALLOC_CAP_SPIRAM);
        aec_input = (int16_t *)heap_caps_calloc(1, frame_bytes_aec * nch, MALLOC_CAP_SPIRAM);

        if (!mic_data_system || !ref_data_system || !output_data_system || !aec_input) {
            ESP_LOGE(TAG_AEC, "Failed to allocate system/AEC buffers");
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        if (need_resample) {
            mic_data_aec = (int16_t *)heap_caps_aligned_calloc(16, total_samples_aec, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            ref_data_aec = (int16_t *)heap_caps_aligned_calloc(16, total_samples_aec, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            output_data_aec = (int16_t *)heap_caps_aligned_calloc(16, total_samples_aec, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (!mic_data_aec || !ref_data_aec || !output_data_aec) {
                ESP_LOGE(TAG_AEC, "Failed to allocate resample buffers");
                ret = ESP_ERR_NO_MEM;
                goto cleanup;
            }
        } else {
            mic_data_aec = mic_data_system;
            ref_data_aec = ref_data_system;
            output_data_aec = output_data_system;
        }
    }

    ESP_LOGI(TAG_AEC, "Audio buffers allocated successfully in SPIRAM");

    // ========== 阶段1: 录音 ==========
    ESP_LOGI(TAG_AEC, "Phase 1: Recording %lu seconds of audio...", record_duration_seconds);
    
    // 检查是否启用了硬件AEC回采 (通道13 = 通道1 + 通道3)
    use_hardware_aec = (mic_channels == AUDIO_MIC_CHANNEL_13);
    
    if (use_hardware_aec) {
        // 确保 I2S RX 处于立体声模式，否则无法同时捕获 CH1 与 CH3
        if (rx_channels != AUDIO_CHANNELS_STEREO) {
            ESP_LOGW(TAG_AEC, "Hardware AEC requested (CH1+CH3), but I2S RX is MONO. Fallback to software reference.");
            ESP_LOGW(TAG_AEC, "To enable hardware AEC loopback, call es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_13, ES7210_TDM_DISABLED).");
            use_hardware_aec = false;
        } else {
            ESP_LOGI(TAG_AEC, "Hardware AEC loopback enabled: CH1(Mic) + CH3(Loopback) with I2S RX STEREO");
        }
    } else {
        ESP_LOGW(TAG_AEC, "Hardware AEC loopback NOT enabled. Using silent reference.");
        ESP_LOGW(TAG_AEC, "To enable real AEC, initialize ES7210 with AUDIO_MIC_CHANNEL_13");
    }
    
    // 计算系统采样率下的总帧数
    total_frames_system = (record_duration_seconds * system_sample_rate) / frame_size_system;

    if (use_hardware_aec) {
        // === 硬件AEC模式:从ES7210读取双通道数据(CH1=麦克风, CH3=回采) ===
        
        // 分配临时缓冲区用于读取交错的双通道数据
        dual_channel_buffer = (int16_t *)malloc(frame_bytes_system * 2); // 双通道
        if (!dual_channel_buffer) {
            ESP_LOGE(TAG_AEC, "Failed to allocate dual channel buffer");
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        
        for (int frame = 0; frame < total_frames_system; frame++) {
            // 从ES7210读取双通道交错数据 [CH1, CH3, CH1, CH3, ...]
            ret = esp_codec_dev_read(record_dev, (uint8_t*)dual_channel_buffer, frame_bytes_system * 2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Failed to read dual channel data at frame %d: %s", frame, esp_err_to_name(ret));
                goto cleanup;
            }

            // 分离通道:奇数索引=CH1(麦克风), 偶数索引=CH3(回采)
            // 注意:实际数据格式可能是 [CH1, CH3, CH1, CH3, ...] 或 [CH3, CH1, CH3, CH1, ...]
            // 需要根据实际硬件测试确定,这里假设 CH1 在前
            for (int i = 0; i < frame_size_system; i++) {
                mic_data_system[samples_recorded_system + i] = dual_channel_buffer[i * 2];     // CH1(Mic)
                ref_data_system[samples_recorded_system + i] = dual_channel_buffer[i * 2 + 1]; // CH3(Loopback)
            }

            samples_recorded_system += frame_size_system;

            // 进度提示
            if (total_frames_system > 10 && (frame + 1) % (total_frames_system / 10) == 0) {
                ESP_LOGI(TAG_AEC, "Recording progress: %d%%", ((frame + 1) * 100) / total_frames_system);
            }
        }
        
        ESP_LOGI(TAG_AEC, "Hardware AEC recording completed: CH1(Mic) and CH3(Loopback) captured");
        
    } else {
        // === 软件模式:仅读取麦克风,参考音频使用静音 ===
        
        for (int frame = 0; frame < total_frames_system; frame++) {
            // 从ES7210读取麦克风数据（单通道或多通道的第一通道）
            ret = esp_codec_dev_read(record_dev, (uint8_t*)(mic_data_system + samples_recorded_system), frame_bytes_system);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Failed to read mic data at frame %d: %s", frame, esp_err_to_name(ret));
                goto cleanup;
            }

            // 参考音频使用静音
            if (frame == 0) {
                ESP_LOGW(TAG_AEC, "Using silent reference audio. AEC will not cancel any actual echo.");
            }
            memset(ref_data_system + samples_recorded_system, 0, frame_bytes_system);

            samples_recorded_system += frame_size_system;

            // 进度提示
            if (total_frames_system > 10 && (frame + 1) % (total_frames_system / 10) == 0) {
                ESP_LOGI(TAG_AEC, "Recording progress: %d%%", ((frame + 1) * 100) / total_frames_system);
            }
        }
    }

    ESP_LOGI(TAG_AEC, "Recording completed: %zu samples at %lu Hz", samples_recorded_system, system_sample_rate);

    // ========== 阶段1.5: 重采样到16kHz（如果需要）==========
    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Phase 1.5: Resampling recorded audio from %lu Hz to %lu Hz...", 
                 system_sample_rate, aec_sample_rate);
        
        int64_t resample_start = esp_timer_get_time();
        size_t output_size_temp = 0;

        // 重采样麦克风数据（int16单声道）
        ret = remix_convert_pcm_to_format(
            (uint8_t*)mic_data_system, samples_recorded_system * sizeof(int16_t),
            system_sample_rate, 1, AUDIO_TYPE_INT16,
            aec_sample_rate, 1, AUDIO_TYPE_INT16,
            (uint8_t**)&resample_temp_mic, &output_size_temp);
        
        if (ret == ESP_OK) {
            size_t output_samples = output_size_temp / sizeof(int16_t);
            if (output_samples <= total_samples_aec) {
                memcpy(mic_data_aec, resample_temp_mic, output_size_temp);
            } else {
                ESP_LOGE(TAG_AEC, "Mic resample size mismatch: got %zu, expected <=%zu", output_samples, total_samples_aec);
                ret = ESP_FAIL;
                goto cleanup;
            }
        } else {
            ESP_LOGE(TAG_AEC, "Failed to resample mic data: %s", esp_err_to_name(ret));
            goto cleanup;
        }

        // 重采样参考数据（int16单声道）
        uint8_t *resample_temp_ref_u8 = nullptr;
        ret = remix_convert_pcm_to_format(
            (uint8_t*)ref_data_system, samples_recorded_system * sizeof(int16_t),
            system_sample_rate, 1, AUDIO_TYPE_INT16,
            aec_sample_rate, 1, AUDIO_TYPE_INT16,
            &resample_temp_ref_u8, &output_size_temp);
        
        if (ret == ESP_OK) {
            resample_temp_ref = (int16_t*)resample_temp_ref_u8;
            size_t output_samples = output_size_temp / sizeof(int16_t);
            if (output_samples <= total_samples_aec) {
                memcpy(ref_data_aec, resample_temp_ref, output_size_temp);
            } else {
                ESP_LOGE(TAG_AEC, "Ref resample size mismatch: got %zu, expected <=%zu", output_samples, total_samples_aec);
                ret = ESP_FAIL;
                goto cleanup;
            }
        } else {
            ESP_LOGE(TAG_AEC, "Failed to resample ref data: %s", esp_err_to_name(ret));
            goto cleanup;
        }

        int64_t resample_end = esp_timer_get_time();
        float resample_time_ms = (resample_end - resample_start) / 1000.0f;
        ESP_LOGI(TAG_AEC, "Resampling completed in %.2f ms", resample_time_ms);
    }

    // ========== 阶段2: AEC处理（16kHz）==========
    ESP_LOGI(TAG_AEC, "Phase 2: Processing AEC at %lu Hz...", aec_sample_rate);
    
    start_time = esp_timer_get_time();
    
    for (int frame = 0; frame < total_frames_aec; frame++) {
        // 准备交错格式的输入数据 [M, R, M, R, ...]
        for (int i = 0; i < frame_size; i++) {
            aec_input[i * nch + mic_idx] = mic_data_aec[frame * frame_size + i];
            aec_input[i * nch + ref_idx] = ref_data_aec[frame * frame_size + i];
        }

        // 执行AEC处理
        size_t processed = afe_aec_process(aec_handle, aec_input, output_data_aec + frame * frame_size);
        
        if (processed != (size_t)frame_bytes_aec) {
            ESP_LOGW(TAG_AEC, "AEC processing size mismatch at frame %d: expected=%d, got=%zu", 
                     frame, frame_bytes_aec, processed);
        }

        // 进度提示
        if (total_frames_aec > 10 && (frame + 1) % (total_frames_aec / 10) == 0) {
            ESP_LOGI(TAG_AEC, "AEC processing progress: %d%%", ((frame + 1) * 100) / total_frames_aec);
        }
    }

    end_time = esp_timer_get_time();
    processing_time_us = end_time - start_time;
    processing_time_ms = processing_time_us / 1000.0f;
    real_time_factor = (processing_time_ms / 1000.0f) / record_duration_seconds;

    ESP_LOGI(TAG_AEC, "AEC processing completed in %.2f ms (RTF=%.3fx)", 
             processing_time_ms, real_time_factor);

    // ========== 阶段2.5: 重采样回系统采样率（如果需要）==========
    if (need_resample) {
        ESP_LOGI(TAG_AEC, "Phase 2.5: Resampling processed audio from %lu Hz to %lu Hz...", 
                 aec_sample_rate, system_sample_rate);
        
        int64_t resample_start = esp_timer_get_time();
        size_t output_size_temp = 0;

        // 重采样AEC处理后的数据（int16单声道）
        ret = remix_convert_pcm_to_format(
            (uint8_t*)output_data_aec, total_samples_aec * sizeof(int16_t),
            aec_sample_rate, 1, AUDIO_TYPE_INT16,
            system_sample_rate, 1, AUDIO_TYPE_INT16,
            (uint8_t**)&resample_temp_output, &output_size_temp);
        
        if (ret == ESP_OK) {
            size_t output_samples = output_size_temp / sizeof(int16_t);
            if (output_samples <= total_samples_system) {
                memcpy(output_data_system, resample_temp_output, output_size_temp);
            } else {
                ESP_LOGE(TAG_AEC, "Output resample size mismatch: got %zu, expected <=%zu", output_samples, total_samples_system);
                ret = ESP_FAIL;
                goto cleanup;
            }
        } else {
            ESP_LOGE(TAG_AEC, "Failed to resample output data: %s", esp_err_to_name(ret));
            goto cleanup;
        }

        int64_t resample_end = esp_timer_get_time();
        float resample_time_ms = (resample_end - resample_start) / 1000.0f;
        ESP_LOGI(TAG_AEC, "Resampling completed in %.2f ms", resample_time_ms);
    }

    // ========== 阶段3: 播放 ==========
    ESP_LOGI(TAG_AEC, "Phase 3: Playback comparison...");
    
    // 播放原始录音(用于对比)
    {
        ESP_LOGI(TAG_AEC, "Phase 3.1: Playing ORIGINAL recorded audio at %lu Hz (before AEC)...", 
                 system_sample_rate);
        
        int16_t *playback_data = mic_data_system;

        // 根据当前TX声道配置,决定是否需要把单声道扩展为双声道
        if (tx_channels == AUDIO_CHANNELS_STEREO) {
            // 扩展为双声道交织 [L=mono, R=mono]
            size_t stereo_samples = samples_recorded_system * 2; // L+R
            size_t stereo_bytes = stereo_samples * sizeof(int16_t);
            int16_t *stereo_buf = (int16_t*)heap_caps_aligned_calloc(16, stereo_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (!stereo_buf) {
                ESP_LOGE(TAG_AEC, "Failed to allocate stereo buffer (%zu bytes)", stereo_bytes);
                ret = ESP_ERR_NO_MEM;
                goto cleanup;
            }
            for (size_t i = 0; i < samples_recorded_system; ++i) {
                int16_t s = playback_data[i];
                stereo_buf[2*i] = s;     // L
                stereo_buf[2*i + 1] = s; // R
            }
            ESP_LOGI(TAG_AEC, "Playback ORIGINAL (stereo): total_samples(LR)=%zu, total_bytes=%zu", stereo_samples, stereo_bytes);
            ret = esp_codec_dev_write(play_dev, (uint8_t*)stereo_buf, (int)stereo_bytes);
            free(stereo_buf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Playback ORIGINAL failed (stereo): %s", esp_err_to_name(ret));
                goto cleanup;
            }
            ESP_LOGI(TAG_AEC, "Playback ORIGINAL completed successfully (stereo)");
        } else {
            // 单声道直接播放
            size_t total_playback_bytes = samples_recorded_system * sizeof(int16_t);
            ESP_LOGI(TAG_AEC, "Playback ORIGINAL (mono): total_samples=%zu, total_bytes=%zu", 
                     samples_recorded_system, total_playback_bytes);
            ret = esp_codec_dev_write(play_dev, (uint8_t*)playback_data, (int)total_playback_bytes);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Playback ORIGINAL failed (mono): %s", esp_err_to_name(ret));
                goto cleanup;
            }
            ESP_LOGI(TAG_AEC, "Playback ORIGINAL completed successfully (mono)");
        }
    }
    
    // 等待1秒,让用户区分两次播放
    ESP_LOGI(TAG_AEC, "Waiting 1 second before playing AEC-processed audio...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 播放AEC处理后的音频(对比效果)
    if (!play_original_audio) {
        ESP_LOGI(TAG_AEC, "Phase 3.2: Playing AEC-PROCESSED audio at %lu Hz (echo removed)...", 
                 system_sample_rate);
        
        int16_t *playback_data = output_data_system;

        // 根据当前TX声道配置,决定是否需要把单声道扩展为双声道
        if (tx_channels == AUDIO_CHANNELS_STEREO) {
            // 扩展为双声道交织 [L=mono, R=mono]
            size_t stereo_samples = samples_recorded_system * 2; // L+R
            size_t stereo_bytes = stereo_samples * sizeof(int16_t);
            int16_t *stereo_buf = (int16_t*)heap_caps_aligned_calloc(16, stereo_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (!stereo_buf) {
                ESP_LOGE(TAG_AEC, "Failed to allocate stereo buffer (%zu bytes)", stereo_bytes);
                ret = ESP_ERR_NO_MEM;
                goto cleanup;
            }
            for (size_t i = 0; i < samples_recorded_system; ++i) {
                int16_t s = playback_data[i];
                stereo_buf[2*i] = s;     // L
                stereo_buf[2*i + 1] = s; // R
            }
            ESP_LOGI(TAG_AEC, "Playback AEC-PROCESSED (stereo): total_samples(LR)=%zu, total_bytes=%zu", stereo_samples, stereo_bytes);
            ret = esp_codec_dev_write(play_dev, (uint8_t*)stereo_buf, (int)stereo_bytes);
            free(stereo_buf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Playback AEC-PROCESSED failed (stereo): %s", esp_err_to_name(ret));
                goto cleanup;
            }
            ESP_LOGI(TAG_AEC, "Playback AEC-PROCESSED completed successfully (stereo)");
        } else {
            // 单声道直接播放
            size_t total_playback_bytes = samples_recorded_system * sizeof(int16_t);
            ESP_LOGI(TAG_AEC, "Playback AEC-PROCESSED (mono): total_samples=%zu, total_bytes=%zu", 
                     samples_recorded_system, total_playback_bytes);
            ret = esp_codec_dev_write(play_dev, (uint8_t*)playback_data, (int)total_playback_bytes);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_AEC, "Playback AEC-PROCESSED failed (mono): %s", esp_err_to_name(ret));
                goto cleanup;
            }
            ESP_LOGI(TAG_AEC, "Playback AEC-PROCESSED completed successfully (mono)");
        }
    }

cleanup:
    // 清理资源
    ESP_LOGI(TAG_AEC, "Cleaning up resources...");
    if (aec_handle) {
        afe_aec_destroy(aec_handle);
    }
    free(aec_input);
    free(mic_data_system);
    free(ref_data_system);
    free(output_data_system);
    free(dual_channel_buffer);  // 释放双通道缓冲区
    if (need_resample) {
        // These pointers were allocated separately only when resampling
        free(mic_data_aec);
        free(ref_data_aec);
        free(output_data_aec);
    }
    // Note: These buffers are allocated by remix_convert_pcm_to_format()
    heap_caps_free(resample_temp_mic);
    heap_caps_free(resample_temp_ref);
    heap_caps_free(resample_temp_output);

    // 测试结束后清空播放管线，避免残留音频
    {
        esp_err_t clr_ret = clear_audio_pipeline(120);
        if (clr_ret != ESP_OK) {
            ESP_LOGW(TAG_AEC, "Failed to clear playback pipeline after test: %s", esp_err_to_name(clr_ret));
        }
    }

    ESP_LOGI(TAG_AEC, "=== AEC Test Completed ===");
    return ret;
}

/**
 * @brief 测试AEC硬件回采功能
 * 
 * 此函数用于验证ES7210的硬件AEC回采是否正常工作。
 * 功能：
 * 1. 从ES7210同时读取CH1(麦克风)和CH3(回采)
 * 2. 分析两个通道的音频数据特征
 * 3. 分别播放麦克风通道和回采通道,让用户听到实际内容
 * 4. 提供详细的统计信息(RMS, 峰值, 零值率等)
 * 
 * @param record_duration_seconds 录音时长（秒），默认3秒
 * @param play_channels 播放选项：0=不播放, 1=仅麦克风, 2=仅回采, 3=两者都播放
 * 
 * @return esp_err_t 返回操作结果
 *         - ESP_OK: 测试成功
 *         - ESP_ERR_INVALID_STATE: 音频系统未初始化或未启用硬件AEC
 *         - ESP_ERR_NO_MEM: 内存分配失败
 *         - 其他: 底层驱动错误
 */
esp_err_t audio_tools::test_aec_loopback(uint32_t record_duration_seconds, uint8_t play_channels)
{
    esp_err_t ret = ESP_OK;
    int16_t *mic_data = nullptr;
    int16_t *ref_data = nullptr;
    int16_t *dual_channel_buffer = nullptr;
    
    // 提前声明所有变量(避免goto跳过初始化警告)
    uint32_t system_sample_rate = 0;
    size_t frame_size = 512;
    size_t frame_bytes = 0;
    size_t total_frames = 0;
    size_t total_samples = 0;
    size_t samples_recorded = 0;
    
    // 分析变量
    int64_t mic_sum_squares = 0;
    int32_t mic_max = INT16_MIN;
    int32_t mic_min = INT16_MAX;
    size_t mic_zero_count = 0;
    size_t mic_clipped_count = 0;
    double mic_rms = 0.0;
    double mic_zero_rate = 0.0;
    double mic_clip_rate = 0.0;
    
    int64_t ref_sum_squares = 0;
    int32_t ref_max = INT16_MIN;
    int32_t ref_min = INT16_MAX;
    size_t ref_zero_count = 0;
    size_t ref_clipped_count = 0;
    double ref_rms = 0.0;
    double ref_zero_rate = 0.0;
    double ref_clip_rate = 0.0;
    
    // 检查基本初始化状态
    if (!es7210_initialized || !es8311_initialized) {
        ESP_LOGE(TAG_AEC, "Audio devices not initialized (ES7210 or ES8311)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!record_dev || !play_dev) {
        ESP_LOGE(TAG_AEC, "Record or playback device not available");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查是否启用了硬件AEC回采
    if (mic_channels != AUDIO_MIC_CHANNEL_13) {
        ESP_LOGE(TAG_AEC, "Hardware AEC loopback NOT enabled. Current mic_channels: 0x%02X", mic_channels);
        ESP_LOGE(TAG_AEC, "To enable AEC loopback, initialize ES7210 with AUDIO_MIC_CHANNEL_13");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查I2S RX是否配置为立体声
    if (rx_channels != AUDIO_CHANNELS_STEREO) {
        ESP_LOGE(TAG_AEC, "I2S RX is not in STEREO mode. Current: %s", 
                 rx_channels == AUDIO_CHANNELS_MONO ? "MONO" : "UNKNOWN");
    ESP_LOGE(TAG_AEC, "To enable AEC loopback, call es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_13, ES7210_TDM_DISABLED)");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG_AEC, "=== Starting AEC Loopback Test ===");
    ESP_LOGI(TAG_AEC, "Configuration: mic_channels=0x%02X (CH1+CH3), I2S_RX=STEREO", mic_channels);
    ESP_LOGI(TAG_AEC, "Sample Rate: %d Hz, Duration: %lu seconds", sample_rate, record_duration_seconds);
    
    // 计算缓冲区大小
    system_sample_rate = static_cast<uint32_t>(sample_rate);
    frame_bytes = frame_size * sizeof(int16_t);
    total_frames = (record_duration_seconds * system_sample_rate) / frame_size;
    total_samples = total_frames * frame_size;
    
    ESP_LOGI(TAG_AEC, "Buffer: frame_size=%zu, total_frames=%zu, total_samples=%zu", 
             frame_size, total_frames, total_samples);
    
    // 分配音频缓冲区
    mic_data = (int16_t *)heap_caps_aligned_calloc(16, total_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    ref_data = (int16_t *)heap_caps_aligned_calloc(16, total_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    dual_channel_buffer = (int16_t *)malloc(frame_bytes * 2); // 双通道
    
    if (!mic_data || !ref_data || !dual_channel_buffer) {
        ESP_LOGE(TAG_AEC, "Failed to allocate buffers");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    
    ESP_LOGI(TAG_AEC, "Buffers allocated successfully");
    
    // ========== 阶段1: 录音 ==========
    ESP_LOGI(TAG_AEC, "Phase 1: Recording %lu seconds (CH1=Mic, CH3=Loopback)...", record_duration_seconds);
    
    for (size_t frame = 0; frame < total_frames; frame++) {
        // 从ES7210读取双通道交错数据 [CH1, CH3, CH1, CH3, ...]
        ret = esp_codec_dev_read(record_dev, (uint8_t*)dual_channel_buffer, frame_bytes * 2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_AEC, "Failed to read dual channel data at frame %zu: %s", frame, esp_err_to_name(ret));
            goto cleanup;
        }

        // 分离通道
        for (size_t i = 0; i < frame_size; i++) {
            mic_data[samples_recorded + i] = dual_channel_buffer[i * 2];     // CH1(Mic)
            ref_data[samples_recorded + i] = dual_channel_buffer[i * 2 + 1]; // CH3(Loopback)
        }

        samples_recorded += frame_size;

        // 进度提示
        if (total_frames > 10 && (frame + 1) % (total_frames / 10) == 0) {
            ESP_LOGI(TAG_AEC, "Recording progress: %d%%", ((frame + 1) * 100) / total_frames);
        }
    }
    
    ESP_LOGI(TAG_AEC, "Recording completed: %zu samples captured", samples_recorded);
    
    // ========== 阶段2: 数据分析 ==========
    ESP_LOGI(TAG_AEC, "Phase 2: Analyzing recorded data...");
    
    // 分析CH1(麦克风)
    mic_sum_squares = 0;
    mic_max = INT16_MIN;
    mic_min = INT16_MAX;
    mic_zero_count = 0;
    mic_clipped_count = 0;
    
    for (size_t i = 0; i < samples_recorded; i++) {
        int16_t sample = mic_data[i];
        mic_sum_squares += (int64_t)sample * sample;
        if (sample > mic_max) mic_max = sample;
        if (sample < mic_min) mic_min = sample;
        if (sample == 0) mic_zero_count++;
        if (sample >= 32767 || sample <= -32768) mic_clipped_count++;
    }
    
    mic_rms = sqrt((double)mic_sum_squares / samples_recorded);
    mic_zero_rate = (double)mic_zero_count / samples_recorded * 100.0;
    mic_clip_rate = (double)mic_clipped_count / samples_recorded * 100.0;
    
    // 分析CH3(回采)
    ref_sum_squares = 0;
    ref_max = INT16_MIN;
    ref_min = INT16_MAX;
    ref_zero_count = 0;
    ref_clipped_count = 0;
    
    for (size_t i = 0; i < samples_recorded; i++) {
        int16_t sample = ref_data[i];
        ref_sum_squares += (int64_t)sample * sample;
        if (sample > ref_max) ref_max = sample;
        if (sample < ref_min) ref_min = sample;
        if (sample == 0) ref_zero_count++;
        if (sample >= 32767 || sample <= -32768) ref_clipped_count++;
    }
    
    ref_rms = sqrt((double)ref_sum_squares / samples_recorded);
    ref_zero_rate = (double)ref_zero_count / samples_recorded * 100.0;
    ref_clip_rate = (double)ref_clipped_count / samples_recorded * 100.0;
    
    // 打印统计信息
    ESP_LOGI(TAG_AEC, "=== CH1 (Microphone) Statistics ===");
    ESP_LOGI(TAG_AEC, "  RMS Level: %.2f", mic_rms);
    ESP_LOGI(TAG_AEC, "  Peak: %ld to %ld", mic_min, mic_max);
    ESP_LOGI(TAG_AEC, "  Zero samples: %zu (%.2f%%)", mic_zero_count, mic_zero_rate);
    ESP_LOGI(TAG_AEC, "  Clipped samples: %zu (%.2f%%)", mic_clipped_count, mic_clip_rate);
    
    ESP_LOGI(TAG_AEC, "=== CH3 (Loopback) Statistics ===");
    ESP_LOGI(TAG_AEC, "  RMS Level: %.2f", ref_rms);
    ESP_LOGI(TAG_AEC, "  Peak: %ld to %ld", ref_min, ref_max);
    ESP_LOGI(TAG_AEC, "  Zero samples: %zu (%.2f%%)", ref_zero_count, ref_zero_rate);
    ESP_LOGI(TAG_AEC, "  Clipped samples: %zu (%.2f%%)", ref_clipped_count, ref_clip_rate);
    
    // 判断结果
    ESP_LOGI(TAG_AEC, "=== Analysis Results ===");
    if (mic_rms < 10.0) {
        ESP_LOGW(TAG_AEC, "CH1 (Mic): Signal too weak (RMS=%.2f). Check microphone connection.", mic_rms);
    } else {
        ESP_LOGI(TAG_AEC, "CH1 (Mic): Signal detected (RMS=%.2f) [OK]", mic_rms);
    }
    
    if (ref_zero_rate > 99.0) {
        ESP_LOGE(TAG_AEC, "CH3 (Loopback): ALL SILENT! Loopback NOT working.");
        ESP_LOGE(TAG_AEC, "Possible causes:");
        ESP_LOGE(TAG_AEC, "  1. ES7210 CH3 not configured for loopback");
        ESP_LOGE(TAG_AEC, "  2. No audio playing during recording");
        ESP_LOGE(TAG_AEC, "  3. Loopback routing incorrect in hardware");
    } else if (ref_rms < 10.0) {
        ESP_LOGW(TAG_AEC, "CH3 (Loopback): Signal too weak (RMS=%.2f). Check loopback routing.", ref_rms);
    } else {
        ESP_LOGI(TAG_AEC, "CH3 (Loopback): Signal detected (RMS=%.2f) [OK]", ref_rms);
        ESP_LOGI(TAG_AEC, "Hardware AEC loopback is WORKING!");
    }
    
    // ========== 阶段3: 播放 ==========
    if (play_channels > 0) {
        ESP_LOGI(TAG_AEC, "Phase 3: Playback test...");
        
        // 播放麦克风通道
        if (play_channels & 0x01) {
            ESP_LOGI(TAG_AEC, "Playing CH1 (Microphone) audio...");
            vTaskDelay(pdMS_TO_TICKS(500));
            
            if (tx_channels == AUDIO_CHANNELS_STEREO) {
                // 扩展为双声道
                size_t stereo_samples = samples_recorded * 2;
                int16_t *stereo_buf = (int16_t*)heap_caps_aligned_calloc(16, stereo_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
                if (stereo_buf) {
                    for (size_t i = 0; i < samples_recorded; ++i) {
                        stereo_buf[2*i] = mic_data[i];
                        stereo_buf[2*i + 1] = mic_data[i];
                    }
                    ret = esp_codec_dev_write(play_dev, (uint8_t*)stereo_buf, (int)(stereo_samples * sizeof(int16_t)));
                    free(stereo_buf);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG_AEC, "CH1 playback completed");
                    } else {
                        ESP_LOGE(TAG_AEC, "CH1 playback failed: %s", esp_err_to_name(ret));
                    }
                }
            } else {
                ret = esp_codec_dev_write(play_dev, (uint8_t*)mic_data, (int)(samples_recorded * sizeof(int16_t)));
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG_AEC, "CH1 playback completed");
                } else {
                    ESP_LOGE(TAG_AEC, "CH1 playback failed: %s", esp_err_to_name(ret));
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // 播放回采通道
        if (play_channels & 0x02) {
            ESP_LOGI(TAG_AEC, "Playing CH3 (Loopback) audio...");
            vTaskDelay(pdMS_TO_TICKS(500));
            
            if (tx_channels == AUDIO_CHANNELS_STEREO) {
                // 扩展为双声道
                size_t stereo_samples = samples_recorded * 2;
                int16_t *stereo_buf = (int16_t*)heap_caps_aligned_calloc(16, stereo_samples, sizeof(int16_t), MALLOC_CAP_SPIRAM);
                if (stereo_buf) {
                    for (size_t i = 0; i < samples_recorded; ++i) {
                        stereo_buf[2*i] = ref_data[i];
                        stereo_buf[2*i + 1] = ref_data[i];
                    }
                    ret = esp_codec_dev_write(play_dev, (uint8_t*)stereo_buf, (int)(stereo_samples * sizeof(int16_t)));
                    free(stereo_buf);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG_AEC, "CH3 playback completed");
                    } else {
                        ESP_LOGE(TAG_AEC, "CH3 playback failed: %s", esp_err_to_name(ret));
                    }
                }
            } else {
                ret = esp_codec_dev_write(play_dev, (uint8_t*)ref_data, (int)(samples_recorded * sizeof(int16_t)));
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG_AEC, "CH3 playback completed");
                } else {
                    ESP_LOGE(TAG_AEC, "CH3 playback failed: %s", esp_err_to_name(ret));
                }
            }
        }
    }
    
    ret = ESP_OK;

cleanup:
    // 清理资源
    ESP_LOGI(TAG_AEC, "Cleaning up resources...");
    free(mic_data);
    free(ref_data);
    free(dual_channel_buffer);
    
    // 清空播放管线
    if (play_channels > 0) {
        esp_err_t clr_ret = clear_audio_pipeline(120);
        if (clr_ret != ESP_OK) {
            ESP_LOGW(TAG_AEC, "Failed to clear playback pipeline: %s", esp_err_to_name(clr_ret));
        }
    }
    
    ESP_LOGI(TAG_AEC, "=== AEC Loopback Test Completed ===");
    return ret;
}
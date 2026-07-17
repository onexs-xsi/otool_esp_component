/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_TOOLS_H__
#define __AUDIO_TOOLS_H__

//es8311 and es7210 include
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_idf_version.h"
#include "soc/soc_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
typedef int i2s_port_t;
#endif

// ESP-SR AEC support
#include "esp_afe_aec.h"

// 音频通用类型定义
#include "audio_types.h"

// 子对象类头文件
#include "audio_playback.h"
#include "audio_recorder.h"

// 前置声明,避免循环依赖
class audio_sr_afe;

// 芯片设备专用头文件
#include "../audio_chip_device/audio_es_es8311/audio_es_es8311.h"
#include "../audio_chip_device/audio_es_es7210/audio_es_es7210.h"

// I2S PIN MAP
#define I2S_BCLK_PIN       GPIO_NUM_40
#define I2S_MCLK_PIN       GPIO_NUM_42
#define I2S_DATA_IN_PIN    GPIO_NUM_38
#define I2S_DATA_OUT_PIN   GPIO_NUM_39
#define I2S_DATA_WS_PIN    GPIO_NUM_41
#define PA_PIN             GPIO_NUM_46

#ifndef CHECK
#define CHECK(r) if (!(r)) return -1;
#endif
#ifndef TEST_ESP_ERR
#define TEST_ESP_ERR(rc, res) CHECK((rc) == (res))
#endif
#ifndef TEST_ESP_OK
#define TEST_ESP_OK(rc) CHECK((rc) == ESP_OK)
#endif

// audio_file_type_t, audio_playback_mode_t, audio_sample_rate_t → 已迁移到 audio_types.h
// channel_split_result_t, mic_channel_quality_t → 已迁移到 audio_recorder.h

/**
 * @brief audio_tools 类
 * 
 * 提供ES8311和ES7210音频芯片的操作功能，包括初始化、播放、录音和睡眠管理
 * 
 * 架构设计：
 * - 主类负责系统级管理和I2S通道控制
 * - ES8311功能实现在 audio_es_es8311.cpp 中
 * - ES7210功能实现在 audio_es_es7210.cpp 中 
 * - 为未来扩展新音频设备提供统一框架
 * 
 * 扩展新设备方法：
 * 1. 创建对应的 audio_xxx.cpp 文件
 * 2. 在此类中添加相应的成员变量和初始化函数
 * 3. 确保遵循现有的I2S用户计数管理机制
 * 4. 在audio_system_sleep()中添加对应的睡眠调用
 */
class audio_tools {
private:
    using pa_power_callback_t = esp_err_t (*)(void *arg, bool on);

    esp_codec_dev_handle_t play_dev;        ///< 播放设备句柄（ES8311）
    esp_codec_dev_handle_t record_dev;      ///< 录音设备句柄（ES7210或ES8311 ADC）
    i2s_chan_handle_t tx_handle;            ///< I2S发送通道句柄
    i2s_chan_handle_t rx_handle;            ///< I2S接收通道句柄
    i2c_master_bus_handle_t i2c_bus_handle;        ///< I2C总线句柄
    
    // 独立状态管理
    bool es8311_initialized;                ///< ES8311初始化状态
    bool es7210_initialized;                ///< ES7210初始化状态
    bool system_initialized;                ///< 音频系统初始化状态
    bool es8311_sleeping;                   ///< ES8311睡眠状态
    bool es7210_sleeping;                   ///< ES7210睡眠状态
    esp_codec_dev_handle_t es8311_dev_handle = nullptr; ///< ES8311 设备句柄（可能同时用于播放与录音）
    esp_codec_dec_work_mode_t es8311_work_mode = ESP_CODEC_DEV_WORK_MODE_NONE; ///< ES8311 当前工作模式
    
    // I2S引脚配置
    gpio_num_t i2s_bck_pin;                 ///< I2S BCK引脚
    gpio_num_t i2s_mck_pin;                 ///< I2S MCK引脚
    gpio_num_t i2s_data_in_pin;             ///< I2S数据输入引脚
    gpio_num_t i2s_data_out_pin;            ///< I2S数据输出引脚
    gpio_num_t i2s_ws_pin;                  ///< I2S WS引脚
    gpio_num_t pa_pin;                      ///< 功放使能引脚

    // I2S 使用引用计数：任一编解码器使用即 +1，用于决定是否需要真正释放通道
    int i2s_user_count = 0;                 ///< 使用 I2S 的编解码器数量
    bool tx_configured = false;             ///< TX 通道已完成模式配置并 enable
    bool rx_configured = false;             ///< RX 通道已完成模式配置并 enable
    uint8_t rx_tdm_slot_count = 0;          ///< RX TDM 启用的槽位数量
    bool i2s_cross_data_pins = true;        ///< 是否使用交叉数据引脚映射（硬件走线导致）
    bool pins_high_z_on_sleep = false;      ///< 进入睡眠时是否将 I2S 与 PA 引脚置为高阻
    bool suppress_release = false;          ///< 在系统整体去初始化期间暂缓 I2S 释放
    
    // I2S 通道配置参数
    i2s_port_t i2s_port_num = I2S_NUM_0;                        ///< I2S通道编号
    audio_channels_t tx_channels = AUDIO_CHANNELS_MONO;         ///< TX（播放）声道数量
    audio_channels_t rx_channels = AUDIO_CHANNELS_MONO;         ///< RX（录音）声道数量
    audio_sample_rate_t sample_rate = AUDIO_SAMPLE_RATE_16K;   ///< 采样率（TX和RX共享）
    i2s_data_bit_width_t bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT;    ///< 位深度（TX和RX共享）
    
    // 音频响度配置
    float volume = 80.0;                    ///< 播放音量 (0.0 - 100.0)
    float record_gain = 30.0;               ///< 录音增益 (0.0 - 66.0 dB)
    
    // ES7210 麦克风通道配置
    audio_mic_channel_t mic_channels = AUDIO_MIC_CHANNEL_1;   ///< 默认使用麦克风通道1
    bool es7210_use_tdm = false;                               ///< ES7210 是否使用 TDM 模式

    // 共享 I2S 数据接口
    const audio_codec_data_if_t *shared_i2s_data_if = nullptr; ///< 共享的 I2S 数据接口（避免重复创建）

    // ES8311 codec 接口对象（init 时创建，deinit 时释放，避免泄漏）
    const audio_codec_ctrl_if_t *es8311_ctrl_if = nullptr;
    const audio_codec_gpio_if_t *es8311_gpio_if = nullptr;
    const audio_codec_if_t      *es8311_codec_if = nullptr;

    // ES7210 codec 接口对象（init 时创建，deinit 时释放，避免泄漏）
    const audio_codec_ctrl_if_t *es7210_ctrl_if = nullptr;
    const audio_codec_if_t      *es7210_codec_if = nullptr;

    // ESP-SR AFE 子对象
    audio_sr_afe* sr_afe_ = nullptr;        ///< ESP-SR AFE 对象指针(用于 AEC 等功能)

    // 子对象指针（惰性创建）
    audio_playback* playback_ = nullptr;    ///< 播放子对象
    audio_recorder* recorder_ = nullptr;    ///< 录音子对象

    // 系统级互斥锁（用于保护 deinit 等操作，also shared by audio_playback）
    SemaphoreHandle_t audio_mutex = nullptr;

    // 外部板级 PA 控制回调。audio_tools 保持通用，不直接依赖具体板卡类型。
    pa_power_callback_t pa_power_callback = nullptr;
    void *pa_power_callback_arg = nullptr;
    TaskHandle_t pa_enable_task_handle = nullptr;
    uint32_t pa_enable_delay_ms = 0;

    // 内部辅助函数
    esp_err_t ensure_i2s_channel();         ///< 确保已创建 I2S 通道
    esp_err_t try_release_i2s();            ///< 在引用计数为 0 时释放 I2S 通道
    void incr_i2s_user();                   ///< 增加 I2S 使用者计数
    void decr_i2s_user();                   ///< 减少 I2S 使用者计数
    static void delayed_pa_enable_task_entry(void *arg);

public:
    using PaPowerCallback = pa_power_callback_t;

    // 声明友元类,允许其访问私有成员
    friend class audio_sr_afe;
    friend class audio_playback;
    friend class audio_recorder;

    /**
     * @brief 构造函数
     * 
     * 使用默认引脚配置创建 audio_tools 对象
     */
    audio_tools();

    /**
     * @brief 构造函数（带参数）
     * 
     * @param bck_pin I2S BCK引脚
     * @param mck_pin I2S MCK引脚
     * @param data_in_pin I2S数据输入引脚
     * @param data_out_pin I2S数据输出引脚
     * @param ws_pin I2S WS引脚
     * @param pa_pin 功放使能引脚
     */
    audio_tools(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                   gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin);

    /**
     * @brief 析构函数
     * 
     * 销毁 audio_tools 对象，清理音频资源
     */
    ~audio_tools();

    /**
     * @brief 初始化 ES8311 音频芯片
     *
     * @param channels 音频声道配置（仅支持单声道或立体声）
     * @param mode     指定启用 ADC、DAC 或全双工路径
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es8311_init(audio_channels_t channels,
                          es8311_path_mode_t mode = ES8311_MODE_DAC);

    /**
     * @brief 去初始化ES8311音频芯片
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es8311_deinit();

    /**
     * @brief 初始化I2S通道（同时创建TX和RX通道）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_channel_init();

    /**
     * @brief 去初始化I2S通道（同时删除TX和RX通道）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_channel_deinit();

    /**
     * @brief 初始化I2S驱动（TX通道用于播放）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_tx_init();

    /**
     * @brief 去初始化I2S驱动（TX通道）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_tx_deinit();

    /**
     * @brief 初始化I2S驱动（RX通道用于录音）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_rx_init();

    /**
     * @brief 去初始化I2S驱动（RX通道）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t i2s_rx_deinit();

    /**
     * @brief 初始化ES7210音频芯片（ADC录音）
     * 
    * @param channels 音频声道配置（单声道/立体声/TDM）
    * @param mic_channels 麦克风通道选择
    * @param use_tdm 是否启用TDM模式（显式指定，启用后强制使用4个slot）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es7210_init(audio_channels_t channels,
                          audio_mic_channel_t mic_channels,
                     es7210_tdm_mode_t use_tdm = ES7210_TDM_DISABLED);

    /**
     * @brief 去初始化ES7210音频芯片
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es7210_deinit();

    /**
     * @brief 设置ES7210指定通道的静音状态
     * 
     * 通过寄存器 0x14(ADC3/ADC4) 和 0x15(ADC1/ADC2) 精确控制每个麦克风通道的静音状态。
     * 与增益控制(0x43-0x46)和通道启用(bit[4])独立工作，允许在不断电的情况下静音通道。
     * 
     * 寄存器映射：
     * - 0x15[0] = ADC1 (MIC1) 静音控制
     * - 0x15[1] = ADC2 (MIC2) 静音控制  
     * - 0x14[0] = ADC3 (MIC3) 静音控制
     * - 0x14[1] = ADC4 (MIC4) 静音控制
     * 
     * 应用场景：
     * - 标准I²S模式下MIC2独占使用（需要启用MIC1但静音）
     * - TDM模式下动态启用/禁用特定麦克风
     * - 多麦克风阵列中的选择性静音
     * - 快速响应的静音控制（无需断电重配置）
     * 
     * @param mic_channel 麦克风通道（仅支持单一通道）
     *                    - AUDIO_MIC_CHANNEL_1: MIC1
     *                    - AUDIO_MIC_CHANNEL_2: MIC2
     *                    - AUDIO_MIC_CHANNEL_3: MIC3
     *                    - AUDIO_MIC_CHANNEL_4: MIC4
     * @param mute 静音状态
     *             - true: 静音该通道（寄存器相应位置1）
     *             - false: 取消静音（寄存器相应位清零）
     * 
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 操作成功
     *         - ESP_ERR_INVALID_STATE: ES7210未初始化
     *         - ESP_ERR_INVALID_ARG: mic_channel不是单一通道或无效
     *         - ESP_FAIL: 寄存器读写失败
     * 
     * @note 此函数仅控制静音，不影响通道的电源状态和增益设置
     * @note 如果寄存器值未改变，函数会提前返回以优化性能
     * @note 在TDM模式下，建议所有通道都保持取消静音状态（硬件自动管理slot）
     * 
     * @example
     * ```cpp
     * // 场景1：MIC2独占模式（标准I²S）
     * es7210_init(AUDIO_CHANNELS_STEREO, AUDIO_MIC_CHANNEL_2, ES7210_TDM_DISABLED);
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_1, true);   // 静音MIC1（伴侣通道）
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_2, false);  // 确保MIC2正常
     * 
     * // 场景2：TDM模式下动态选择麦克风
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_3, true);   // 临时禁用MIC3
     * // ... 录音 ...
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_3, false);  // 重新启用MIC3
     * 
     * // 场景3：批量静音所有通道
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_1, true);
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_2, true);
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_3, true);
     * es7210_set_mic_channel_mute(AUDIO_MIC_CHANNEL_4, true);
     * ```
     */
    esp_err_t es7210_set_mic_channel_mute(audio_mic_channel_t mic_channel, bool mute);

    /**
     * @brief 初始化音频系统（包含所有已配置的音频模块）
     * 
     * @param i2c_bus_handle I2C总线句柄
     * @param i2s_port_num I2S通道编号
     * @param sample_rate 采样率（播放和录音共享，因为共享I2S时钟）
     * @param bits_per_sample 位深度（播放和录音共享，因为共享I2S时钟）
     * @return esp_err_t 返回操作结果
     * 
     * @note 采样率和位深度必须在播放和录音之间保持一致，因为它们共享同一个I2S总线
     * @note 声道配置在各自的 es8311_init() 和 es7210_init() 中独立设置
     */
    esp_err_t audio_system_init(i2c_master_bus_handle_t i2c_bus_handle, i2s_port_t i2s_port_num, audio_sample_rate_t sample_rate, i2s_data_bit_width_t bits_per_sample);

    /**
     * @brief 去初始化音频系统
     *
     * @param for_deep_sleep true=深度睡眠路径，跳过 i2s_del_channel 与 DMA
     *        描述符释放（HP 域断电后会整体复位）。默认 false，常规释放路径。
     * @return esp_err_t 返回操作结果
     *
     * @note 在 ESP32-P4 深度睡眠流程上调用 i2s_del_channel 会触发
     *       i2s_free_dma_desc 中的 free()，已观察到 TLSF 堆元数据断言崩溃；
     *       且这一释放对深度睡眠毫无价值（HP 域整体断电后内存复位）。
     */
    esp_err_t audio_system_deinit(bool for_deep_sleep = false);

    /**
     * @brief 使ES8311进入睡眠模式
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es8311_sleep();

    /**
     * @brief 使ES7210进入睡眠模式
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es7210_sleep();

    /**
     * @brief 使音频系统进入睡眠模式
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t audio_system_sleep();

    // ===== 录音方法（已迁移到 audio_recorder）=====
    // 请使用 get_recorder()->xxx() 访问新接口

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

    [[deprecated("Use get_recorder()->record_test()")]]
    esp_err_t record_test(uint32_t record_duration_ms = 3000) {
        return get_recorder()->record_test(record_duration_ms);
    }

    [[deprecated("Use get_recorder()->record_and_play_test()")]]
    esp_err_t record_and_play_test(uint32_t record_duration_seconds = 3) {
        return get_recorder()->record_and_play_test(record_duration_seconds);
    }

    [[deprecated("Use get_recorder()->record_and_play_test_with_channel_select()")]]
    esp_err_t record_and_play_test_with_channel_select(uint32_t record_duration_seconds = 3, 
                                                        audio_mic_channel_t target_mic_channel = AUDIO_MIC_CHANNEL_1,
                                                        bool analysis_only = false) {
        return get_recorder()->record_and_play_test_with_channel_select(record_duration_seconds, target_mic_channel, analysis_only);
    }

    [[deprecated("Use get_recorder()->record_all_channel_to_files()")]]
    esp_err_t record_all_channel_to_files(uint32_t record_duration_seconds,
                                          const char* output_directory,
                                          const char* file_prefix = nullptr) {
        return get_recorder()->record_all_channel_to_files(record_duration_seconds, output_directory, file_prefix);
    }

    [[deprecated("Use get_recorder()->record_and_playback_test()")]]
    esp_err_t record_and_playback_test(uint32_t record_duration_seconds = 5,
                                       bool loop_playback = false,
                                       audio_mic_channel_t target_mic_channel = AUDIO_MIC_NONE) {
        return get_recorder()->record_and_playback_test(record_duration_seconds, loop_playback, target_mic_channel);
    }

    [[deprecated("Use audio_recorder::compute_split_channel_quality()")]]
    static void compute_split_channel_quality(const channel_split_result_t& split_result,
                                              mic_channel_quality_t quality[4]) {
        audio_recorder::compute_split_channel_quality(split_result, quality);
    }

    [[deprecated("Use audio_recorder::split_recorded_channels()")]]
    static channel_split_result_t split_recorded_channels(const uint8_t* record_buffer,
                                                          size_t bytes_read,
                                                          const esp_codec_dev_sample_info_t& fs,
                                                          bool is_tdm_mode,
                                                          audio_mic_channel_t mic_channels) {
        return audio_recorder::split_recorded_channels(record_buffer, bytes_read, fs, is_tdm_mode, mic_channels);
    }

    [[deprecated("Use audio_recorder::free_channel_split_result()")]]
    static void free_channel_split_result(channel_split_result_t& result) {
        audio_recorder::free_channel_split_result(result);
    }

    // [已移除] 旧版 AEC 测试接口 aec_test()/test_aec_loopback() 已迁移到 audio_sr_afe 类。
    // 请使用 get_sr_afe()->aec_init()/aec_test_loopback()/aec_test() 进行 AEC 相关测试。

    [[deprecated("Use get_recorder()->record_to_file()")]]
    esp_err_t record_to_file(const char* filepath, uint32_t record_duration_seconds, size_t chunk_size = 4096) {
        return get_recorder()->record_to_file(filepath, record_duration_seconds, chunk_size);
    }

#pragma GCC diagnostic pop

    // ===== 播放方法（已迁移到 audio_playback）=====
    // 请使用 get_playback()->xxx() 访问新接口

    [[deprecated("Use get_playback()->play_audio_file()")]]
    esp_err_t play_audio_file(audio_file_type_t audio_type, audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING, float duration_limit_seconds = 0.0f) {
        return get_playback()->play_audio_file(audio_type, mode, duration_limit_seconds);
    }

    [[deprecated("Use get_playback()->play_audio_buffer()")]]
    esp_err_t play_audio_buffer(const uint8_t* buffer, size_t buffer_size, 
                                 uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels, 
                                 i2s_data_bit_width_t buffer_bits,
                                 audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING, 
                                 float duration_limit_seconds = 0.0f) {
        return get_playback()->play_audio_buffer(buffer, buffer_size, buffer_sample_rate_hz, buffer_channels, buffer_bits, mode, duration_limit_seconds);
    }

    [[deprecated("Use get_playback()->is_async_playback_running()")]]
    bool is_async_playback_running() const {
        return playback_ != nullptr && playback_->is_async_playback_running();
    }

    [[deprecated("Use get_playback()->stop_async_playback()")]]
    esp_err_t stop_async_playback() {
        return get_playback()->stop_async_playback();
    }

    [[deprecated("Use get_playback()->clear_audio_pipeline()")]]
    esp_err_t clear_audio_pipeline(uint32_t silence_duration_ms = 100) {
        return get_playback()->clear_audio_pipeline(silence_duration_ms);
    }

    /**
     * @brief 获取播放设备句柄
     * 
     * @return esp_codec_dev_handle_t 播放设备句柄
     */
    esp_codec_dev_handle_t get_play_device_handle() const;

    /**
     * @brief 获取录音设备句柄
     * 
     * @return esp_codec_dev_handle_t 录音设备句柄
     */
    esp_codec_dev_handle_t get_record_device_handle() const;

    /**
     * @brief 检查ES8311是否已初始化
     * 
     * @return bool 返回ES8311初始化状态
     */
    bool is_es8311_initialized() const;

    /**
     * @brief 检查ES7210是否已初始化
     * 
     * @return bool 返回ES7210初始化状态
     */
    bool is_es7210_initialized() const;

    /**
     * @brief 检查音频系统是否已初始化
     * 
     * @return bool 返回音频系统初始化状态
     */
    bool is_system_initialized() const;

    /**
     * @brief 检查ES8311是否在睡眠状态
     * 
     * @return bool 返回ES8311睡眠状态
     */
    bool is_es8311_sleeping() const;

    /**
     * @brief 检查ES7210是否在睡眠状态
     * 
     * @return bool 返回ES7210睡眠状态
     */
    bool is_es7210_sleeping() const;

    /**
     * @brief 设置I2S引脚配置
     * 
     * @param bck_pin I2S BCK引脚
     * @param mck_pin I2S MCK引脚
     * @param data_in_pin I2S数据输入引脚
     * @param data_out_pin I2S数据输出引脚
     * @param ws_pin I2S WS引脚
     * @param pa_pin 功放使能引脚
     */
    void set_i2s_pin_config(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                           gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin);

    // ===== 音频文件信息方法（已迁移到 audio_playback）=====

    [[deprecated("Use get_playback()->get_available_pcm_count()")]]
    int get_available_pcm_count() const {
        return playback_ ? playback_->get_available_pcm_count() : 0;
    }

    [[deprecated("Use get_playback()->get_audio_file_name()")]]
    const char* get_audio_file_name(audio_file_type_t audio_type) const {
        return playback_ ? playback_->get_audio_file_name(audio_type) : "UNKNOWN";
    }

    [[deprecated("Use get_playback()->is_audio_file_available()")]]
    bool is_audio_file_available(audio_file_type_t audio_type) const {
        return playback_ ? playback_->is_audio_file_available(audio_type) : false;
    }

    /**
     * @brief 设置 I2C 总线句柄（在调用各 codec init 前必须设置）
     */
    void set_i2c_bus(i2c_master_bus_handle_t bus) { i2c_bus_handle = bus; }

    /**
     * @brief 设置是否交叉映射 TX/RX 数据脚
     * 硬件若把 SoC 的 DIN 接到 Codec 的 DOUT，需要保持 cross=true（默认）
     */
    void set_i2s_cross_data_pins(bool cross) { i2s_cross_data_pins = cross; }

    /**
     * @brief 设置进入睡眠时是否将 I2S 与 PA 引脚置为高阻
     */
    void set_pins_high_z_on_sleep(bool enable) { pins_high_z_on_sleep = enable; }
    bool get_pins_high_z_on_sleep() const { return pins_high_z_on_sleep; }

    bool es8311_has_dac_path() const { return (es8311_work_mode & ESP_CODEC_DEV_WORK_MODE_DAC) != 0; }
    bool es8311_has_adc_path() const { return (es8311_work_mode & ESP_CODEC_DEV_WORK_MODE_ADC) != 0; }

    /**
     * @brief 设置TX（播放）声道数量
     * 
     * @param channels 声道数量
     * @note 通常应该通过 es8311_init() 设置，而不是直接调用此函数
     */
    void set_tx_channels(audio_channels_t channels) { 
        tx_channels = channels; 
    }

    /**
     * @brief 获取当前TX（播放）声道数量
     * 
     * @return audio_channels_t 当前TX声道数量
     */
    audio_channels_t get_tx_channels() const { return tx_channels; }

    /**
     * @brief 设置RX（录音）声道数量
     * 
     * @param channels 声道数量
     * @note 通常应该通过 es7210_init() 设置，而不是直接调用此函数
     */
    void set_rx_channels(audio_channels_t channels) { 
        rx_channels = channels; 
    }

    /**
     * @brief 获取当前RX（录音）声道数量
     * 
     * @return audio_channels_t 当前RX声道数量
     */
    audio_channels_t get_rx_channels() const { return rx_channels; }

    /**
     * @brief 设置音频声道数量（已废弃）
     * 
     * @deprecated 请使用 set_tx_channels() 和 set_rx_channels() 替代
     * @param channels 声道数量
     */
    [[deprecated("Use set_tx_channels() and set_rx_channels() instead")]]
    void set_audio_channels(audio_channels_t channels) { 
        tx_channels = channels;
        rx_channels = channels;
    }

    /**
     * @brief 获取当前音频声道数量（已废弃）
     * 
     * @deprecated 请使用 get_tx_channels() 和 get_rx_channels() 替代
     * @return audio_channels_t 返回TX声道数量（为了向后兼容）
     */
    [[deprecated("Use get_tx_channels() and get_rx_channels() instead")]]
    audio_channels_t get_audio_channels() const { return tx_channels; }

    /**
     * @brief 设置I2S通道编号
     * 
     * @param port_num I2S通道编号
     */
    void set_i2s_port(i2s_port_t port_num) { i2s_port_num = port_num; }

    /**
     * @brief 获取当前I2S通道编号
     * 
     * @return i2s_port_t 当前I2S通道编号
     */
    i2s_port_t get_i2s_port() const { return i2s_port_num; }

    /**
     * @brief 设置采样率
     * 
     * @param rate 采样率
     */
    void set_sample_rate(audio_sample_rate_t rate) { 
        sample_rate = rate; 
    }

    /**
     * @brief 获取当前采样率
     * 
     * @return audio_sample_rate_t 当前采样率
     */
    audio_sample_rate_t get_sample_rate() const { return sample_rate; }

    /**
     * @brief 设置位深度
     * 
     * @param bits 位深度
     */
    void set_bits_per_sample(i2s_data_bit_width_t bits) { 
        bits_per_sample = bits; 
    }

    /**
     * @brief 获取当前位深度
     * 
     * @return i2s_data_bit_width_t 当前位深度
     */
    i2s_data_bit_width_t get_bits_per_sample() const { return bits_per_sample; }

    /**
     * @brief 设置播放音量
     * 
     * @param volume 播放音量 (0.0 - 100.0)
     * @return esp_err_t 返回操作结果
     */
    esp_err_t set_volume(float volume);

    /**
     * @brief 注册板级功放电源控制回调
     *
     * audio_tools 是通用音频组件，具体板卡通过该回调提供 PA 开关实现。
     *
     * @param callback 回调函数，参数为 (用户上下文, 是否开启)
     * @param arg 用户上下文指针
     */
    void set_pa_power_callback(PaPowerCallback callback, void *arg);

    /**
     * @brief 立即设置功放电源
     *
     * @param on true=打开，false=关闭
     * @return esp_err_t 返回操作结果
     */
    esp_err_t set_pa_power(bool on);

    /**
     * @brief 异步延迟打开功放
     *
     * @param delay_ms 延迟时间，单位 ms
     * @return esp_err_t 返回调度或立即执行结果
     */
    esp_err_t enable_pa_after_delay(uint32_t delay_ms = 500);

    /**
     * @brief 取消尚未执行的异步功放开启动作
     */
    void cancel_pending_pa_enable();

    /**
     * @brief 获取当前播放音量
     * 
     * @return float 当前播放音量 (0.0 - 100.0)
     */
    float get_volume() const { return volume; }

    /**
     * @brief 设置录音增益
     * 
     * @param gain 录音增益 (0.0 - 66.0 dB)
     * @return esp_err_t 返回操作结果
     */
    esp_err_t set_record_gain(float gain);

    /**
     * @brief 获取当前录音增益
     * 
     * @return float 当前录音增益 (0.0 - 66.0 dB)
     */
    float get_record_gain() const { return record_gain; }

    /**
     * @brief 设置音量和录音增益
     * 
     * @param volume 播放音量 (0.0 - 100.0)
     * @param gain 录音增益 (0.0 - 66.0 dB)
     * @return esp_err_t 返回操作结果
     */
    esp_err_t set_audio_levels(float volume, float gain);

    /**
     * @brief 设置ES7210指定麦克风通道的增益
     * 
     * @param mic_channels 要设置的麦克风通道 (audio_mic_channel_t)
     * @param gain 增益值 (es7210_mic_gain_t)
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 设置成功
     *         - ESP_ERR_INVALID_ARG: 参数无效
     *         - ESP_ERR_INVALID_STATE: ES7210未初始化
     * 
     * @example
     * ```cpp
     * es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_1, ES7210_MIC_GAIN_30DB);
     * es7210_set_mic_channel_gain(AUDIO_MIC_CHANNEL_23, ES7210_MIC_GAIN_24DB);
     * ```
     */
    esp_err_t es7210_set_mic_channel_gain(audio_mic_channel_t mic_channels, es7210_mic_gain_t gain);

    /**
     * @brief 设置ES7210麦克风通道
     * 
     * @param channels 麦克风通道选择，可以组合使用多个通道
     * @return esp_err_t 返回操作结果
     */
    // esp_err_t set_mic_channels(audio_mic_channel_t channels); // 已删除，请直接在es7210_init中传入参数

    /**
     * @brief 获取当前麦克风通道配置
     * 
     * @return audio_mic_channel_t 当前麦克风通道配置
     */
    audio_mic_channel_t get_mic_channels() const { return mic_channels; }

    /**
     * @brief 获取麦克风通道的描述字符串
     * 
     * @param channels 麦克风通道配置
     * @return const char* 麦克风通道描述字符串
     */
    const char* get_mic_channels_description(audio_mic_channel_t channels) const;

    /**
     * @brief 检查指定的麦克风通道配置是否有效
     * 
     * @param channels 要检查的麦克风通道配置
     * @return bool 返回通道配置的有效性
     */
    bool is_mic_channels_valid(audio_mic_channel_t channels) const;

    /**
     * @brief 计算当前选择的麦克风数量(按位计数)
     * @return int 麦克风通道个数(0~4)
     */
    int count_selected_mics() const;

    /**
     * @brief 检查 ES7210 是否正在使用 TDM 模式
     * 
     * @return bool 返回 ES7210 的 TDM 模式状态
     *         - true: ES7210 使用 TDM 模式（AUDIO_CHANNELS_3CHs 或 AUDIO_CHANNELS_4CHs）
     *         - false: ES7210 使用标准模式（MONO 或 STEREO）
     */
    bool is_es7210_tdm_mode() const { return es7210_use_tdm; }

    /**
     * @brief 获取 ESP-SR AFE 对象
     * 
     * 用于访问 AEC (回声消除) 等 ESP-SR 音频前端功能。
     * 对象会在首次调用时自动创建。
     * 
     * @return audio_sr_afe* ESP-SR AFE 对象指针,失败返回 nullptr
     * 
     * @example
     * ```cpp
     * audio_sr_afe* afe = audio.get_sr_afe();
     * if (afe) {
     *     afe->aec_init(AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_3, 4, AFE_MODE_LOW_COST);
     *     afe->aec_test_loopback(5);
     * }
     * ```
     */
    audio_sr_afe* get_sr_afe();

    /**
     * @brief 获取播放子对象
     * 
     * 对象会在首次调用时自动创建。
     * @return audio_playback* 播放子对象指针
     */
    audio_playback* get_playback();

    /**
     * @brief 获取录音子对象
     * 
     * 对象会在首次调用时自动创建。
     * @return audio_recorder* 录音子对象指针
     */
    audio_recorder* get_recorder();

    // ========== 未来扩展接口示例 ==========
    // 为接入新的音频设备预留的接口框架
    
    /**
     * @brief 通用音频设备初始化接口（预留）
     * 
     * 当需要支持新的音频设备时，可以实现此接口
     * @param device_type 设备类型标识
     * @param config 设备配置参数
     * @return esp_err_t 返回操作结果
     * 
     * 使用示例：
     * // 未来支持ES9038等新设备时
     * // audio_device_init("ES9038", &es9038_config);
     */
    // esp_err_t audio_device_init(const char* device_type, void* config);
    
    /**
     * @brief 获取支持的音频设备列表（预留）
     * 
     * @return const char** 返回支持的设备类型数组
     */
    // const char** get_supported_devices() const;
};

#endif // __AUDIO_TOOLS_H__

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_ES_TOOLS_H__
#define __AUDIO_ES_TOOLS_H__

//es8311 and es7210 include
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
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

// ESP-SR AEC support
#include "esp_afe_aec.h"

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

/**
 * @brief 音频文件枚举
 * 
 * 定义可播放的音频文件类型
 */
typedef enum {
    AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S = 0,  ///< Candy Wind 1通道 16kHz 16bit 9秒音频文件 (candy_wind_pcm_1ch_16k_16bit_9s.pcm)
    AUDIO_FILE_CANDY_WIND_1CH_44K_16B_45S,     ///< Candy Wind 1通道 44.1kHz 16bit 45.5秒音频文件 (candy_wind_pcm_1ch_44.1k_16bit_45.5s.pcm)
    AUDIO_FILE_CANDY_WIND_2CH_16K_16B_9S,      ///< Candy Wind 2通道 16kHz 16bit 9秒音频文件 (candy_wind_pcm_2ch_16k_16bit_9s.pcm)
    AUDIO_FILE_CANDY_WIND_2CH_44K_16B_45S,     ///< Candy Wind 2通道 44.1kHz 16bit 45.5秒音频文件 (candy_wind_pcm_2ch_44.1k_16bit_45.5s.pcm)
    AUDIO_FILE_STARTUP_1CH_16K_16B_4S,         ///< 启动音频文件 1通道 16kHz 16bit 4秒 (startup_pcm_1ch_16k_16bit_4s.pcm)
    AUDIO_FILE_STARTUP_2CH_16K_16B_4S,         ///< 启动音频文件 2通道 16kHz 16bit 4秒 (startup_pcm_2ch_16k_16bit_4s.pcm)
    AUDIO_FILE_SINE_440HZ_2CH_16K_16B_10S,     ///< 440Hz正弦波音频文件 2通道 16kHz 16bit 10秒 (sine_440Hz_pcm_2ch_16k_16bit_10s.pcm)
    AUDIO_FILE_MAX                             ///< 枚举最大值（用于边界检查）
} audio_file_type_t;

/**
 * @brief 音频播放模式
 *
 * 决定播放接口是阻塞执行还是创建后台任务异步播放
 */
typedef enum {
    AUDIO_PLAYBACK_BLOCKING = 0, ///< 阻塞播放，函数调用完成后表示播放已结束
    AUDIO_PLAYBACK_ASYNC         ///< 异步播放，立即返回并在后台任务中完成播放
} audio_playback_mode_t;

/**
 * @brief 音频声道枚举
 * 
 * 定义音频声道配置
 */
typedef enum {
    AUDIO_CHANNELS_MONO = 1,     ///< 单声道
    AUDIO_CHANNELS_STEREO = 2,   ///< 立体声
    AUDIO_CHANNELS_3CHs = 3,      ///< 3声道
    AUDIO_CHANNELS_4CHs = 4       ///< 4声道
} audio_channels_t;

/**
 * @brief 音频采样率枚举
 * 
 * 定义常用的音频采样率
 */
typedef enum {
    AUDIO_SAMPLE_RATE_8K = 8000,      ///< 8kHz - 电话质量
    AUDIO_SAMPLE_RATE_16K = 16000,    ///< 16kHz - 语音通话
    AUDIO_SAMPLE_RATE_22K = 22050,    ///< 22.05kHz - FM广播质量
    AUDIO_SAMPLE_RATE_32K = 32000,    ///< 32kHz - 数字广播
    AUDIO_SAMPLE_RATE_44K1 = 44100,   ///< 44.1kHz - CD质量
    AUDIO_SAMPLE_RATE_48K = 48000,    ///< 48kHz - 专业音频
    AUDIO_SAMPLE_RATE_88K2 = 88200,   ///< 88.2kHz - 高保真
    AUDIO_SAMPLE_RATE_96K = 96000,    ///< 96kHz - 高保真专业
    AUDIO_SAMPLE_RATE_176K4 = 176400, ///< 176.4kHz - 超高保真
    AUDIO_SAMPLE_RATE_192K = 192000   ///< 192kHz - 超高保真专业
} audio_sample_rate_t;

/**
 * @brief ES7210麦克风通道选择枚举
 * 
 * 定义ES7210 ADC的麦克风输入通道配置
 * 可以组合使用多个通道（使用位或操作）
 */
typedef enum {
    AUDIO_MIC_NONE = 0x00,                    ///< 不选择任何麦克风
    AUDIO_MIC_CHANNEL_1 = 0x01,               ///< 麦克风通道1 (MIC1)
    AUDIO_MIC_CHANNEL_2 = 0x02,               ///< 麦克风通道2 (MIC2) 
    AUDIO_MIC_CHANNEL_3 = 0x04,               ///< 麦克风通道3 (MIC3)
    AUDIO_MIC_CHANNEL_4 = 0x08,               ///< 麦克风通道4 (MIC4)
    AUDIO_MIC_CHANNEL_12 = 0x03,              ///< 麦克风通道1+2
    AUDIO_MIC_CHANNEL_13 = 0x05,              ///< 麦克风通道1+3
    AUDIO_MIC_CHANNEL_14 = 0x09,              ///< 麦克风通道1+4
    AUDIO_MIC_CHANNEL_23 = 0x06,              ///< 麦克风通道2+3
    AUDIO_MIC_CHANNEL_24 = 0x0A,              ///< 麦克风通道2+4
    AUDIO_MIC_CHANNEL_34 = 0x0C,              ///< 麦克风通道3+4
    AUDIO_MIC_CHANNEL_123 = 0x07,             ///< 麦克风通道1+2+3
    AUDIO_MIC_CHANNEL_124 = 0x0B,             ///< 麦克风通道1+2+4
    AUDIO_MIC_CHANNEL_134 = 0x0D,             ///< 麦克风通道1+3+4
    AUDIO_MIC_CHANNEL_234 = 0x0E,             ///< 麦克风通道2+3+4
    AUDIO_MIC_CHANNEL_ALL = 0x0F              ///< 所有麦克风通道1+2+3+4
} audio_mic_channel_t;

/**
 * @brief 录音缓冲拆分结果
 *
 * 为最多4路麦克风通道提供独立的样本缓冲和基础元数据。
 * status 非 ESP_OK 时，其余字段可能为空，应先检查再使用。
 */
typedef struct {
    esp_err_t status;                          ///< 拆分操作结果
    size_t samples_per_channel;                ///< 每个通道的样本数
    size_t bytes_per_sample;                   ///< 原始I2S样本的字节数
    bool is_tdm_mode;                          ///< 是否基于TDM模式拆分
    audio_mic_channel_t enabled_mask;          ///< 参与拆分的麦克风掩码
    int16_t* mic_buffers[4];                   ///< 各通道指向16bit单声道缓冲的指针（未启用时为nullptr）
} channel_split_result_t;

typedef struct {
    bool available;                            ///< 通道是否有有效缓冲
    size_t sample_count;                       ///< 通道样本数量
    int16_t min_value;                         ///< 通道最小采样值
    int16_t max_value;                         ///< 通道最大采样值
    int32_t average_abs_amplitude;             ///< 平均绝对幅度
    double rms_db;                             ///< RMS 电平 (dB)
    double zero_percent;                       ///< 零值占比 (%)
    double clipped_percent;                    ///< 剪裁占比 (%)
} mic_channel_quality_t;

/**
 * @brief ES7210 TDM 模式控制
 */
typedef enum {
    ES7210_TDM_DISABLED = 0,  ///< 禁用TDM，使用标准I2S
    ES7210_TDM_ENABLED = 1    ///< 启用TDM，固定4通道slot
} es7210_tdm_mode_t;

/**
 * @brief ES7210麦克风增益枚举
 * 
 * 定义ES7210 ADC支持的增益档位
 * 寄存器值直接对应档位序号(0-14)
 */
typedef enum {
    ES7210_MIC_GAIN_0DB = 0,        ///< 0dB (寄存器值=0)
    ES7210_MIC_GAIN_3DB = 1,        ///< 3dB (寄存器值=1)
    ES7210_MIC_GAIN_6DB = 2,        ///< 6dB (寄存器值=2)
    ES7210_MIC_GAIN_9DB = 3,        ///< 9dB (寄存器值=3)
    ES7210_MIC_GAIN_12DB = 4,       ///< 12dB (寄存器值=4)
    ES7210_MIC_GAIN_15DB = 5,       ///< 15dB (寄存器值=5)
    ES7210_MIC_GAIN_18DB = 6,       ///< 18dB (寄存器值=6)
    ES7210_MIC_GAIN_21DB = 7,       ///< 21dB (寄存器值=7)
    ES7210_MIC_GAIN_24DB = 8,       ///< 24dB (寄存器值=8)
    ES7210_MIC_GAIN_27DB = 9,       ///< 27dB (寄存器值=9)
    ES7210_MIC_GAIN_30DB = 10,      ///< 30dB (寄存器值=10, 常用值)
    ES7210_MIC_GAIN_33DB = 11,      ///< 33dB (寄存器值=11)
    ES7210_MIC_GAIN_34_5DB = 12,    ///< 34.5dB (寄存器值=12)
    ES7210_MIC_GAIN_36DB = 13,      ///< 36dB (寄存器值=13)
    ES7210_MIC_GAIN_37_5DB = 14     ///< 37.5dB (寄存器值=14, 最大增益)
} es7210_mic_gain_t;

/**
 * @brief audio_es_tools 类
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
 * 1. 创建对应的 audio_es_xxx.cpp 文件
 * 2. 在此类中添加相应的成员变量和初始化函数
 * 3. 确保遵循现有的I2S用户计数管理机制
 * 4. 在audio_system_sleep()中添加对应的睡眠调用
 */
class audio_es_tools {
private:
    esp_codec_dev_handle_t play_dev;        ///< 播放设备句柄（ES8311）
    esp_codec_dev_handle_t record_dev;      ///< 录音设备句柄（ES7210）
    i2s_chan_handle_t tx_handle;            ///< I2S发送通道句柄
    i2s_chan_handle_t rx_handle;            ///< I2S接收通道句柄
    i2c_master_bus_handle_t i2c_bus_handle;        ///< I2C总线句柄
    
    // 独立状态管理
    bool es8311_initialized;                ///< ES8311初始化状态
    bool es7210_initialized;                ///< ES7210初始化状态
    bool system_initialized;                ///< 音频系统初始化状态
    bool es8311_sleeping;                   ///< ES8311睡眠状态
    bool es7210_sleeping;                   ///< ES7210睡眠状态
    
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
    TaskHandle_t playback_task_handle = nullptr;   ///< 异步播放任务句柄
    SemaphoreHandle_t audio_mutex = nullptr;       ///< 音频操作互斥锁
    
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

    struct playback_task_args {
        audio_es_tools* instance;
        audio_file_type_t audio_type;
        float duration_limit_seconds;
    };

    struct buffer_playback_task_args {
        audio_es_tools* instance;
        const uint8_t* buffer;
        size_t buffer_size;
        uint32_t buffer_sample_rate_hz;
        audio_channels_t buffer_channels;
        i2s_data_bit_width_t buffer_bits;
        float duration_limit_seconds;
        bool own_buffer;  ///< 是否需要在任务中释放buffer内存
    };

    // 内部辅助函数
    esp_err_t ensure_i2s_channel();         ///< 确保已创建 I2S 通道
    void try_release_i2s();                 ///< 在引用计数为 0 时释放 I2S 通道
    void incr_i2s_user();                   ///< 增加 I2S 使用者计数
    void decr_i2s_user();                   ///< 减少 I2S 使用者计数
    esp_err_t play_audio_file_impl(audio_file_type_t audio_type, bool check_stop_signal, float duration_limit_seconds); ///< 内部播放实现
    esp_err_t play_audio_buffer_impl(const uint8_t* buffer, size_t buffer_size, 
                                      uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels, 
                                      i2s_data_bit_width_t buffer_bits, 
                                      bool check_stop_signal, float duration_limit_seconds); ///< 内部缓冲区播放实现
    static void playback_task_entry(void* param);                ///< 异步播放任务入口
    static void buffer_playback_task_entry(void* param);         ///< 异步缓冲区播放任务入口
    static void free_channel_split_result(channel_split_result_t& result); ///< 释放拆分结果中的缓冲区
    static channel_split_result_t split_recorded_channels(const uint8_t* record_buffer,
                                                          size_t bytes_read,
                                                          const esp_codec_dev_sample_info_t& fs,
                                                          bool is_tdm_mode,
                                                          audio_mic_channel_t mic_channels); ///< 拆分录音缓冲为独立通道
    
    /**
     * @brief 获取指定音频文件的PCM数据和格式参数
     * 
     * @param audio_type 音频文件类型
     * @param pcm_start 输出：PCM数据起始指针
     * @param pcm_len 输出：PCM数据长度
     * @param file_sample_rate_hz 输出：文件采样率
     * @param file_channels 输出：文件声道数
     * @param file_bits 输出：文件位深
     * @return esp_err_t 返回操作结果
     */
    esp_err_t get_pcm_data_and_format(audio_file_type_t audio_type,
                                       const uint8_t*& pcm_start,
                                       size_t& pcm_len,
                                       uint32_t& file_sample_rate_hz,
                                       audio_channels_t& file_channels,
                                       i2s_data_bit_width_t& file_bits);

public:
    /**
     * @brief 构造函数
     * 
     * 使用默认引脚配置创建 audio_es_tools 对象
     */
    audio_es_tools();

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
    audio_es_tools(gpio_num_t bck_pin, gpio_num_t mck_pin, gpio_num_t data_in_pin, 
                   gpio_num_t data_out_pin, gpio_num_t ws_pin, gpio_num_t pa_pin);

    /**
     * @brief 析构函数
     * 
     * 销毁 audio_es_tools 对象，清理音频资源
     */
    ~audio_es_tools();

    /**
     * @brief 初始化ES8311音频芯片（DAC播放）
     * 
     * @param channels 音频声道配置（单声道或立体声）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es8311_init(audio_channels_t channels);

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
     * @return esp_err_t 返回操作结果
     */
    esp_err_t audio_system_deinit();

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

    /**
     * @brief 录音测试
     * 
     * @param record_duration_ms 录音时长（毫秒）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t record_test(uint32_t record_duration_ms = 3000);

    /**
     * @brief 单次录音并播放测试
     * 
     * 此函数执行简单的录音-播放测试流程：
     * 1. 录制指定时长的音频
     * 2. 分析录制的音频数据（RMS、峰值等）
     * 3. 播放刚录制的音频
     * 
     * 适用场景：
     * - 快速验证麦克风和扬声器功能
     * - 测试音频采集和播放质量
     * - 检查录音延迟和音质
     * 
     * @param record_duration_seconds 录音时长（秒），默认3秒
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 测试成功
     *         - ESP_ERR_INVALID_STATE: 音频设备未初始化
     *         - ESP_ERR_NO_MEM: 内存分配失败
     * 
     * @note 录音和播放之间会有500ms的间隔
     * @note 播放结束后会自动清空播放管线
     * 
     * @example
     * ```cpp
     * // 录音3秒并播放
     * audio_es_tools::record_and_play_test(3);
     * 
     * // 录音5秒并播放
     * audio_es_tools::record_and_play_test(5);
     * ```
     */
    esp_err_t record_and_play_test(uint32_t record_duration_seconds = 3);

    /**
     * @brief 单次录音并播放测试（支持TDM通道选择）
     * 
     * 此函数在TDM模式下录制多个麦克风的数据，然后只播放指定麦克风的音频。
     * 
     * 工作流程：
     * 1. 录制完整的TDM多通道音频数据
     * 2. 从录制的数据中提取指定麦克风通道
     * 3. 分析提取的音频数据质量
     * 4. 播放提取的单通道音频（可选，支持"干运行"模式）
     * 
     * 适用场景：
     * - TDM模式下测试特定麦克风（如AUDIO_MIC_CHANNEL_123时测试MIC3）
     * - 多麦克风阵列中单独验证每个麦克风
     * - 麦克风性能对比测试
     * - 快速分析音频质量而不播放（analysis_only=true）
     * 
     * @param record_duration_seconds 录音时长（秒），默认3秒
     * @param target_mic_channel 目标麦克风通道（使用 audio_mic_channel_t 枚举值）
     *                          - AUDIO_MIC_CHANNEL_1: MIC1
     *                          - AUDIO_MIC_CHANNEL_2: MIC2
     *                          - AUDIO_MIC_CHANNEL_3: MIC3
     *                          - AUDIO_MIC_CHANNEL_4: MIC4
     * @param analysis_only 仅分析模式（默认false=播放音频）
     *                      - false: 正常模式，录音→提取→分析→播放
     *                      - true: 干运行模式，录音→提取→分析（跳过播放）
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 测试成功
     *         - ESP_ERR_INVALID_STATE: 音频设备未初始化或非TDM模式
     *         - ESP_ERR_INVALID_ARG: 目标通道无效或未启用
     *         - ESP_ERR_NO_MEM: 内存分配失败
     * 
     * @note 仅在TDM模式（3个或以上麦克风）下有效
     * @note 在STEREO+TDM模式下，数据格式为8位采样
     * @note 提取的通道会被转换为16位并扩展为立体声
     * @note target_mic_channel必须是单一麦克风通道，不能是组合通道（如AUDIO_MIC_CHANNEL_12）
     * @note analysis_only=true时不需要初始化ES8311播放设备
     * 
     * @example
     * ```cpp
    * // 初始化为TDM模式（3个麦克风）
    * es7210_init(AUDIO_CHANNELS_3CHs, AUDIO_MIC_CHANNEL_123, ES7210_TDM_ENABLED);
     * 
     * // 正常模式：录音5秒并播放MIC3的音频
     * record_and_play_test_with_channel_select(5, AUDIO_MIC_CHANNEL_3, false);
     * 
     * // 仅分析模式：录音5秒，分析MIC2数据，不播放（快速测试）
     * record_and_play_test_with_channel_select(5, AUDIO_MIC_CHANNEL_2, true);
     * 
     * // 批量测试所有麦克风质量（无需播放）
     * for (auto mic : {AUDIO_MIC_CHANNEL_1, AUDIO_MIC_CHANNEL_2, AUDIO_MIC_CHANNEL_3}) {
     *     record_and_play_test_with_channel_select(3, mic, true);
     * }
     * ```
     */
    esp_err_t record_and_play_test_with_channel_select(uint32_t record_duration_seconds = 3, 
                                                        audio_mic_channel_t target_mic_channel = AUDIO_MIC_CHANNEL_1,
                                                        bool analysis_only = false);

    /**
     * @brief 录音并播放录音内容测试
     * 
     * 使用简洁的实现方式，仿照示例代码结构
     * @param record_duration_seconds 录音时长（秒）
     * @param loop_playback 是否循环录音播放：
     *                      false - 单次录音播放模式（录音N秒->播放录音内容）
     *                      true  - 循环录音播放模式（录音N秒->播放N秒->录音N秒->播放N秒...）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t record_and_playback_test(uint32_t record_duration_seconds = 5, bool loop_playback = false);

    /**
     * @brief 计算拆分后四个通道的质量指标
     *
     * @param split_result split_recorded_channels 的输出结构体
     * @param quality 输出：4个通道对应的质量指标数组
     */
    static void compute_split_channel_quality(const channel_split_result_t& split_result,
                                              mic_channel_quality_t quality[4]);

    /**
     * @brief AEC（回声消除）测试函数
     * 
     * 此函数实现了完整的AEC测试流程：
     * 1. 同时录制麦克风输入和参考音频（正在播放的音频）
     * 2. 使用ESP-SR的AEC算法进行回声消除处理
     * 3. 播放经过AEC处理后的音频，并与原始录音对比
     * 
     * 测试场景：在播放音乐的同时说话，AEC会消除播放的音乐回声，只保留说话声音
     * 
     * @param record_duration_seconds 录音时长（秒），默认5秒
     * @param filter_length AEC滤波器长度，推荐值：ESP32P4=4, ESP32C5=2
     *                      值越大CPU负载越高但回声消除效果越好
     * @param aec_mode AEC工作模式：
     *                 - AFE_MODE_LOW_COST: 低功耗模式（适合电池供电设备）
     *                 - AFE_MODE_HIGH_PERF: 高性能模式（更好的回声消除效果）
     * @param play_original_audio 是否播放原始录音：
     *                            - true: 播放原始录音（用于对比效果）
     *                            - false: 播放AEC处理后的音频
     * 
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 测试成功
     *         - ESP_ERR_INVALID_STATE: 音频系统未初始化
     *         - ESP_ERR_NO_MEM: 内存分配失败
     *         - 其他: 底层驱动错误
     * 
     * @note 使用前确保已调用：
     *       - audio_system_init()
     *       - es8311_init()
     *       - es7210_init()
     */
    esp_err_t aec_test(uint32_t record_duration_seconds = 5, int filter_length = 4, 
                       afe_mode_t aec_mode = AFE_MODE_LOW_COST, bool play_original_audio = false);

    /**
     * @brief 测试AEC硬件回采功能
     * 
     * 此函数用于验证ES7210的硬件AEC回采是否正常工作。
     * 功能：
     * - 同时读取CH1(麦克风)和CH3(回采)
     * - 分析两个通道的音频数据特征（RMS、峰值、零值率等）
     * - 可选择性播放各通道数据进行听觉验证
     * - 提供详细诊断信息
     * 
     * @param record_duration_seconds 录音时长（秒），默认3秒
     * @param play_channels 播放选项：0=不播放, 1=仅麦克风, 2=仅回采, 3=两者都播放
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 测试成功
     *         - ESP_ERR_INVALID_STATE: 音频系统未初始化或未启用硬件AEC
     *         - ESP_ERR_NO_MEM: 内存分配失败
     * 
     * @note 要求:
     *       - ES7210必须使用AUDIO_MIC_CHANNEL_13初始化(CH1+CH3)
     *       - audio_system_init必须配置为AUDIO_CHANNELS_STEREO
     * 
     * @example
     * ```cpp
     * // 录音并播放两个通道进行对比
     * audio_es_tools::test_aec_loopback(3, 3);
     * 
     * // 仅录音和分析,不播放
     * audio_es_tools::test_aec_loopback(5, 0);
     * ```
     */
    esp_err_t test_aec_loopback(uint32_t record_duration_seconds = 3, uint8_t play_channels = 3);

    /**
     * @brief 将录音数据保存到指定文件
     *
     * 在当前音频配置下采集指定时长的原始 PCM 数据，并写入到已经挂载的文件系统路径。
     * @param filepath 目标文件的完整路径（例如 "/sdcard/recordings/test.pcm"）
     * @param record_duration_seconds 录音时长（秒）
     * @param chunk_size 每次从编解码器读取并写入文件的字节数，默认 4096
     * @return esp_err_t
     *         - ESP_OK：录音成功写入文件
     *         - ESP_ERR_INVALID_STATE：音频系统或录音设备未初始化
     *         - ESP_ERR_INVALID_ARG：参数非法
     *         - ESP_ERR_TIMEOUT：录音在预期时间内未完成，文件为部分内容
     *         - 其他：底层 I/O 或驱动错误
     */
    esp_err_t record_to_file(const char* filepath, uint32_t record_duration_seconds, size_t chunk_size = 4096);

    /**
     * @brief 播放指定类型的音频文件
     *
     * @param audio_type 要播放的音频文件类型
     * @param mode 播放模式（阻塞或异步）
     * @param duration_limit_seconds 播放时长限制（秒），0 表示播放完整文件
     * @return esp_err_t 返回操作结果
     */
    esp_err_t play_audio_file(audio_file_type_t audio_type, audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING, float duration_limit_seconds = 0.0f);

    /**
     * @brief 播放内存中的音频缓冲区，支持自适应格式转换
     * 
     * 该函数可以播放任意格式的PCM音频数据，并自动转换为系统配置的格式
     * 
     * @param buffer 音频数据缓冲区指针
     * @param buffer_size 缓冲区大小（字节）
     * @param buffer_sample_rate_hz 缓冲区音频采样率（Hz）
     * @param buffer_channels 缓冲区声道配置
     * @param buffer_bits 缓冲区位深配置
     * @param mode 播放模式（阻塞或异步）
     * @param duration_limit_seconds 播放时长限制（秒），0 表示播放完整缓冲区
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 播放成功
     *         - ESP_ERR_INVALID_ARG: 参数无效
     *         - ESP_ERR_INVALID_STATE: 播放设备未就绪或已有异步任务运行
     *         - ESP_ERR_NO_MEM: 内存分配失败
     */
    esp_err_t play_audio_buffer(const uint8_t* buffer, size_t buffer_size, 
                                 uint32_t buffer_sample_rate_hz, audio_channels_t buffer_channels, 
                                 i2s_data_bit_width_t buffer_bits,
                                 audio_playback_mode_t mode = AUDIO_PLAYBACK_BLOCKING, 
                                 float duration_limit_seconds = 0.0f);

    /**
    * @brief 查询是否存在正在运行的异步播放任务
    *
    * @return bool 返回异步播放任务是否正在运行
    */
    bool is_async_playback_running() const { return playback_task_handle != nullptr; }

    /**
     * @brief 停止正在运行的异步播放任务
     * 
     * 强制停止当前正在运行的异步播放任务并清理音频管道
     * @return esp_err_t 返回操作结果
     *         - ESP_OK: 停止成功或没有运行的任务
     *         - ESP_ERR_TIMEOUT: 任务停止超时
     */
    esp_err_t stop_async_playback();

    /**
     * @brief 清理音频播放管道，发送静音数据清除残留音频
     * 
     * @param silence_duration_ms 静音持续时间（毫秒），默认100ms
     * @return esp_err_t 返回操作结果
     */
    esp_err_t clear_audio_pipeline(uint32_t silence_duration_ms = 100);

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

    /**
     * @brief 检查PCM Candy Wind 1通道 16kHz文件是否可用
     * 
     * @return bool 返回PCM文件的可用状态
     */
    bool is_pcm_candy_wind_1ch_16k_available() const;

    /**
     * @brief 检查PCM Candy Wind 1通道 44.1kHz文件是否可用
     * 
     * @return bool 返回PCM文件的可用状态
     */
    bool is_pcm_candy_wind_1ch_44k_available() const;

    /**
     * @brief 检查PCM Candy Wind 2通道 16kHz文件是否可用
     * 
     * @return bool 返回PCM文件的可用状态
     */
    bool is_pcm_candy_wind_2ch_16k_available() const;

    /**
     * @brief 检查PCM Candy Wind 2通道 44.1kHz文件是否可用
     * 
     * @return bool 返回PCM文件的可用状态
     */
    bool is_pcm_candy_wind_2ch_44k_available() const;

    /**
     * @brief 检查PCM启动音频文件 1通道 16kHz是否可用
     * 
     * @return bool 返回PCM启动音频文件的可用状态
     */
    bool is_pcm_startup_1ch_16k_available() const;

    /**
     * @brief 检查PCM启动音频文件 2通道 16kHz是否可用
     * 
     * @return bool 返回PCM启动音频文件的可用状态
     */
    bool is_pcm_startup_2ch_16k_available() const;

    /**
     * @brief 检查PCM 440Hz正弦波文件是否可用
     * 
     * @return bool 返回PCM 440Hz正弦波文件的可用状态
     */
    bool is_pcm_sine_440hz_2ch_16k_16b_10s_available() const;

    /**
     * @brief 获取可用PCM测试文件的数量
     * 
     * @return int 返回可用的PCM测试文件数量
     */
    int get_available_pcm_count() const;

    /**
     * @brief 获取音频文件的名称字符串
     * 
     * @param audio_type 音频文件类型
     * @return const char* 返回音频文件名称
     */
    const char* get_audio_file_name(audio_file_type_t audio_type) const;

    /**
     * @brief 检查指定音频文件是否可用
     * 
     * @param audio_type 音频文件类型
     * @return bool 返回音频文件的可用状态
     */
    bool is_audio_file_available(audio_file_type_t audio_type) const;

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

#endif // __AUDIO_ES_TOOLS_H__
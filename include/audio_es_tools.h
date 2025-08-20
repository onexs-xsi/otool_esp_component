/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_ES_TOOLS_H__
#define __AUDIO_ES_TOOLS_H__

//es8311 and es7210 include
#include "driver/i2s_std.h"
// #include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"

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
    AUDIO_FILE_TEST_A = 0,    ///< 测试音频文件A (test_a.pcm)
    AUDIO_FILE_TEST_B,        ///< 测试音频文件B (test_b.pcm)
    AUDIO_FILE_SINE_440HZ,    ///< 440Hz正弦波音频文件 (sine_440Hz_30s_44100Hz_16bit_1ch.pcm)
    AUDIO_FILE_AUTO,          ///< 自动选择可用的音频文件
    AUDIO_FILE_MAX            ///< 枚举最大值（用于边界检查）
} audio_file_type_t;

/**
 * @brief 音频声道枚举
 * 
 * 定义音频声道配置
 */
typedef enum {
    AUDIO_CHANNELS_MONO = 1,     ///< 单声道
    AUDIO_CHANNELS_STEREO = 2    ///< 立体声
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
 * @brief 音频位深度枚举
 * 
 * 定义音频位深度配置
 */
typedef enum {
    AUDIO_BITS_16 = 16,    ///< 16位 - 标准质量
    AUDIO_BITS_24 = 24,    ///< 24位 - 高质量
    AUDIO_BITS_32 = 32     ///< 32位 - 专业质量
} audio_bits_per_sample_t;

/**
 * @brief audio_es_tools 类
 * 
 * 提供ES8311和ES7210音频芯片的操作功能，包括初始化、播放、录音和睡眠管理
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
    bool i2s_cross_data_pins = true;        ///< 是否使用交叉数据引脚映射（硬件走线导致）
    bool pins_high_z_on_sleep = false;      ///< 进入睡眠时是否将 I2S 与 PA 引脚置为高阻
    bool suppress_release = false;          ///< 在系统整体去初始化期间暂缓 I2S 释放
    
    // I2S 通道配置参数
    i2s_port_t i2s_port_num = I2S_NUM_0;              ///< I2S通道编号
    audio_channels_t audio_channels = AUDIO_CHANNELS_STEREO;    ///< 音频声道数量
    audio_sample_rate_t sample_rate = AUDIO_SAMPLE_RATE_44K1;   ///< 采样率
    audio_bits_per_sample_t bits_per_sample = AUDIO_BITS_16;    ///< 位深度

    // 内部辅助函数
    esp_err_t ensure_i2s_channel();         ///< 确保已创建 I2S 通道
    void try_release_i2s();                 ///< 在引用计数为 0 时释放 I2S 通道
    void incr_i2s_user();                   ///< 增加 I2S 使用者计数
    void decr_i2s_user();                   ///< 减少 I2S 使用者计数

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
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es8311_init();

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
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es7210_init();

    /**
     * @brief 去初始化ES7210音频芯片
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t es7210_deinit();

    /**
     * @brief 初始化音频系统（包含所有已配置的音频模块）
     * 
     * @param i2c_bus_handle I2C总线句柄
     * @param i2s_port_num I2S通道编号
     * @param channels 音频声道数量
     * @param sample_rate 采样率
     * @param bits_per_sample 位深度
     * @return esp_err_t 返回操作结果
     */
    esp_err_t audio_system_init(i2c_master_bus_handle_t i2c_bus_handle, i2s_port_t i2s_port_num, audio_channels_t channels, audio_sample_rate_t sample_rate, audio_bits_per_sample_t bits_per_sample);

    /**
     * @brief 去初始化音频系统
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t audio_system_deinit();

    /**
     * @brief 播放和录音测试
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t play_and_record_test();

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
     * @brief 播放音乐测试
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t play_music_test();

    /**
     * @brief 录音测试
     * 
     * @param record_duration_ms 录音时长（毫秒）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t record_test(uint32_t record_duration_ms = 3000);

    /**
     * @brief 录音并播放录音内容测试
     * 
     * 使用简洁的实现方式，仿照示例代码结构
     * @param record_duration_seconds 录音时长（秒）
     * @return esp_err_t 返回操作结果
     */
    esp_err_t record_and_playback_test(uint32_t record_duration_seconds = 5);

    /**
     * @brief 播放指定类型的音频文件
     * 
     * @param audio_type 要播放的音频文件类型
     * @return esp_err_t 返回操作结果
     */
    esp_err_t play_audio_file(audio_file_type_t audio_type);

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
     * @brief 检查PCM测试文件A是否可用
     * 
     * @return bool 返回PCM测试文件A的可用状态
     */
    bool is_pcm_test_a_available() const;

    /**
     * @brief 检查PCM测试文件B是否可用
     * 
     * @return bool 返回PCM测试文件B的可用状态
     */
    bool is_pcm_test_b_available() const;

    /**
     * @brief 检查PCM 440Hz正弦波文件是否可用
     * 
     * @return bool 返回PCM 440Hz正弦波文件的可用状态
     */
    bool is_pcm_sine_440hz_available() const;

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
     * @brief 播放所有可用的音频文件
     * 
     * 遍历所有音频文件类型并依次播放可用的文件
     * @return esp_err_t 返回操作结果
     */
    esp_err_t play_all_available_files() const;

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
     * @brief 设置音频声道数量
     * 
     * @param channels 声道数量
     */
    void set_audio_channels(audio_channels_t channels) { 
        audio_channels = channels; 
    }

    /**
     * @brief 获取当前音频声道数量
     * 
     * @return audio_channels_t 当前声道数量
     */
    audio_channels_t get_audio_channels() const { return audio_channels; }

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
    void set_bits_per_sample(audio_bits_per_sample_t bits) { 
        bits_per_sample = bits; 
    }

    /**
     * @brief 获取当前位深度
     * 
     * @return audio_bits_per_sample_t 当前位深度
     */
    audio_bits_per_sample_t get_bits_per_sample() const { return bits_per_sample; }
};

#endif // __AUDIO_ES_TOOLS_H__
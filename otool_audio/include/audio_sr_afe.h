/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_SR_AFE_H__
#define __AUDIO_SR_AFE_H__

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_aec.h"
#include "audio_types.h"

// 前置声明,避免循环依赖
class audio_tools;

/**
 * @brief AEC (Acoustic Echo Cancellation) 运行时上下文
 * 
 * 存储当前 AEC 会话的配置和状态信息。
 */
typedef struct {
    bool initialized;                      ///< AEC 是否已初始化
    afe_aec_handle_t* handle;              ///< ESP-SR AEC 句柄指针（与 esp_afe_aec.h 保持一致）
    int filter_length;                     ///< 滤波器长度
    afe_mode_t mode;                       ///< AEC 工作模式
    audio_mic_channel_t mic_channel;       ///< 麦克风通道
    audio_mic_channel_t reference_channel; ///< 参考通道(回声源)
} aec_runtime_context_t;

/**
 * @brief audio_sr_afe 类
 * 
 * 封装 ESP-SR (Speech Recognition) AFE (Audio Front-End) 功能,
 * 主要提供 AEC (Acoustic Echo Cancellation) 回声消除能力。
 * 
 * 架构设计:
 * - 作为 audio_tools 的子对象存在,复用其音频硬件接口
 * - 独立管理 AEC 算法的生命周期和上下文
 * - 提供简洁的初始化/测试/销毁接口
 * 
 * 使用流程:
 * 1. 通过 audio_tools 初始化音频硬件
 * 2. 调用 aec_init() 配置 AEC 参数
 * 3. 调用 aec_test_loopback() 执行测试
 * 4. 销毁时自动调用 aec_deinit()
 */
class audio_sr_afe {
private:
    audio_tools* parent_;                  ///< 父对象指针(用于访问音频硬件接口)
    aec_runtime_context_t aec_ctx_;        ///< AEC 运行时上下文

    /**
     * @brief 将麦克风通道枚举转换为索引
     * 
     * @param channel 麦克风通道枚举
     * @return int 通道索引 (0-3),失败返回 -1
     */
    static int mic_channel_to_index(audio_mic_channel_t channel);

    struct aec_capture_output;

    esp_err_t capture_aec_buffers(uint32_t record_duration_seconds,
                                  aec_capture_output& output);
    void release_aec_buffers(aec_capture_output& output);

public:
    /**
     * @brief 构造函数
     * 
     * @param parent 父 audio_tools 对象指针
     */
    explicit audio_sr_afe(audio_tools* parent);

    /**
     * @brief 析构函数
     * 
     * 自动清理 AEC 资源
     */
    ~audio_sr_afe();

    /**
     * @brief 初始化 AEC (回声消除)
     * 
     * 配置 AEC 算法参数并创建处理句柄。支持指定麦克风通道和参考通道,
     * 参考通道可以是实际麦克风(硬件回采)或静音(软件模式)。
     * 
     * @param mic_channel 麦克风通道(捕获期望信号)
     * @param reference_channel 参考通道(回声源),使用 AUDIO_MIC_NONE 表示静音参考
     * @param filter_length AEC 滤波器长度,推荐值: ESP32P4=4, ESP32C5=2
     * @param aec_mode AEC 工作模式:
     *                 - AFE_MODE_LOW_COST: 低功耗
     *                 - AFE_MODE_HIGH_PERF: 高性能
     * @return esp_err_t
     *         - ESP_OK: 初始化成功
     *         - ESP_ERR_INVALID_ARG: 参数无效
     *         - ESP_ERR_INVALID_STATE: 音频硬件未就绪
     *         - ESP_ERR_NO_MEM: 内存分配失败
     */
    esp_err_t aec_init(audio_mic_channel_t mic_channel,
                       audio_mic_channel_t reference_channel,
                       int filter_length = 4,
                       afe_mode_t aec_mode = AFE_MODE_LOW_COST);

    /**
     * @brief 反初始化 AEC
     * 
     * 释放 AEC 句柄和相关资源
     * @return esp_err_t ESP_OK 表示成功
     */
    esp_err_t aec_deinit();

    /**
     * @brief AEC 回声消除测试
     * 
     * 执行完整的 AEC 测试流程:
     * 1. 录制麦克风和参考音频
     * 2. 执行 AEC 处理
     * 3. 可选播放原始/处理后的音频
     * 
     * @param record_duration_seconds 录音时长(秒)
     * @param play_original_audio 是否播放原始音频
     * @param play_processed_audio 是否播放处理后的音频
     * @return esp_err_t
     *         - ESP_OK: 测试成功
     *         - ESP_ERR_INVALID_STATE: AEC 未初始化
     *         - ESP_ERR_NO_MEM: 内存不足
     */
    esp_err_t aec_test_loopback(uint32_t record_duration_seconds = 5,
                                bool play_original_audio = true,
                                bool play_processed_audio = true);

    /**
     * @brief AEC 测试并保存结果至 SD 卡
     *
     * 录制指定时长音频并执行 AEC 处理, 将麦克风原始信号、参考信号、处理后结果分别写入文件。
     *
     * @param record_duration_seconds 录音时长(秒)
     * @param mount_point SD 卡挂载路径 (默认 /sdcard)
     * @param file_prefix 输出文件前缀 (例如 "aec_test")
     * @return esp_err_t ESP_OK 表示成功
     */
    esp_err_t aec_test_loopback_to_files(uint32_t record_duration_seconds,
                                         const char* mount_point = "/sdcard",
                                         const char* file_prefix = "aec_test");

    /**
     * @brief AEC 便捷测试接口
     * 
     * 自动完成初始化->测试->清理的完整流程,适合快速测试。
     * 
     * @param record_duration_seconds 录音时长(秒)
     * @param mic_channel 麦克风通道
     * @param reference_channel 参考通道
     * @param filter_length 滤波器长度
     * @param aec_mode AEC 模式
     * @param play_original_audio 是否播放原始音频
     * @param play_processed_audio 是否播放处理后的音频
     * @return esp_err_t ESP_OK 表示成功
     */
    esp_err_t aec_test(uint32_t record_duration_seconds = 5,
                       audio_mic_channel_t mic_channel = AUDIO_MIC_CHANNEL_1,
                       audio_mic_channel_t reference_channel = AUDIO_MIC_CHANNEL_3,
                       int filter_length = 4,
                       afe_mode_t aec_mode = AFE_MODE_LOW_COST,
                       bool play_original_audio = true,
                       bool play_processed_audio = true);

    /**
     * @brief 检查 AEC 是否已初始化
     * 
     * @return bool true 表示已初始化
     */
    bool is_initialized() const { return aec_ctx_.initialized; }

    /**
     * @brief 获取当前 AEC 配置的麦克风通道
     * 
     * @return audio_mic_channel_t 麦克风通道
     */
    audio_mic_channel_t get_mic_channel() const { return aec_ctx_.mic_channel; }

    /**
     * @brief 获取当前 AEC 配置的参考通道
     * 
     * @return audio_mic_channel_t 参考通道
     */
    audio_mic_channel_t get_reference_channel() const { return aec_ctx_.reference_channel; }

    /**
     * @brief 兼容旧接口(已废弃)
     * 
     * @deprecated 请使用 aec_init() + aec_test_loopback() 替代
     */
    [[deprecated("Use aec_init() + aec_test_loopback() instead")]]
    esp_err_t test_aec_loopback(uint32_t record_duration_seconds = 3,
                                uint8_t play_channels = 3);
};

#endif // __AUDIO_SR_AFE_H__

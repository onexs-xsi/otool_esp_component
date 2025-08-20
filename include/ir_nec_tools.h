/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_NEC_TOOLS_H__
#define __IR_NEC_TOOLS_H__

#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 默认引脚和参数，可在实例化后修改
#define IR_NEC_DEFAULT_RESOLUTION_HZ   1000000
#define IR_NEC_DEFAULT_TX_GPIO         GPIO_NUM_45
#define IR_NEC_DEFAULT_RX_GPIO         (gpio_num_t)-1
#define IR_NEC_DEFAULT_EN_GPIO         (gpio_num_t)-1

/**
 * @brief 简单的NEC红外发送工具类（仅发送）
 */
class ir_nec_tools {
private:
    bool initialized = false;
    uint32_t resolution_hz = IR_NEC_DEFAULT_RESOLUTION_HZ;
    gpio_num_t tx_gpio = IR_NEC_DEFAULT_TX_GPIO;
    gpio_num_t rx_gpio = IR_NEC_DEFAULT_RX_GPIO; // 预留
    gpio_num_t en_gpio = IR_NEC_DEFAULT_EN_GPIO; // 预留供外部上电/使能

    rmt_channel_handle_t tx_channel = nullptr;
    rmt_encoder_handle_t nec_encoder = nullptr;

public:
    ir_nec_tools() = default;
    ~ir_nec_tools();

    /**
     * @brief 初始化发送通道和编码器（使用默认配置）
     */
    esp_err_t init();

    /**
     * @brief 初始化发送通道和编码器（自定义GPIO配置）
     * @param tx_gpio_pin 发送引脚（可传-1）
     * @param rx_gpio_pin 接收引脚（可传-1）
     * @param en_gpio_pin 使能引脚（可传-1）
     * @note TX和RX不能同时为-1，至少需要配置一个
     */
    esp_err_t init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, gpio_num_t en_gpio_pin);

    /**
     * @brief 反初始化
     */
    esp_err_t deinit();

    /**
     * @brief 发送一个 NEC 扫描码
     */
    esp_err_t send(uint16_t address, uint16_t command);

    bool is_initialized() const { return initialized; }

    void set_resolution(uint32_t hz) { resolution_hz = hz; }
    void set_tx_gpio(gpio_num_t gpio) { tx_gpio = gpio; }
};

#endif // __IR_NEC_TOOLS_H__

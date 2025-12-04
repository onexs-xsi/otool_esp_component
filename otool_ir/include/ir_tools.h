/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_TOOLS_H__
#define __IR_TOOLS_H__

#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ir_codec_core.h"
#include "ir_codec_dispatcher.h"
#include "ir_codec_types.h"
#include "ir_protocol_decoder.h"

using ir_enable_callback_t = void (*)(bool enable);

class ir_tools {
public:
    ir_tools() = default;
    ~ir_tools();

    esp_err_t init();
    esp_err_t init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, gpio_num_t en_gpio_pin);
    esp_err_t init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, ir_enable_callback_t enable_cb);
    esp_err_t deinit();

    esp_err_t start_receive();
    esp_err_t stop_receive();

    bool is_initialized() const { return initialized_; }
    void set_resolution(uint32_t hz) { resolution_hz_ = hz; }
    void set_tx_gpio(gpio_num_t gpio) { tx_gpio_ = gpio; }
    void set_rx_gpio(gpio_num_t gpio) { rx_gpio_ = gpio; }

    esp_err_t register_decoder(IrProtocolDecoderPtr decoder);
    void subscribe(ir_format_t format, ir_decode_callback_t callback = {});
    void unsubscribe(ir_format_t format);
    void set_default_callback(ir_decode_callback_t callback);

    esp_err_t send(const ir_send_request_t &request);

private:
    static constexpr uint32_t IR_DEFAULT_RESOLUTION_HZ = 1000000;
    static constexpr gpio_num_t IR_DEFAULT_TX_GPIO = static_cast<gpio_num_t>(-1);
    static constexpr gpio_num_t IR_DEFAULT_RX_GPIO = static_cast<gpio_num_t>(-1);
    static constexpr gpio_num_t IR_DEFAULT_EN_GPIO = static_cast<gpio_num_t>(-1);

    bool initialized_ = false;
    uint32_t resolution_hz_ = IR_DEFAULT_RESOLUTION_HZ;
    gpio_num_t tx_gpio_ = IR_DEFAULT_TX_GPIO;
    gpio_num_t rx_gpio_ = IR_DEFAULT_RX_GPIO;
    gpio_num_t en_gpio_ = IR_DEFAULT_EN_GPIO;

    rmt_channel_handle_t tx_channel_ = nullptr;
    rmt_channel_handle_t rx_channel_ = nullptr;
    ir_encoder_context *encoder_ctx_ = nullptr;
    ir_decoder_context *decoder_ctx_ = nullptr;

    QueueHandle_t receive_queue_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    volatile bool rx_task_should_exit_ = false;

    ir_enable_callback_t enable_callback_ = nullptr;

    esp_err_t init_tx_channel();
    esp_err_t init_rx_channel();
    esp_err_t configure_rx_carrier_for_channel();
    esp_err_t reconfigure_rx_carrier();
    static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    static void rx_task(void *pvParameters);
    bool handle_received_symbols(const rmt_symbol_word_t *symbols, size_t symbol_num);
    esp_err_t ensure_decoder_context();
    esp_err_t ensure_encoder_context();
};

#endif // __IR_TOOLS_H__

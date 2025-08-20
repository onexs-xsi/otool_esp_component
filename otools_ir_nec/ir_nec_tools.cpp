/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "ir_nec_tools.h"
#include "ir_nec_encoder.h"
#include "driver/gpio.h"
#include "esp_check.h"

static const char *IR_TAG = "ir_nec_tools";

ir_nec_tools::~ir_nec_tools() {
    deinit();
}

esp_err_t ir_nec_tools::init() {
    // 使用默认配置调用重载版本
    return init(IR_NEC_DEFAULT_TX_GPIO, IR_NEC_DEFAULT_RX_GPIO, IR_NEC_DEFAULT_EN_GPIO);
}

esp_err_t ir_nec_tools::init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, gpio_num_t en_gpio_pin) {
    if (initialized) return ESP_OK;

    // 检查TX和RX都为-1的情况，这是无效配置
    if (tx_gpio_pin == (gpio_num_t)-1 && rx_gpio_pin == (gpio_num_t)-1) {
        ESP_LOGE(IR_TAG, "Configuration failed: both TX and RX GPIO are -1, at least one must be valid");
        return ESP_ERR_INVALID_ARG;
    }

    // 应用传入的配置
    resolution_hz = IR_NEC_DEFAULT_RESOLUTION_HZ;
    tx_gpio = tx_gpio_pin;
    rx_gpio = rx_gpio_pin;
    en_gpio = en_gpio_pin;

    // 配置TX GPIO引脚
    if (tx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring TX GPIO: %d", (int)tx_gpio);
        gpio_reset_pin(tx_gpio);
        gpio_set_direction(tx_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(tx_gpio, 0);  // 初始为低电平
    }

    // 配置RX GPIO引脚
    if (rx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring RX GPIO: %d", (int)rx_gpio);
        gpio_reset_pin(rx_gpio);
        gpio_set_direction(rx_gpio, GPIO_MODE_INPUT);
    }

    // 配置使能引脚
    if (en_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring EN GPIO: %d", (int)en_gpio);
        gpio_reset_pin(en_gpio);
        gpio_set_direction(en_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio, 1);  // 使能IR模块
    }



    ESP_LOGI(IR_TAG, "create RMT TX channel gpio=%d res=%u", (int)tx_gpio, (unsigned)resolution_hz);
    rmt_tx_channel_config_t tx_channel_cfg = {
        .gpio_num = tx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        .flags = {
            .invert_out = true, // NEC 典型需要反相
        }
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_channel_cfg, &tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "new tx channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = 38000,
        .duty_cycle = 0.5f,
        .flags = {}
    };
    ret = rmt_apply_carrier(tx_channel, &carrier_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "apply carrier failed: %s", esp_err_to_name(ret));
        rmt_del_channel(tx_channel);
        return ret;
    }

    ir_nec_encoder_config_t nec_encoder_cfg = { .resolution = resolution_hz };
    ret = rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "new nec encoder failed: %s", esp_err_to_name(ret));
        rmt_del_channel(tx_channel);
        return ret;
    }

    ret = rmt_enable(tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "enable tx failed: %s", esp_err_to_name(ret));
        rmt_del_encoder(nec_encoder);
        rmt_del_channel(tx_channel);
        return ret;
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t ir_nec_tools::deinit() {
    if (!initialized) return ESP_OK;
    if (nec_encoder) {
        rmt_del_encoder(nec_encoder);
        nec_encoder = nullptr;
    }
    if (tx_channel) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = nullptr;
    }
    initialized = false;
    return ESP_OK;
}

esp_err_t ir_nec_tools::send(uint16_t address, uint16_t command) {
    if (!initialized) {
        esp_err_t err = init();
        if (err != ESP_OK) return err;
    }
    ir_nec_scan_code_t scan_code = { .address = address, .command = command };
    rmt_transmit_config_t transmit_config = { 
        .loop_count = 0,
        .flags = {}
    };
    return rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config);
}

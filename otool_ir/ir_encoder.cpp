/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ir_codec_dispatcher.h"

#include <array>
#include <map>
#include <new>

#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "format/nec/nec_encoder.h"
#include "format/rc5/rc5_encoder.h"

static const char *TAG = "ir_encoder";

namespace {

struct EncoderHolder {
    std::map<ir_format_t, rmt_encoder_handle_t> encoders;
    uint32_t resolution_hz{0};
    rmt_channel_handle_t tx_channel{nullptr};
};

esp_err_t ensure_nec_encoder(EncoderHolder *holder, rmt_encoder_handle_t &handle) {
    auto it = holder->encoders.find(ir_format_t::NEC);
    if (it != holder->encoders.end()) {
        handle = it->second;
        return ESP_OK;
    }
    ir_nec_encoder_config_t cfg = {
        .resolution = holder->resolution_hz,
    };
    rmt_encoder_handle_t nec_handle = nullptr;
    ESP_RETURN_ON_ERROR(rmt_new_ir_nec_encoder(&cfg, &nec_handle), TAG, "failed to create NEC encoder");
    holder->encoders[ir_format_t::NEC] = nec_handle;
    handle = nec_handle;
    return ESP_OK;
}

esp_err_t ensure_rc5_encoder(EncoderHolder *holder, rmt_encoder_handle_t &handle) {
    auto it = holder->encoders.find(ir_format_t::RC5);
    if (it != holder->encoders.end()) {
        handle = it->second;
        return ESP_OK;
    }

    rmt_encoder_handle_t copy_handle = nullptr;
    rmt_copy_encoder_config_t config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&config, &copy_handle), TAG, "create RC5 encoder failed");
    holder->encoders[ir_format_t::RC5] = copy_handle;
    handle = copy_handle;
    return ESP_OK;
}

void destroy_encoders(EncoderHolder *holder) {
    for (auto &entry : holder->encoders) {
        if (entry.second) {
            rmt_del_encoder(entry.second);
        }
    }
    holder->encoders.clear();
}

} // namespace

ir_encoder_context *ir_encoder_create(uint32_t resolution_hz) {
    auto *ctx = new (std::nothrow) EncoderHolder();
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate encoder context");
        return nullptr;
    }
    ctx->resolution_hz = resolution_hz;
    return reinterpret_cast<ir_encoder_context *>(ctx);
}

void ir_encoder_destroy(ir_encoder_context *ctx) {
    if (!ctx) {
        return;
    }
    auto *holder = reinterpret_cast<EncoderHolder *>(ctx);
    destroy_encoders(holder);
    delete holder;
}

void ir_encoder_set_resolution(ir_encoder_context *ctx, uint32_t resolution_hz) {
    if (!ctx) {
        return;
    }
    auto *holder = reinterpret_cast<EncoderHolder *>(ctx);
    if (holder->resolution_hz == resolution_hz) {
        return;
    }
    holder->resolution_hz = resolution_hz;
    destroy_encoders(holder);
}

esp_err_t ir_encoder_send(ir_encoder_context *ctx,
                          rmt_channel_handle_t channel,
                          const ir_send_request_t &request) {
    if (!ctx || channel == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    auto *holder = reinterpret_cast<EncoderHolder *>(ctx);
    holder->tx_channel = channel;
    rmt_encoder_handle_t encoder = nullptr;

    // Apply protocol-specific carrier frequency
    rmt_carrier_config_t carrier_cfg{};
    carrier_cfg.duty_cycle = 0.5f;
    switch (request.format) {
        case ir_format_t::NEC:
            carrier_cfg.frequency_hz = 38000;
            break;
        case ir_format_t::RC5:
            carrier_cfg.frequency_hz = 36000;
            break;
        default:
            carrier_cfg.frequency_hz = 38000;
            break;
    }
    ESP_RETURN_ON_ERROR(rmt_apply_carrier(channel, &carrier_cfg), TAG, "apply carrier failed");

    switch (request.format) {
        case ir_format_t::NEC: {
            ESP_RETURN_ON_ERROR(ensure_nec_encoder(holder, encoder), TAG, "ensure NEC encoder failed");
            ir_nec_scan_code_t scan_code{};
            scan_code.address = static_cast<uint16_t>(request.address & 0xFFFFu);
            scan_code.command = static_cast<uint16_t>(request.command & 0xFFFFu);

            rmt_transmit_config_t config = {};
            config.loop_count = request.is_repeat ? 1 : 0;
            return rmt_transmit(channel, encoder, &scan_code, sizeof(scan_code), &config);
        }
        case ir_format_t::RC5: {
            ESP_RETURN_ON_ERROR(ensure_rc5_encoder(holder, encoder), TAG, "ensure RC5 encoder failed");
            std::array<rmt_symbol_word_t, rc5::RC5_MAX_SYMBOLS> symbols{};
            size_t symbol_count = 0;
            ESP_RETURN_ON_ERROR(rc5::build_frame_symbols(request, holder->resolution_hz, symbols, symbol_count), TAG, "build RC5 frame failed");
            rmt_transmit_config_t config = {};
            return rmt_transmit(channel,
                                encoder,
                                symbols.data(),
                                symbol_count * sizeof(rmt_symbol_word_t),
                                &config);
        }
        default:
            ESP_LOGW(TAG, "Format %d not supported for TX", static_cast<int>(request.format));
            return ESP_ERR_NOT_SUPPORTED;
    }
}

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nec_encoder.h"

#include <cstdlib>

#include "esp_check.h"

namespace {

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_encoder_t *bytes_encoder;
    rmt_symbol_word_t nec_leading_symbol;
    rmt_symbol_word_t nec_ending_symbol;
    int state;
} rmt_ir_nec_encoder_t;

static size_t rmt_encode_ir_nec(rmt_encoder_t *encoder,
                                rmt_channel_handle_t channel,
                                const void *primary_data,
                                size_t data_size,
                                rmt_encode_state_t *ret_state) {
    auto *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    (void)data_size;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    uint32_t state_flags = static_cast<uint32_t>(RMT_ENCODING_RESET);
    size_t encoded_symbols = 0;
    auto *scan_code = static_cast<const ir_nec_scan_code_t *>(primary_data);
    rmt_encoder_handle_t copy_encoder = nec_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = nec_encoder->bytes_encoder;
    switch (nec_encoder->state) {
        case 0:
            encoded_symbols += copy_encoder->encode(copy_encoder,
                                                    channel,
                                                    &nec_encoder->nec_leading_symbol,
                                                    sizeof(rmt_symbol_word_t),
                                                    &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                nec_encoder->state = 1;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state_flags |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
            // fallthrough
        case 1:
            encoded_symbols += bytes_encoder->encode(bytes_encoder,
                                                     channel,
                                                     &scan_code->address,
                                                     sizeof(uint16_t),
                                                     &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                nec_encoder->state = 2;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state_flags |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
            // fallthrough
        case 2:
            encoded_symbols += bytes_encoder->encode(bytes_encoder,
                                                     channel,
                                                     &scan_code->command,
                                                     sizeof(uint16_t),
                                                     &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                nec_encoder->state = 3;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state_flags |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
            // fallthrough
        case 3:
            encoded_symbols += copy_encoder->encode(copy_encoder,
                                                    channel,
                                                    &nec_encoder->nec_ending_symbol,
                                                    sizeof(rmt_symbol_word_t),
                                                    &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                nec_encoder->state = RMT_ENCODING_RESET;
                state_flags |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state_flags |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
            break;
        default:
            break;
    }

out:
    *ret_state = static_cast<rmt_encode_state_t>(state_flags);
    return encoded_symbols;
}

static esp_err_t rmt_del_ir_nec_encoder(rmt_encoder_t *encoder) {
    auto *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    rmt_del_encoder(nec_encoder->copy_encoder);
    rmt_del_encoder(nec_encoder->bytes_encoder);
    free(nec_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ir_nec_encoder_reset(rmt_encoder_t *encoder) {
    auto *nec_encoder = __containerof(encoder, rmt_ir_nec_encoder_t, base);
    rmt_encoder_reset(nec_encoder->copy_encoder);
    rmt_encoder_reset(nec_encoder->bytes_encoder);
    nec_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

} // namespace

esp_err_t rmt_new_ir_nec_encoder(const ir_nec_encoder_config_t *config,
                                 rmt_encoder_handle_t *ret_encoder) {
    static const char *TAG = "nec_encoder";
    ESP_RETURN_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    auto *nec_encoder = static_cast<rmt_ir_nec_encoder_t *>(
        rmt_alloc_encoder_mem(sizeof(rmt_ir_nec_encoder_t)));
    ESP_RETURN_ON_FALSE(nec_encoder, ESP_ERR_NO_MEM, TAG, "no mem for ir nec encoder");
    *nec_encoder = {};

    esp_err_t ret = ESP_OK;
    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_bytes_encoder_config_t bytes_encoder_config = {};
    nec_encoder->base.encode = rmt_encode_ir_nec;
    nec_encoder->base.del = rmt_del_ir_nec_encoder;
    nec_encoder->base.reset = rmt_ir_nec_encoder_reset;
    nec_encoder->state = RMT_ENCODING_RESET;

    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &nec_encoder->copy_encoder),
                      err,
                      TAG,
                      "create copy encoder failed");

    nec_encoder->nec_leading_symbol = {};
    nec_encoder->nec_leading_symbol.duration0 = 9000ULL * config->resolution / 1000000;
    nec_encoder->nec_leading_symbol.level0 = 1;
    nec_encoder->nec_leading_symbol.duration1 = 4500ULL * config->resolution / 1000000;
    nec_encoder->nec_leading_symbol.level1 = 0;

    nec_encoder->nec_ending_symbol = {};
    nec_encoder->nec_ending_symbol.duration0 = 560 * config->resolution / 1000000;
    nec_encoder->nec_ending_symbol.level0 = 1;
    nec_encoder->nec_ending_symbol.duration1 = 0x7FFF;
    nec_encoder->nec_ending_symbol.level1 = 0;

    bytes_encoder_config.bit0.duration0 = 560 * config->resolution / 1000000;
    bytes_encoder_config.bit0.level0 = 1;
    bytes_encoder_config.bit0.duration1 = 560 * config->resolution / 1000000;
    bytes_encoder_config.bit0.level1 = 0;

    bytes_encoder_config.bit1.duration0 = 560 * config->resolution / 1000000;
    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.duration1 = 1690 * config->resolution / 1000000;
    bytes_encoder_config.bit1.level1 = 0;
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &nec_encoder->bytes_encoder),
                      err,
                      TAG,
                      "create bytes encoder failed");

    *ret_encoder = &nec_encoder->base;
    return ESP_OK;
err:
    if (nec_encoder) {
        if (nec_encoder->bytes_encoder) {
            rmt_del_encoder(nec_encoder->bytes_encoder);
        }
        if (nec_encoder->copy_encoder) {
            rmt_del_encoder(nec_encoder->copy_encoder);
        }
        free(nec_encoder);
    }
    return ret;
}

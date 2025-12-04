/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_CODEC_DISPATCHER_H__
#define __IR_CODEC_DISPATCHER_H__

#include "driver/rmt_types.h"
#include "esp_err.h"
#include "ir_codec_core.h"
#include "ir_protocol_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

ir_encoder_context *ir_encoder_create(uint32_t resolution_hz);
void ir_encoder_destroy(ir_encoder_context *ctx);
void ir_encoder_set_resolution(ir_encoder_context *ctx, uint32_t resolution_hz);
esp_err_t ir_encoder_send(ir_encoder_context *ctx,
                          rmt_channel_handle_t channel,
                          const ir_send_request_t &request);

ir_decoder_context *ir_decoder_create();
void ir_decoder_destroy(ir_decoder_context *ctx);
esp_err_t ir_decoder_register_decoder(ir_decoder_context *ctx, IrProtocolDecoderPtr decoder);
void ir_decoder_subscribe(ir_decoder_context *ctx, ir_format_t format, ir_decode_callback_t callback);
void ir_decoder_unsubscribe(ir_decoder_context *ctx, ir_format_t format);
void ir_decoder_set_default_callback(ir_decoder_context *ctx, ir_decode_callback_t callback);
size_t ir_decoder_get_subscribed_count(const ir_decoder_context *ctx);
ir_format_t ir_decoder_get_first_subscribed_format(const ir_decoder_context *ctx);
bool ir_decoder_handle_symbols(ir_decoder_context *ctx,
                               const rmt_symbol_word_t *symbols,
                               size_t symbol_num);

#ifdef __cplusplus
}
#endif

#endif // __IR_CODEC_DISPATCHER_H__

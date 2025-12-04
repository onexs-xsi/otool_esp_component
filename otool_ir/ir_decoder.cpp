/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#include "ir_codec_dispatcher.h"

#include <algorithm>
#include <map>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include "esp_log.h"
#include "format/nec/nec_decoder.h"
#include "format/rc5/rc5_decoder.h"

struct ir_decoder_context {
    std::vector<IrProtocolDecoderPtr> decoders;
    std::map<ir_format_t, ir_decode_callback_t> callbacks;
    ir_decode_callback_t default_callback;
    bool builtin_ready{false};
    size_t subscribed_formats_count{0};
};

namespace {
static const char *IR_DEC_TAG = "ir_decoder";

static void dispatch_result(ir_decoder_context *ctx, const ir_decode_result_t &result) {
    auto it = ctx->callbacks.find(result.format);
    if (it != ctx->callbacks.end() && it->second) {
        it->second(result);
        return;
    }
    if (ctx->default_callback) {
        ctx->default_callback(result);
    }
}

static void ensure_builtin_decoders(ir_decoder_context *ctx) {
    if (!ctx || ctx->builtin_ready) {
        return;
    }
    auto ensure_format = [&](ir_format_t format, auto factory) {
        bool exists = std::any_of(ctx->decoders.begin(), ctx->decoders.end(), [format](const IrProtocolDecoderPtr &decoder) {
            return decoder && decoder->format() == format;
        });
        if (!exists) {
            ctx->decoders.emplace_back(factory());
        }
    };

    ensure_format(ir_format_t::NEC, [] { return create_nec_decoder(); });
    ensure_format(ir_format_t::RC5, [] { return create_rc5_decoder(); });

    ctx->builtin_ready = true;
}
}

ir_decoder_context *ir_decoder_create() {
    auto *ctx = new (std::nothrow) ir_decoder_context();
    if (!ctx) {
        ESP_LOGE(IR_DEC_TAG, "Failed to allocate decoder context");
    }
    return ctx;
}

void ir_decoder_destroy(ir_decoder_context *ctx) {
    delete ctx;
}

esp_err_t ir_decoder_register_decoder(ir_decoder_context *ctx, IrProtocolDecoderPtr decoder) {
    if (!ctx || !decoder) {
        return ESP_ERR_INVALID_ARG;
    }
    ir_format_t target = decoder->format();
    bool exists = std::any_of(ctx->decoders.begin(), ctx->decoders.end(), [target](const IrProtocolDecoderPtr &item) {
        return item && item->format() == target;
    });
    if (exists) {
        ESP_LOGW(IR_DEC_TAG, "Decoder for format %d already exists", static_cast<int>(target));
        return ESP_ERR_INVALID_STATE;
    }
    ctx->decoders.emplace_back(std::move(decoder));
    return ESP_OK;
}

void ir_decoder_subscribe(ir_decoder_context *ctx, ir_format_t format, ir_decode_callback_t callback) {
    if (!ctx) {
        return;
    }
    bool was_subscribed = (ctx->callbacks.find(format) != ctx->callbacks.end());
    if (callback) {
        ctx->callbacks[format] = std::move(callback);
        if (!was_subscribed) {
            ctx->subscribed_formats_count++;
        }
    } else {
        if (was_subscribed) {
            ctx->subscribed_formats_count--;
        }
        ctx->callbacks.erase(format);
    }
}

void ir_decoder_unsubscribe(ir_decoder_context *ctx, ir_format_t format) {
    if (!ctx) {
        return;
    }
    if (ctx->callbacks.find(format) != ctx->callbacks.end()) {
        ctx->subscribed_formats_count--;
    }
    ctx->callbacks.erase(format);
}

void ir_decoder_set_default_callback(ir_decoder_context *ctx, ir_decode_callback_t callback) {
    if (!ctx) {
        return;
    }
    ctx->default_callback = std::move(callback);
}

size_t ir_decoder_get_subscribed_count(const ir_decoder_context *ctx) {
    return ctx ? ctx->subscribed_formats_count : 0;
}

ir_format_t ir_decoder_get_first_subscribed_format(const ir_decoder_context *ctx) {
    if (!ctx || ctx->callbacks.empty()) {
        return ir_format_t::UNKNOWN;
    }
    return ctx->callbacks.begin()->first;
}

bool ir_decoder_handle_symbols(ir_decoder_context *ctx,
                               const rmt_symbol_word_t *symbols,
                               size_t symbol_num) {
    if (!ctx || !symbols || symbol_num == 0) {
        return false;
    }
    ensure_builtin_decoders(ctx);
    for (const auto &decoder : ctx->decoders) {
        if (!decoder) {
            continue;
        }
        ir_decode_result_t result;
        if (decoder->decode(symbols, symbol_num, result)) {
            dispatch_result(ctx, result);
            return true;
        }
    }
    return false;
}

/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_CODEC_CORE_H__
#define __IR_CODEC_CORE_H__

#include <functional>
#include "ir_codec_types.h"

using ir_decode_callback_t = std::function<void(const ir_decode_result_t &result)>;

struct ir_encoder_context;
struct ir_decoder_context;

#endif // __IR_CODEC_CORE_H__

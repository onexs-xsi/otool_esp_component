/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_PROTOCOL_DECODER_H__
#define __IR_PROTOCOL_DECODER_H__

#include <cstddef>
#include <memory>
#include "driver/rmt_types.h"
#include "ir_codec_types.h"

class IrProtocolDecoder {
public:
    virtual ~IrProtocolDecoder() = default;
    virtual ir_format_t format() const = 0;
    virtual bool decode(const rmt_symbol_word_t *symbols,
                        size_t symbol_num,
                        ir_decode_result_t &result) = 0;
};

using IrProtocolDecoderPtr = std::unique_ptr<IrProtocolDecoder>;

#endif // __IR_PROTOCOL_DECODER_H__

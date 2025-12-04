/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#ifndef __IR_CODEC_TYPES_H__
#define __IR_CODEC_TYPES_H__

#include <array>
#include <cstddef>
#include <cstdint>

static constexpr size_t IR_MAX_PAYLOAD_BYTES = 32;

enum class ir_format_t : uint8_t {
    UNKNOWN = 0,
    NEC,
    RC5,
};

static inline const char *ir_format_name(ir_format_t format) {
    switch (format) {
        case ir_format_t::NEC:
            return "NEC";
        case ir_format_t::RC5:
            return "RC5";
        case ir_format_t::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

struct ir_decode_result_t {
    ir_format_t format{ir_format_t::UNKNOWN};
    bool is_repeat{false};
    uint32_t address{0};
    uint32_t command{0};
    uint32_t raw_value{0};
    size_t payload_length{0};
    std::array<uint8_t, IR_MAX_PAYLOAD_BYTES> payload{};
};

struct ir_send_request_t {
    ir_format_t format{ir_format_t::UNKNOWN};
    uint32_t address{0};
    uint32_t command{0};
    const uint8_t *payload{nullptr};
    size_t payload_length{0};
    bool is_repeat{false};
};

#endif // __IR_CODEC_TYPES_H__

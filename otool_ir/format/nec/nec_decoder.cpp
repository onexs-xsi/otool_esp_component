/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#include "nec_decoder.h"

#include <algorithm>
#include <cstring>
#include "esp_log.h"

namespace {
static const char *NEC_DECODER_TAG = "nec_decoder";
static constexpr uint32_t IR_NEC_DECODE_MARGIN = 200; // us
static constexpr uint32_t NEC_LEADING_CODE_DURATION_0 = 9000;
static constexpr uint32_t NEC_LEADING_CODE_DURATION_1 = 4500;
static constexpr uint32_t NEC_PAYLOAD_ZERO_DURATION_0 = 560;
static constexpr uint32_t NEC_PAYLOAD_ZERO_DURATION_1 = 560;
static constexpr uint32_t NEC_PAYLOAD_ONE_DURATION_0 = 560;
static constexpr uint32_t NEC_PAYLOAD_ONE_DURATION_1 = 1690;
static constexpr uint32_t NEC_REPEAT_CODE_DURATION_0 = 9000;
static constexpr uint32_t NEC_REPEAT_CODE_DURATION_1 = 2250;
static constexpr size_t NEC_MAX_DATA_BYTES = IR_MAX_PAYLOAD_BYTES;

class NecDecoder final : public IrProtocolDecoder {
public:
    NecDecoder() = default;
    ~NecDecoder() override = default;

    ir_format_t format() const override { return ir_format_t::NEC; }

    bool decode(const rmt_symbol_word_t *symbols,
                size_t symbol_num,
                ir_decode_result_t &result) override {
        if (symbols == nullptr || symbol_num == 0) {
            return false;
        }

        if (symbol_num == 2) {
            if (parse_repeat_frame(symbols)) {
                result = make_result(last_address_, last_command_, true, nullptr, 0);
                return true;
            }
            return false;
        }

        if (symbol_num < 17) {
            ESP_LOGV(NEC_DECODER_TAG, "Frame too short: %d symbols", (int)symbol_num);
            return false;
        }

        const rmt_symbol_word_t *cur = symbols;
        if (!check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) ||
            !check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1)) {
            ESP_LOGV(NEC_DECODER_TAG, "Invalid leading code: %u/%u", cur->duration0, cur->duration1);
            return false;
        }
        cur++;

        uint16_t address = 0;
        for (int i = 0; i < 16; ++i, ++cur) {
            if (parse_logic1(cur)) {
                address |= 1 << i;
            } else if (parse_logic0(cur)) {
                address &= ~(1 << i);
            } else {
                ESP_LOGV(NEC_DECODER_TAG, "Address bit %d invalid", i);
                return false;
            }
        }

        size_t remaining_symbols = symbol_num - 17;
        size_t data_bytes = remaining_symbols / 8;
        if (data_bytes == 0 || data_bytes > NEC_MAX_DATA_BYTES) {
            ESP_LOGV(NEC_DECODER_TAG, "Invalid data length=%d", (int)data_bytes);
            return false;
        }

        uint8_t data[NEC_MAX_DATA_BYTES] = {0};
        for (size_t byte_idx = 0; byte_idx < data_bytes; ++byte_idx) {
            uint8_t byte_val = 0;
            for (int bit = 0; bit < 8; ++bit, ++cur) {
                if (parse_logic1(cur)) {
                    byte_val |= 1 << bit;
                } else if (parse_logic0(cur)) {
                    byte_val &= ~(1 << bit);
                } else {
                    ESP_LOGV(NEC_DECODER_TAG, "Data[%d] bit %d invalid", (int)byte_idx, bit);
                    return false;
                }
            }
            data[byte_idx] = byte_val;
        }

        last_address_ = address;
        last_command_ = data[0];

        bool is_standard_nec = (symbol_num == 34 && data_bytes == 2);
        if (is_standard_nec) {
            uint16_t command = static_cast<uint16_t>(data[0] | (data[1] << 8));
            last_command_ = command;
            result = make_result(address, command, false, data, data_bytes);
            return true;
        }

        result = make_result(address, data[0], false, data, data_bytes);
        return true;
    }

private:
    bool check_in_range(uint32_t signal_duration, uint32_t spec_duration) const {
        return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
               (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
    }

    bool parse_logic0(const rmt_symbol_word_t *symbol) const {
        return check_in_range(symbol->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
               check_in_range(symbol->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
    }

    bool parse_logic1(const rmt_symbol_word_t *symbol) const {
        return check_in_range(symbol->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
               check_in_range(symbol->duration1, NEC_PAYLOAD_ONE_DURATION_1);
    }

    bool parse_repeat_frame(const rmt_symbol_word_t *symbol) const {
        return check_in_range(symbol->duration0, NEC_REPEAT_CODE_DURATION_0) &&
               check_in_range(symbol->duration1, NEC_REPEAT_CODE_DURATION_1);
    }

    ir_decode_result_t make_result(uint16_t address,
                                   uint16_t command,
                                   bool is_repeat,
                                   const uint8_t *payload,
                                   size_t payload_length) const {
        ir_decode_result_t result;
        result.format = ir_format_t::NEC;
        result.is_repeat = is_repeat;
        result.address = address;
        result.command = command;
        result.raw_value = (static_cast<uint32_t>(address) << 16) | command;
        result.payload_length = std::min(payload_length, IR_MAX_PAYLOAD_BYTES);
        if (payload && result.payload_length > 0) {
            std::memcpy(result.payload.data(), payload, result.payload_length);
        }
        return result;
    }

    uint16_t last_address_{0};
    uint16_t last_command_{0};
};
} // namespace

IrProtocolDecoderPtr create_nec_decoder() {
    return std::make_unique<NecDecoder>();
}

/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#include "rc5_decoder.h"

#include <cstdint>
#include <memory>

#include "esp_log.h"

namespace {
static const char *TAG = "rc5_decoder";
static constexpr uint32_t RC5_HALF_BIT_US = 889; // microseconds per half bit
static constexpr uint32_t RC5_TOLERANCE_US = 450; // Relaxed for 36kHz/38kHz tolerance
static constexpr size_t RC5_TOTAL_BITS = 14;

class Rc5Decoder final : public IrProtocolDecoder {
public:
    ir_format_t format() const override { return ir_format_t::RC5; }

    bool decode(const rmt_symbol_word_t *symbols,
                size_t symbol_num,
                ir_decode_result_t &result) override {
        if (!symbols || symbol_num == 0) {
            return false;
        }

        ESP_LOGD(TAG, "RC5 decode: symbol_num=%zu", symbol_num);

        // 标准RC5：每个symbol = 一个Manchester编码的位
        // 期望14个符号（每位889us+889us）
        ESP_LOGI(TAG, "RC5 standard mode: symbol_num=%zu", symbol_num);
        return decode_demodulated(symbols, symbol_num, result);
    }

private:
    // 基带模式（无载波解调）：使用参考代码esp32-rmt-ir的rc5_check算法
    bool decode_demodulated(const rmt_symbol_word_t *symbols,
                           size_t symbol_num,
                           ir_decode_result_t &result) {
        if (symbol_num < 7 || symbol_num > 30) {
            ESP_LOGW(TAG, "Invalid symbol_num=%zu (expect 7-30)", symbol_num);
            return false;
        }
        
        const uint32_t RC5_FULL = RC5_HALF_BIT_US * 2;
        uint32_t code = 0;
        bool c = false;
        
        for (size_t i = 0; i < symbol_num; ++i) {
            uint32_t d0 = symbols[i].duration0;
            uint32_t d1 = symbols[i].duration1;
            
            ESP_LOGI(TAG, "  [%zu] l0=%u l1=%u d0=%u d1=%u",
                     i,
                     static_cast<unsigned>(symbols[i].level0),
                     static_cast<unsigned>(symbols[i].level1),
                     static_cast<unsigned>(d0),
                     static_cast<unsigned>(d1));
            
            if (rc5_bit_match(d0, RC5_HALF_BIT_US)) {
                code = (code << 1) | (c ? 1 : 0);
                c = rc5_bit_match(d1, RC5_FULL) ? !c : c;
                ESP_LOGI(TAG, "    half: bit=%u -> 0x%X",
                         static_cast<unsigned>(c ? 1 : 0),
                         static_cast<unsigned>(code));
            } else if (rc5_bit_match(d0, RC5_FULL)) {
                uint32_t two_bits = (symbols[i].level0 << 1) | (!symbols[i].level0 ? 1 : 0);
                code = (code << 2) | two_bits;
                c = rc5_bit_match(d1, RC5_HALF_BIT_US) ? !c : c;
                ESP_LOGI(TAG, "    full: l0=%u bits=0b%u%u -> 0x%X",
                         static_cast<unsigned>(symbols[i].level0),
                         static_cast<unsigned>((two_bits >> 1) & 1),
                         static_cast<unsigned>(two_bits & 1),
                         static_cast<unsigned>(code));
            } else {
                ESP_LOGW(TAG, "    invalid d0=%u", static_cast<unsigned>(d0));
                return false;
            }
        }
        
        // 参考代码的rc5_check直接返回code，不验证位数
        // 位数验证通过起始位检查来完成
        ESP_LOGI(TAG, "RC5 decoded: code=0x%X", static_cast<unsigned>(code));
        
        uint16_t frame_bits = static_cast<uint16_t>(code);

        if (!validate_start_bits(frame_bits)) {
            ESP_LOGW(TAG, "Start bits validation failed: 0x%04X", static_cast<unsigned>(frame_bits));
            char bin_str[17] = {0};
            for (int b = 13; b >= 0; --b) {
                bin_str[13 - b] = (frame_bits & (1 << b)) ? '1' : '0';
            }
            ESP_LOGW(TAG, "  Binary: %s (expect 11xxxx xxxxxxxx)", bin_str);
            return false;
        }

        ESP_LOGI(TAG, "RC5 decoded successfully: 0x%04X", static_cast<unsigned>(frame_bits));
        return fill_result(frame_bits, result);
    }

    // 基带模式：处理载波解调后的Manchester符号（参考esp32-rmt-ir的rc5_check实现）
    bool decode_baseband(const rmt_symbol_word_t *symbols,
                        size_t symbol_num,
                        ir_decode_result_t &result) {
        ESP_LOGD(TAG, "RC5 baseband decode: symbol_num=%zu", symbol_num);
        
        // 打印前3个符号用于调试
        if (symbol_num > 0) {
            ESP_LOGD(TAG, "First symbols: [0]d0=%u d1=%u l0=%u l1=%u",
                     static_cast<unsigned>(symbols[0].duration0),
                     static_cast<unsigned>(symbols[0].duration1),
                     static_cast<unsigned>(symbols[0].level0),
                     static_cast<unsigned>(symbols[0].level1));
        }
        if (symbol_num > 1) {
            ESP_LOGD(TAG, "               [1]d0=%u d1=%u l0=%u l1=%u",
                     static_cast<unsigned>(symbols[1].duration0),
                     static_cast<unsigned>(symbols[1].duration1),
                     static_cast<unsigned>(symbols[1].level0),
                     static_cast<unsigned>(symbols[1].level1));
        }
        
        // RC5需要至少7个符号（最紧凑的情况：7个全位周期=14位）
        // 最松散情况：14个符号（每位都是半位+半位）+ 一些容错空间
        if (symbol_num < 7 || symbol_num > 30) {
            ESP_LOGW(TAG, "RC5 baseband: invalid symbol_num=%zu (expect 7~30)", symbol_num);
            return false;
        }

        // 完全参考esp32-rmt-ir的rc5_check算法，加上电平反转处理
        // 红外接收器低电平有效：有载波→输出低，无载波→输出高
        // 所以接收到的电平与发射端相反
        const uint32_t RC5_FULL_BIT_US = RC5_HALF_BIT_US * 2; // 1778us
        uint32_t frame_bits = 0;
        bool c = false;  // 状态位：参考代码中的"当前位值"
        size_t bit_count = 0;
        
        for (size_t i = 0; i < symbol_num && bit_count < RC5_TOTAL_BITS; ++i) {
            const auto &sym = symbols[i];
            uint32_t d0 = sym.duration0;
            uint32_t d1 = sym.duration1;
            
            ESP_LOGI(TAG, "  [%zu] d0=%u d1=%u l0=%u l1=%u (bits=%zu c=%u)",
                     i,
                     static_cast<unsigned>(d0),
                     static_cast<unsigned>(d1),
                     static_cast<unsigned>(sym.level0),
                     static_cast<unsigned>(sym.level1),
                     bit_count,
                     static_cast<unsigned>(c ? 1 : 0));
            
            if (rc5_bit_match(d0, RC5_HALF_BIT_US)) {
                // d0是半位：输出状态c（参考代码的核心逻辑）
                uint32_t old_bits = frame_bits;
                frame_bits = (frame_bits << 1) | (c ? 1U : 0U);
                bit_count++;
                ESP_LOGI(TAG, "    half: output c=%u, bits=0x%X->0x%X",
                         static_cast<unsigned>(c ? 1 : 0),
                         static_cast<unsigned>(old_bits),
                         static_cast<unsigned>(frame_bits));
                
                // 根据d1更新c（参考代码：c = rc5_bit(d1, RC5_High) ? !c : c）
                if (d1 == 0) {
                    // 结束
                } else if (rc5_bit_match(d1, RC5_FULL_BIT_US)) {
                    c = !c;  // d1是全位，状态翻转
                    ESP_LOGI(TAG, "      d1=full, flip c->%u", static_cast<unsigned>(c ? 1 : 0));
                } else if (rc5_bit_match(d1, RC5_HALF_BIT_US)) {
                    // d1是半位，c不变
                    ESP_LOGI(TAG, "      d1=half, keep c=%u", static_cast<unsigned>(c ? 1 : 0));
                } else {
                    ESP_LOGW(TAG, "RC5 d1=%u invalid", static_cast<unsigned>(d1));
                    return false;
                }
            } else if (rc5_bit_match(d0, RC5_FULL_BIT_US)) {
                // d0是全位：参考代码 (item[i].level0 << 1) | !item[i].level0
                // 注意：接收到的level已经是反转的，我们需要再反转回来理解
                // 但参考代码直接用level0，说明参考代码的level0就是原始level
                // 所以我们需要反转：接收level0 → 原始level = !level0
                uint8_t original_level0 = sym.level0 ? 0 : 1;  // 反转回发射端的电平
                uint32_t old_bits = frame_bits;
                uint32_t two_bits = ((original_level0 << 1) | !original_level0);
                frame_bits = (frame_bits << 2) | two_bits;
                bit_count += 2;
                ESP_LOGI(TAG, "    full: rx_l0=%u->orig_l0=%u, output 0b%u%u, bits=0x%X->0x%X",
                         static_cast<unsigned>(sym.level0),
                         static_cast<unsigned>(original_level0),
                         static_cast<unsigned>((two_bits >> 1) & 1),
                         static_cast<unsigned>(two_bits & 1),
                         static_cast<unsigned>(old_bits),
                         static_cast<unsigned>(frame_bits));
                
                // 根据d1更新c（参考代码：c = rc5_bit(d1, proto[RC5].one_low) ? !c : c）
                if (d1 == 0) {
                    // 结束
                } else if (rc5_bit_match(d1, RC5_HALF_BIT_US)) {
                    c = !c;  // d1是半位，状态翻转
                    ESP_LOGI(TAG, "      d1=half, flip c->%u", static_cast<unsigned>(c ? 1 : 0));
                } else if (rc5_bit_match(d1, RC5_FULL_BIT_US)) {
                    // d1是全位，c不变（注意：这里与半位情况相反）
                    ESP_LOGI(TAG, "      d1=full, keep c=%u", static_cast<unsigned>(c ? 1 : 0));
                } else {
                    ESP_LOGW(TAG, "RC5 d1=%u invalid", static_cast<unsigned>(d1));
                    return false;
                }
            } else {
                ESP_LOGW(TAG, "RC5 d0=%u invalid", static_cast<unsigned>(d0));
                return false;
            }
        }
        
        // 检查是否解码了正确的位数
        if (bit_count != RC5_TOTAL_BITS) {
            ESP_LOGW(TAG, "RC5 baseband: decoded %zu bits (expect %zu)", bit_count, RC5_TOTAL_BITS);
            return false;
        }

        ESP_LOGI(TAG, "RC5 baseband decoded: frame_bits=0x%04X (bit_count=%zu)",
                 static_cast<unsigned>(frame_bits), bit_count);

        if (!validate_start_bits(frame_bits)) {
            ESP_LOGW(TAG, "RC5 start bits validation failed: 0x%04X (expect MSB 2 bits = 11)",
                     static_cast<unsigned>(frame_bits));
            // 打印二进制表示用于调试
            char bin_str[17] = {0};
            for (int b = 13; b >= 0; --b) {
                bin_str[13 - b] = (frame_bits & (1 << b)) ? '1' : '0';
            }
            ESP_LOGW(TAG, "  Binary: %s", bin_str);
            return false;
        }

        return fill_result(frame_bits, result);
    }
    
    // 检查duration是否匹配期望值（带容差）
    bool rc5_bit_match(uint32_t duration, uint32_t expected) const {
        return duration > (expected - RC5_TOLERANCE_US) &&
               duration < (expected + RC5_TOLERANCE_US);
    }

    bool decode_symbol_demodulated(const rmt_symbol_word_t &symbol, bool &bit_value) const {
        if (!rc5_bit_match(symbol.duration0, RC5_HALF_BIT_US) || 
            !rc5_bit_match(symbol.duration1, RC5_HALF_BIT_US)) {
            return false;
        }
        if (symbol.level0 == 1 && symbol.level1 == 0) {
            bit_value = true;
            return true;
        }
        if (symbol.level0 == 0 && symbol.level1 == 1) {
            bit_value = false;
            return true;
        }
        return false;
    }

    bool validate_start_bits(uint16_t frame_bits) const {
        // RC5是14位协议，最高2位（bit13和bit12）必须是11
        return ((frame_bits >> 12) & 0x3) == 0x3;
    }

    bool fill_result(uint16_t frame_bits, ir_decode_result_t &result) {
        const uint16_t toggle = (frame_bits >> 11) & 0x1;
        const uint16_t address = (frame_bits >> 6) & 0x1F;
        const uint16_t command = frame_bits & 0x3F;

        bool is_repeat = last_valid_ && (toggle == last_toggle_) &&
                         (address == last_address_) && (command == last_command_);

        last_toggle_ = toggle;
        last_address_ = address;
        last_command_ = command;
        last_valid_ = true;

        result.format = ir_format_t::RC5;
        result.is_repeat = is_repeat;
        result.address = address;
        result.command = command;
        result.raw_value = frame_bits;
        result.payload_length = 0;
        return true;
    }

    uint16_t last_toggle_{0};
    uint16_t last_address_{0};
    uint16_t last_command_{0};
    bool last_valid_{false};
};
} // namespace

IrProtocolDecoderPtr create_rc5_decoder() {
    return std::make_unique<Rc5Decoder>();
}

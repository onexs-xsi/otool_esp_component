/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#include "rc5_encoder.h"

#include <cstdint>

#include "esp_check.h"
#include "esp_log.h"

namespace rc5 {
namespace {
constexpr uint32_t RC5_HALF_BIT_US = 889;
constexpr uint32_t RC5_TOTAL_BITS = 14;
}

esp_err_t build_frame_symbols(const ir_send_request_t &request,
                              uint32_t resolution_hz,
                              std::array<rmt_symbol_word_t, RC5_MAX_SYMBOLS> &symbols,
                              size_t &symbol_count) {
    ESP_RETURN_ON_FALSE(resolution_hz > 0, ESP_ERR_INVALID_ARG, "rc5_encoder", "resolution must be > 0");
    const uint32_t half_ticks = (RC5_HALF_BIT_US * resolution_hz) / 1000000U;
    ESP_RETURN_ON_FALSE(half_ticks > 0, ESP_ERR_INVALID_STATE, "rc5_encoder", "half_ticks is zero");

    const uint16_t toggle_bit = request.is_repeat ? 0U : 1U;
    const uint16_t address = static_cast<uint16_t>(request.address & 0x1FU);
    const uint16_t command = static_cast<uint16_t>(request.command & 0x3FU);

    uint16_t frame = 0;
    frame |= (0x3 << 12);            // start bits (S1/S2)
    frame |= (toggle_bit & 0x1) << 11;
    frame |= (address & 0x1F) << 6;
    frame |= (command & 0x3F);

    // 标准RC5 Manchester编码：每个位发送一个独立的symbol
    // 逻辑1: 先高后低 (1→0)
    // 逻辑0: 先低后高 (0→1)
    // 每个位持续时间：1.778ms (889us × 2)
    
    symbol_count = RC5_TOTAL_BITS;
    for (size_t bit_idx = 0; bit_idx < RC5_TOTAL_BITS; ++bit_idx) {
        size_t shift = RC5_TOTAL_BITS - bit_idx - 1;
        bool bit_value = ((frame >> shift) & 0x1) != 0;
        
        rmt_symbol_word_t symbol{};
        symbol.duration0 = half_ticks;
        symbol.duration1 = half_ticks;
        
        // Manchester编码（Philips RC5标准）：
        // 发射端有invert_out=true，会自动反转输出
        // RMT: level=1时输出载波，level=0时停止载波
        // 逻辑1: 先发载波(1)后停止(0) -> level0=1, level1=0
        // 逻辑0: 先停止(0)后发载波(1) -> level0=0, level1=1
        if (bit_value) {
            symbol.level0 = 1;
            symbol.level1 = 0;
        } else {
            symbol.level0 = 0;
            symbol.level1 = 1;
        }
        
        symbols[bit_idx] = symbol;
    }
    
    ESP_LOGI("rc5_encoder", "Frame=0x%04X encoded to %zu standard RC5 symbols", frame, symbol_count);
    
    return ESP_OK;
}

} // namespace rc5

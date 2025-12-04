/*
 * SPDX-FileCopyrightText: 2025 exia
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <array>
#include <cstddef>

#include "driver/rmt_types.h"
#include "esp_err.h"
#include "ir_codec_types.h"

namespace rc5 {
constexpr size_t RC5_MAX_SYMBOLS = 14;

esp_err_t build_frame_symbols(const ir_send_request_t &request,
                              uint32_t resolution_hz,
                              std::array<rmt_symbol_word_t, RC5_MAX_SYMBOLS> &symbols,
                              size_t &symbol_count);
}

/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

// Deprecated legacy AEC implementation
// This file intentionally left as an empty translation unit.
// All AEC features have been moved to audio_sr_afe.{h,cpp} and are accessed via:
//   audio_sr_afe* afe = audio.get_sr_afe();
//   afe->aec_init(...); afe->aec_test_loopback(...); afe->aec_test(...);

// Keeping this file to avoid altering CMakeLists. No symbols are emitted here.
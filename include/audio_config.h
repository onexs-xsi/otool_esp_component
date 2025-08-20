/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H

// 这个文件定义了音频相关的配置
// 宏定义由CMake自动控制

// PCM测试文件配置
#ifdef USE_PCM_TEST_A
    #define PCM_TEST_A_ENABLED 1
    // 二进制数据声明
    extern "C" const uint8_t _binary_test_a_pcm_start[] asm("_binary_test_a_pcm_start");
    extern "C" const uint8_t _binary_test_a_pcm_end[] asm("_binary_test_a_pcm_end");
#else
    #define PCM_TEST_A_ENABLED 0
#endif

#ifdef USE_PCM_TEST_B
    #define PCM_TEST_B_ENABLED 1
    // 二进制数据声明
    extern "C" const uint8_t _binary_test_b_pcm_start[] asm("_binary_test_b_pcm_start");
    extern "C" const uint8_t _binary_test_b_pcm_end[] asm("_binary_test_b_pcm_end");
#else
    #define PCM_TEST_B_ENABLED 0
#endif

// 辅助宏，用于运行时检查
#define IS_PCM_TEST_A_AVAILABLE() (PCM_TEST_A_ENABLED)
#define IS_PCM_TEST_B_AVAILABLE() (PCM_TEST_B_ENABLED)

#endif // AUDIO_CONFIG_H

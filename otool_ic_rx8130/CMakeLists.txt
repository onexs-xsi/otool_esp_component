# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_ic_rx8130 — RTC 子组件（Epson RX8130）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_RTC_RX8130（option，由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / COMPILE_DEFS
# ============================================================================

if(NOT ENABLE_OTOOL_RTC_RX8130)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_RTC_RX8130)

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/rx8130_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

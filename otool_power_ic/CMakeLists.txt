# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_power_ic — 电源 IC 子组件（IP2315）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_POWER_IC（option，由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / COMPILE_DEFS
# ============================================================================

if(NOT ENABLE_OTOOL_POWER_IC)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_POWER_IC)

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/ip2315_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

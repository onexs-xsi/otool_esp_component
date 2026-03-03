# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_sd — SD 卡子组件（MMC 1-bit/4-bit / SPI 模式）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_SD（option，由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / TOOLBOX_REQUIRES / COMPILE_DEFS
# ============================================================================

if(NOT ENABLE_OTOOL_SD)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_SD)

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/sd_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

list(APPEND TOOLBOX_REQUIRES
    fatfs
    sdmmc
)

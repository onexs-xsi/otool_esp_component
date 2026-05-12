# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_ir — 红外子组件（NEC / RC5 编解码）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_IR（option，由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / COMPILE_DEFS
# ============================================================================

if(NOT ENABLE_OTOOL_IR)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_IR)

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/ir_encoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ir_decoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ir_tools.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/format/nec/nec_encoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/format/nec/nec_decoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/format/rc5/rc5_decoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/format/rc5/rc5_encoder.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${CMAKE_CURRENT_LIST_DIR}/format/nec"
    "${CMAKE_CURRENT_LIST_DIR}/format/rc5"
)

list(APPEND TOOLBOX_REQUIRES
    esp_driver_rmt
)

# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_i2c_bus — I2C 总线工具（框架基础，始终启用）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / TOOLBOX_REQUIRES
# ============================================================================

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/i2c_bus_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

list(APPEND TOOLBOX_REQUIRES
    driver
    esp_driver_i2c
    i2c_bus
)

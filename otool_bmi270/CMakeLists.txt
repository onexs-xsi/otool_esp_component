# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_bmi270 — IMU 子组件（BMI270 加速度/陀螺仪 + BMM150 磁力计）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_BMI270（option，由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / TOOLBOX_REQUIRES / COMPILE_DEFS
# ============================================================================

if(NOT ENABLE_OTOOL_BMI270)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_BMI270)

file(GLOB BMI270_API_SOURCES "${CMAKE_CURRENT_LIST_DIR}/BMI270_SensorAPI/*.c")
file(GLOB BMM150_API_SOURCES "${CMAKE_CURRENT_LIST_DIR}/BMM150_SensorAPI/*.c")

list(APPEND TOOLBOX_SRCS
    ${BMI270_API_SOURCES}
    ${BMM150_API_SOURCES}
    "${CMAKE_CURRENT_LIST_DIR}/bmi270_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${CMAKE_CURRENT_LIST_DIR}/BMI270_SensorAPI"
    "${CMAKE_CURRENT_LIST_DIR}/BMM150_SensorAPI"
)

list(APPEND TOOLBOX_REQUIRES
    esp_timer
)

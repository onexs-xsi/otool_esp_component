# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_esp_component — 根 CMakeLists.txt（编排层）
#
# 职责：
#   1. 声明所有子组件 option（便于工具脚本统一读写）
#   2. 声明音频嵌入配置（AUDIO_FILE_CONFIGS，供 toolkit.py 解析）
#   3. include() 各子组件 cmake，让子组件自行追加 SRCS/INCLUDES/REQUIRES
#   4. 调用 idf_component_register() 完成最终注册
#
# 添加新子组件步骤：
#   1. 在对应目录创建 CMakeLists.txt（参照 otool_sd/CMakeLists.txt 为模板）
#   2. 在此文件添加 option() 声明
#   3. 在"包含子组件"区块添加一行 include()
# ============================================================================

# ============================================================================
# 子组件开关（默认均为 OFF，确保拉取后只保留框架）
# ============================================================================
option(ENABLE_OTOOL_AUDIO      "Enable otool_audio subcomponent"      OFF)
option(ENABLE_OTOOL_BMI270     "Enable otool_bmi270 subcomponent"     OFF)
option(ENABLE_OTOOL_IR         "Enable otool_ir subcomponent"         OFF)
option(ENABLE_OTOOL_POWER_IC   "Enable otool_power_ic subcomponent"   OFF)
option(ENABLE_OTOOL_SD         "Enable otool_sd subcomponent"         OFF)
option(ENABLE_OTOOL_RTC_RX8130 "Enable otool_ic_rx8130 subcomponent"  OFF)

# ============================================================================
# 音频文件嵌入配置（供 otool_audio/CMakeLists.txt 使用，由 toolkit.py 解析）
# 格式：ID:文件名:默认开关:采样率:声道:位深:描述
# 添加新音频文件：只需在此列表增加一行
# ============================================================================
set(AUDIO_PLAYBACK_DIR "otool_audio/audio_playback_material")

set(AUDIO_FILE_CONFIGS
    # ID:文件名:默认值:采样率:声道:位深:描述
    "AUDIO_CANDY_WIND_1CH_16K_16BIT_9S:candy_wind_pcm_1ch_16k_16bit_9s.pcm:OFF:16000:1:16:candy_wind 1ch 16000Hz 9.0s"
    "AUDIO_CANDY_WIND_1CH_44K_16BIT_45S:candy_wind_pcm_1ch_44.1k_16bit_45s.pcm:OFF:44100:1:16:candy_wind 1ch 44100Hz 45.0s"
    "AUDIO_CANDY_WIND_2CH_16K_16BIT_9S:candy_wind_pcm_2ch_16k_16bit_9s.pcm:OFF:16000:2:16:candy_wind 2ch 16000Hz 9.0s"
    "AUDIO_CANDY_WIND_2CH_44K_16BIT_45S:candy_wind_pcm_2ch_44.1k_16bit_45s.pcm:OFF:44100:2:16:candy_wind 2ch 44100Hz 45.0s"
    "AUDIO_SINE_440HZ_2CH_16K_16BIT_10S:sine_440Hz_pcm_2ch_16k_16bit_10s.pcm:OFF:16000:2:16:sine_440Hz 2ch 16000Hz 10.0s"
    "AUDIO_STARTUP_1CH_16K_16BIT_4S:startup_pcm_1ch_16k_16bit_4s.pcm:OFF:16000:1:16:startup 1ch 16000Hz 4.0s"
    "AUDIO_STARTUP_2CH_16K_16BIT_4S:startup_pcm_2ch_16k_16bit_4s.pcm:OFF:16000:2:16:startup 2ch 16000Hz 4.0s"
)

# ============================================================================
# 框架基础变量（始终启用，由根组件自身提供）
# otool_toolbox.cpp 是胶水层，聚合各子组件工具对象
# ============================================================================
set(TOOLBOX_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/otool_toolbox.cpp"
)
set(TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set(TOOLBOX_REQUIRES)
set(COMPILE_DEFS)
set(EMBED_FILE_LIST)

# ============================================================================
# 包含子组件配置
# 每个子组件 cmake 负责追加自身的 SRCS/INCLUDE_DIRS/REQUIRES/COMPILE_DEFS
# ============================================================================
include(${CMAKE_CURRENT_LIST_DIR}/otool_i2c_bus/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_audio/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_bmi270/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_ir/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_power_ic/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_sd/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/otool_ic_rx8130/CMakeLists.txt)

# ============================================================================
# 去重（子组件可能重复追加同名依赖）
# ============================================================================
list(REMOVE_DUPLICATES TOOLBOX_SRCS)
list(REMOVE_DUPLICATES TOOLBOX_INCLUDE_DIRS)
list(REMOVE_DUPLICATES TOOLBOX_REQUIRES)
list(REMOVE_DUPLICATES COMPILE_DEFS)

# ============================================================================
# 注册 IDF 组件
# ============================================================================
if(EMBED_FILE_LIST)
    idf_component_register(
        SRCS             ${TOOLBOX_SRCS}
        INCLUDE_DIRS     ${TOOLBOX_INCLUDE_DIRS}
        REQUIRES         ${TOOLBOX_REQUIRES}
        EMBED_FILES      ${EMBED_FILE_LIST}
    )
else()
    idf_component_register(
        SRCS             ${TOOLBOX_SRCS}
        INCLUDE_DIRS     ${TOOLBOX_INCLUDE_DIRS}
        REQUIRES         ${TOOLBOX_REQUIRES}
    )
endif()

# 使用 PUBLIC 传播编译定义，使上层组件包含 otool_toolbox.h 时
# 能正确看到 OTOOL_ENABLE_* 宏展开条件头文件
if(COMPILE_DEFS)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC ${COMPILE_DEFS})
endif()
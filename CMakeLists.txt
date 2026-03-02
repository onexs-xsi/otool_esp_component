# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# 子组件开关（默认均为 OFF，确保拉取后只保留框架）
# ============================================================================
option(ENABLE_OTOOL_AUDIO "Enable otool_audio subcomponent" OFF)
option(ENABLE_OTOOL_BMI270 "Enable otool_bmi270 subcomponent" OFF)
option(ENABLE_OTOOL_IR "Enable otool_ir subcomponent" OFF)
option(ENABLE_OTOOL_POWER_IC "Enable otool_power_ic subcomponent" OFF)
option(ENABLE_OTOOL_SD "Enable otool_sd subcomponent" OFF)
option(ENABLE_OTOOL_RTC_RX8130 "Enable otool_ic_rx8130 subcomponent" OFF)

# ============================================================================
# 音频文件嵌入配置 - 统一命名规范
# ============================================================================
# 配置格式：ID:文件名:默认开关:采样率:声道:位深:描述
# ID规则：自动从文件名生成，统一使用 AUDIO_ 前缀
# 添加新文件：只需在下面列表中添加一行即可
# ============================================================================
set(AUDIO_PLAYBACK_DIR "otool_audio/audio_playback_material")

set(AUDIO_FILE_CONFIGS
    # ID:文件名:默认值:采样率:声道:位深:描述
    "AUDIO_CANDY_WIND_1CH_16K_16BIT_9S:candy_wind_pcm_1ch_16k_16bit_9s.pcm:OFF:16000:1:16:candy_wind 1ch 16000Hz 9.0s"    "AUDIO_CANDY_WIND_1CH_44K_16BIT_45S:candy_wind_pcm_1ch_44.1k_16bit_45s.pcm:OFF:44100:1:16:candy_wind 1ch 44100Hz 45.0s"    "AUDIO_CANDY_WIND_2CH_16K_16BIT_9S:candy_wind_pcm_2ch_16k_16bit_9s.pcm:OFF:16000:2:16:candy_wind 2ch 16000Hz 9.0s"    "AUDIO_CANDY_WIND_2CH_44K_16BIT_45S:candy_wind_pcm_2ch_44.1k_16bit_45s.pcm:OFF:44100:2:16:candy_wind 2ch 44100Hz 45.0s"    "AUDIO_SINE_440HZ_2CH_16K_16BIT_10S:sine_440Hz_pcm_2ch_16k_16bit_10s.pcm:OFF:16000:2:16:sine_440Hz 2ch 16000Hz 10.0s"    "AUDIO_STARTUP_1CH_16K_16BIT_4S:startup_pcm_1ch_16k_16bit_4s.pcm:OFF:16000:1:16:startup 1ch 16000Hz 4.0s"    "AUDIO_STARTUP_2CH_16K_16BIT_4S:startup_pcm_2ch_16k_16bit_4s.pcm:OFF:16000:2:16:startup 2ch 16000Hz 4.0s"
)

# 动态构建EMBED_FILES列表和编译定义
set(EMBED_FILE_LIST)
set(COMPILE_DEFS)

# 框架基础（始终启用）
set(TOOLBOX_SRCS
    "otool_toolbox.cpp"
    "otool_i2c_bus/i2c_bus_tools.cpp"
)
set(TOOLBOX_INCLUDE_DIRS
    "include"
    "otool_i2c_bus/include"
)
set(TOOLBOX_REQUIRES
    driver
    esp_driver_i2c
    i2c_bus
)

# otool_audio 子组件（可选）
if(ENABLE_OTOOL_AUDIO)
    list(APPEND COMPILE_DEFS OTOOL_ENABLE_AUDIO)
    list(APPEND TOOLBOX_SRCS
        "otool_audio/audio_tools.cpp"
        "otool_audio/audio_chip_device/audio_es_es8311/audio_es_es8311.cpp"
        "otool_audio/audio_chip_device/audio_es_es7210/audio_es_es7210.cpp"
        "otool_audio/audio_sr_afe.cpp"
        "otool_audio/audio_remix_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_audio/include"
    )
    list(APPEND TOOLBOX_REQUIRES
        esp_codec_dev
        esp-sr
        esp-dsp
    )

    foreach(config ${AUDIO_FILE_CONFIGS})
        string(REPLACE ":" ";" config_list ${config})
        list(GET config_list 0 audio_id)
        list(GET config_list 1 filename)
        list(GET config_list 2 default_val)
        list(GET config_list 6 description)

        # 创建 CMake option (统一命名：ENABLE_${audio_id})
        option(ENABLE_${audio_id} "Enable embedding ${description}" ${default_val})

        # 如果启用，添加到嵌入列表和编译定义
        if(ENABLE_${audio_id})
            list(APPEND EMBED_FILE_LIST "${AUDIO_PLAYBACK_DIR}/${filename}")
            list(APPEND COMPILE_DEFS "USE_${audio_id}")
        endif()
    endforeach()
endif()

# otool_bmi270 子组件（可选）
if(ENABLE_OTOOL_BMI270)
    file(GLOB BMI270_API_SOURCES "otool_bmi270/BMI270_SensorAPI/*.c")
    file(GLOB BMM150_API_SOURCES "otool_bmi270/BMM150_SensorAPI/*.c")

    list(APPEND COMPILE_DEFS OTOOL_ENABLE_BMI270)
    list(APPEND TOOLBOX_SRCS
        ${BMI270_API_SOURCES}
        ${BMM150_API_SOURCES}
        "otool_bmi270/bmi270_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_bmi270/include"
        "otool_bmi270/BMI270_SensorAPI"
        "otool_bmi270/BMM150_SensorAPI"
    )
    list(APPEND TOOLBOX_REQUIRES
        esp_timer
    )
endif()

# otool_ir 子组件（可选）
if(ENABLE_OTOOL_IR)
    list(APPEND COMPILE_DEFS OTOOL_ENABLE_IR)
    list(APPEND TOOLBOX_SRCS
        "otool_ir/ir_encoder.cpp"
        "otool_ir/ir_decoder.cpp"
        "otool_ir/format/nec/nec_encoder.cpp"
        "otool_ir/format/nec/nec_decoder.cpp"
        "otool_ir/format/rc5/rc5_decoder.cpp"
        "otool_ir/format/rc5/rc5_encoder.cpp"
        "otool_ir/ir_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_ir/include"
        "otool_ir/format/nec"
        "otool_ir/format/rc5"
    )
endif()

# otool_power_ic 子组件（可选）
if(ENABLE_OTOOL_POWER_IC)
    list(APPEND COMPILE_DEFS OTOOL_ENABLE_POWER_IC)
    list(APPEND TOOLBOX_SRCS
        "otool_power_ic/ip2315_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_power_ic/include"
    )
endif()

# otool_sd 子组件（可选）
if(ENABLE_OTOOL_SD)
    list(APPEND COMPILE_DEFS OTOOL_ENABLE_SD)
    list(APPEND TOOLBOX_SRCS
        "otool_sd/sd_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_sd/include"
    )
    list(APPEND TOOLBOX_REQUIRES
        fatfs
        sdmmc
    )
endif()

# otool_ic_rx8130 子组件（可选）
if(ENABLE_OTOOL_RTC_RX8130)
    list(APPEND COMPILE_DEFS OTOOL_ENABLE_RTC_RX8130)
    list(APPEND TOOLBOX_SRCS
        "otool_ic_rx8130/rx8130_tools.cpp"
    )
    list(APPEND TOOLBOX_INCLUDE_DIRS
        "otool_ic_rx8130/include"
    )
endif()

list(REMOVE_DUPLICATES TOOLBOX_INCLUDE_DIRS)
list(REMOVE_DUPLICATES TOOLBOX_REQUIRES)
list(REMOVE_DUPLICATES COMPILE_DEFS)

if(EMBED_FILE_LIST)
    idf_component_register(
        SRCS ${TOOLBOX_SRCS}
        INCLUDE_DIRS ${TOOLBOX_INCLUDE_DIRS}
        REQUIRES ${TOOLBOX_REQUIRES}
        EMBED_FILES ${EMBED_FILE_LIST}
    )
else()
    idf_component_register(
        SRCS ${TOOLBOX_SRCS}
        INCLUDE_DIRS ${TOOLBOX_INCLUDE_DIRS}
        REQUIRES ${TOOLBOX_REQUIRES}
    )
endif()

# 添加编译定义（在组件注册后）
if(COMPILE_DEFS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${COMPILE_DEFS})
endif()

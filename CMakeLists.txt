# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

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
    "AUDIO_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S:candy_wind_pcm_1ch_16k_16bit_9s.pcm:OFF:16000:1:16:candy_wind 1ch 16000Hz 9.0s"
    "AUDIO_AUDIO_CANDY_WIND_1CH_44K_16BIT_45S:candy_wind_pcm_1ch_44.1k_16bit_45s.pcm:OFF:44100:1:16:candy_wind 1ch 44100Hz 45.0s"
    "AUDIO_AUDIO_CANDY_WIND_2CH_16K_16BIT_9S:candy_wind_pcm_2ch_16k_16bit_9s.pcm:ON:16000:2:16:candy_wind 2ch 16000Hz 9.0s"
    "AUDIO_AUDIO_CANDY_WIND_2CH_44K_16BIT_45S:candy_wind_pcm_2ch_44.1k_16bit_45s.pcm:OFF:44100:2:16:candy_wind 2ch 44100Hz 45.0s"
    "AUDIO_AUDIO_SINE_440HZ_2CH_16K_16BIT_10S:sine_440Hz_pcm_2ch_16k_16bit_10s.pcm:OFF:16000:2:16:sine_440Hz 2ch 16000Hz 10.0s"
    "AUDIO_AUDIO_STARTUP_1CH_16K_16BIT_4S:startup_pcm_1ch_16k_16bit_4s.pcm:OFF:16000:1:16:startup 1ch 16000Hz 4.0s"
    "AUDIO_AUDIO_STARTUP_2CH_16K_16BIT_4S:startup_pcm_2ch_16k_16bit_4s.pcm:OFF:16000:2:16:startup 2ch 16000Hz 4.0s"
)
# 动态构建EMBED_FILES列表和编译定义
set(EMBED_FILE_LIST "")
set(COMPILE_DEFS "")

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

# 动态添加BMI270和BMM150传感器API源文件
file(GLOB BMI270_API_SOURCES "otool_bmi270/BMI270_SensorAPI/*.c")
file(GLOB BMM150_API_SOURCES "otool_bmi270/BMM150_SensorAPI/*.c")
set(BMI_API_SOURCES ${BMI270_API_SOURCES} ${BMM150_API_SOURCES})
set(BMI270_SRCS ${BMI_API_SOURCES} "otool_bmi270/bmi270_tools.cpp")

# 注册组件
set(TOOLBOX_SRCS 
    "otool_audio/audio_tools.cpp"
    "otool_audio/audio_chip_device/audio_es_es8311/audio_es_es8311.cpp"
    "otool_audio/audio_chip_device/audio_es_es7210/audio_es_es7210.cpp"
    "otool_audio/audio_esp_sr_afe.cpp"
    "otool_audio/audio_remix_tools.cpp"
    "otool_i2c_bus/i2c_bus_tools.cpp"
    "otool_ir_nec/ir_nec_encoder.c"
    "otool_ir_nec/ir_nec_tools.cpp"
    "otool_power_ic/ip2315_tools.cpp"
    "otool_sd/sd_tools.cpp"
    "otool_ic_rx8130/rx8130_tools.cpp"
    "otool_toolbox.cpp"
    ${BMI270_SRCS}
)

if(EMBED_FILE_LIST)
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include" 
                                     "otool_audio/include" 
                                     "otool_bmi270/include"
                                     "otool_bmi270/BMI270_SensorAPI" 
                                     "otool_bmi270/BMM150_SensorAPI"
                                     "otool_i2c_bus/include"
                                     "otool_ic_rx8130/include"
                                     "otool_ir_nec/include"
                                     "otool_power_ic/include"
                                     "otool_sd/include"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev fatfs sdmmc esp_timer esp-sr esp-dsp
                        EMBED_FILES ${EMBED_FILE_LIST})
else()
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include" 
                                     "otool_audio/include" 
                                     "otool_bmi270/include"
                                     "otool_bmi270/BMI270_SensorAPI" 
                                     "otool_bmi270/BMM150_SensorAPI"
                                     "otool_i2c_bus/include"
                                     "otool_ic_rx8130/include"
                                     "otool_ir_nec/include"
                                     "otool_power_ic/include"
                                     "otool_sd/include"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev fatfs sdmmc esp_timer esp-sr esp-dsp)
endif()

# 添加编译定义（在组件注册后）
if(COMPILE_DEFS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${COMPILE_DEFS})
endif()
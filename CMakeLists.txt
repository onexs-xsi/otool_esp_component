# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# 定义PCM文件控制选项
option(ENABLE_PCM_TEST_A "Enable embedding test_a.pcm file" OFF)
option(ENABLE_PCM_TEST_B "Enable embedding test_b.pcm file" OFF)
option(ENABLE_PCM_STARTUP_1CH "Enable embedding startup_1ch.pcm file" ON)
option(ENABLE_PCM_SINE_440HZ "Enable embedding sine_440Hz_30s_44100Hz_16bit_1ch.pcm file" OFF)

# 动态构建EMBED_FILES列表和编译定义
set(EMBED_FILE_LIST "")
set(COMPILE_DEFS "")

if(ENABLE_PCM_TEST_A)
    list(APPEND EMBED_FILE_LIST "otool_audio/test_a.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_TEST_A")
endif()

if(ENABLE_PCM_TEST_B)
    list(APPEND EMBED_FILE_LIST "otool_audio/test_b.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_TEST_B")
endif()

if(ENABLE_PCM_SINE_440HZ)
    list(APPEND EMBED_FILE_LIST "otool_audio/sine_440Hz_30s_44100Hz_16bit_1ch.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_SINE_440HZ")
endif()

if(ENABLE_PCM_STARTUP_1CH)
    list(APPEND EMBED_FILE_LIST "otool_audio/startup_1ch.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_STARTUP_1CH")
endif()

# 动态添加BMI270和BMM150传感器API源文件
file(GLOB BMI270_API_SOURCES "otool_bmi270/BMI270_SensorAPI/*.c")
file(GLOB BMM150_API_SOURCES "otool_bmi270/BMM150_SensorAPI/*.c")
set(BMI_API_SOURCES ${BMI270_API_SOURCES} ${BMM150_API_SOURCES})
set(BMI270_SRCS ${BMI_API_SOURCES} "otool_bmi270/bmi270_tools.cpp")

# 注册组件
set(TOOLBOX_SRCS 
    "otool_audio/audio_es_tools.cpp"
    "otool_audio/audio_es_es8311.cpp"
    "otool_audio/audio_es_es7210.cpp"
    "otool_i2c_bus/i2c_bus_tools.cpp"
    "otool_ir_nec/ir_nec_encoder.c"
    "otool_ir_nec/ir_nec_tools.cpp"
    "otool_power_ic/power_ic_ip2315.cpp"
    "otool_sd/esp_sd_tool.cpp"
    "otool_ic_rx8130/ic_rx8130.cpp"
    "otool_toolbox.cpp"
    ${BMI270_SRCS}
)

if(EMBED_FILE_LIST)
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include" "otool_bmi270/BMI270_SensorAPI" "otool_bmi270/BMM150_SensorAPI"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev fatfs sdmmc  esp_timer
                        EMBED_FILES ${EMBED_FILE_LIST})
else()
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include" "otool_bmi270/BMI270_SensorAPI" "otool_bmi270/BMM150_SensorAPI"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev fatfs sdmmc  esp_timer)
endif()

# 添加编译定义（在组件注册后）
if(COMPILE_DEFS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${COMPILE_DEFS})
endif()
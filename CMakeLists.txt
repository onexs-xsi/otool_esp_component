# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# 定义PCM文件控制选项
option(ENABLE_PCM_TEST_A "Enable embedding test_a.pcm file" ON)
option(ENABLE_PCM_TEST_B "Enable embedding test_b.pcm file" OFF)
option(ENABLE_PCM_SINE_440HZ "Enable embedding sine_440Hz_30s_44100Hz_16bit_1ch.pcm file" ON)

# 动态构建EMBED_FILES列表和编译定义
set(EMBED_FILE_LIST "")
set(COMPILE_DEFS "")

if(ENABLE_PCM_TEST_A)
    list(APPEND EMBED_FILE_LIST "otools_audio/test_a.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_TEST_A")
endif()

if(ENABLE_PCM_TEST_B)
    list(APPEND EMBED_FILE_LIST "otools_audio/test_b.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_TEST_B")
endif()

if(ENABLE_PCM_SINE_440HZ)
    list(APPEND EMBED_FILE_LIST "otools_audio/sine_440Hz_30s_44100Hz_16bit_1ch.pcm")
    list(APPEND COMPILE_DEFS "USE_PCM_SINE_440HZ")
endif()

# 注册组件
set(TOOLBOX_SRCS 
    "otools_audio/audio_es_tools.cpp"
    "otool_i2c_bus/i2c_bus_tools.cpp"
    "otools_ir_nec/ir_nec_encoder.c"
    "otools_ir_nec/ir_nec_tools.cpp"
    "otool_power_ic/power_ic_ip2315.cpp"
    "otool_toolbox.cpp"
)

if(EMBED_FILE_LIST)
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev
                        EMBED_FILES ${EMBED_FILE_LIST})
else()
    idf_component_register(SRCS ${TOOLBOX_SRCS}
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev)
endif()

# 添加编译定义（在组件注册后）
if(COMPILE_DEFS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${COMPILE_DEFS})
endif()
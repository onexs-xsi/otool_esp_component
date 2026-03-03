# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

# ============================================================================
# otool_audio — 音频子组件（ES8311 DAC / ES7210 ADC / SR / DSP）
#
# 使用方式：由父级 CMakeLists.txt 通过 include() 调用
# 前置变量：ENABLE_OTOOL_AUDIO（option，由根 cmake 定义）
#           AUDIO_FILE_CONFIGS / AUDIO_PLAYBACK_DIR（由根 cmake 定义）
# 供给变量：TOOLBOX_SRCS / TOOLBOX_INCLUDE_DIRS / TOOLBOX_REQUIRES
#           COMPILE_DEFS / EMBED_FILE_LIST
# ============================================================================

if(NOT ENABLE_OTOOL_AUDIO)
    return()
endif()

list(APPEND COMPILE_DEFS OTOOL_ENABLE_AUDIO)

list(APPEND TOOLBOX_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/audio_tools.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_chip_device/audio_es_es8311/audio_es_es8311.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_chip_device/audio_es_es7210/audio_es_es7210.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_sr_afe.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_remix_tools.cpp"
)

list(APPEND TOOLBOX_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

list(APPEND TOOLBOX_REQUIRES
    esp_codec_dev
    esp-sr
    esp-dsp
)

# ── 音频文件嵌入配置 ──────────────────────────────────────────────────────────
# AUDIO_FILE_CONFIGS 格式：ID:文件名:默认开关:采样率:声道:位深:描述
# 由根 CMakeLists.txt 提供，此处处理嵌入逻辑
if(DEFINED AUDIO_FILE_CONFIGS AND DEFINED AUDIO_PLAYBACK_DIR)
    foreach(config ${AUDIO_FILE_CONFIGS})
        string(REPLACE ":" ";" config_list ${config})
        list(GET config_list 0 audio_id)
        list(GET config_list 1 filename)
        list(GET config_list 2 default_val)
        list(GET config_list 6 description)

        # 创建 CMake option（统一命名：ENABLE_${audio_id}）
        option(ENABLE_${audio_id} "Enable embedding ${description}" ${default_val})

        if(ENABLE_${audio_id})
            list(APPEND EMBED_FILE_LIST "${AUDIO_PLAYBACK_DIR}/${filename}")
            list(APPEND COMPILE_DEFS "USE_${audio_id}")
        endif()
    endforeach()
endif()

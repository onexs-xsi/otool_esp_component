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
    "${CMAKE_CURRENT_LIST_DIR}/audio_playback.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/audio_recorder.cpp"
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
    esp_driver_i2s
    esp_ringbuf
    esp-sr
    esp-dsp
)

# ── 音频文件嵌入配置 ──────────────────────────────────────────────────────────
# AUDIO_FILE_CONFIGS 格式：ID:文件名:默认开关:采样率:声道:位深:描述
# 由根 CMakeLists.txt 提供，此处处理嵌入逻辑
# Optional project-level OTOOL_AUDIO_EXTERNAL_FILES entries use:
#   AUDIO_ID|build_source_file|runtime_file_path
# Any enabled ID listed there is loaded from its runtime path. Every other
# enabled ID is embedded with EMBED_FILES. No audio ID is special-cased here.
set(_otool_external_audio_ids)
if(DEFINED OTOOL_AUDIO_EXTERNAL_FILES)
    foreach(external_config IN LISTS OTOOL_AUDIO_EXTERNAL_FILES)
        string(REPLACE "|" ";" external_fields "${external_config}")
        list(LENGTH external_fields external_field_count)
        if(NOT external_field_count EQUAL 3)
            message(FATAL_ERROR
                "Invalid OTOOL_AUDIO_EXTERNAL_FILES entry '${external_config}'. "
                "Expected AUDIO_ID|build_source_file|runtime_file_path")
        endif()

        list(GET external_fields 0 external_audio_id)
        list(GET external_fields 2 external_runtime_path)
        string(STRIP "${external_audio_id}" external_audio_id)
        string(STRIP "${external_runtime_path}" external_runtime_path)

        if(external_audio_id STREQUAL "" OR external_runtime_path STREQUAL "")
            message(FATAL_ERROR
                "External audio ID and runtime path must not be empty: "
                "'${external_config}'")
        endif()
        if(NOT external_runtime_path MATCHES "^/")
            message(FATAL_ERROR
                "External audio runtime path must be absolute: "
                "'${external_runtime_path}'")
        endif()

        set(external_path_var "_otool_external_path_${external_audio_id}")
        if(DEFINED ${external_path_var})
            message(FATAL_ERROR
                "Duplicate external audio configuration for ${external_audio_id}")
        endif()
        set(${external_path_var} "${external_runtime_path}")
        list(APPEND _otool_external_audio_ids "${external_audio_id}")
    endforeach()
endif()

set(_otool_catalog_audio_ids)
if(DEFINED AUDIO_FILE_CONFIGS AND DEFINED AUDIO_PLAYBACK_DIR)
    foreach(config ${AUDIO_FILE_CONFIGS})
        string(REPLACE ":" ";" config_list ${config})
        list(GET config_list 0 audio_id)
        list(GET config_list 1 filename)
        list(GET config_list 2 default_val)
        list(GET config_list 6 description)
        list(APPEND _otool_catalog_audio_ids "${audio_id}")

        # 创建 CMake option（统一命名：ENABLE_${audio_id}）
        set(ENABLE_${audio_id} ${default_val} CACHE BOOL
            "Enable audio file ${description}" FORCE)

        if(ENABLE_${audio_id})
            list(APPEND COMPILE_DEFS "USE_${audio_id}")
            set(external_path_var "_otool_external_path_${audio_id}")
            if(DEFINED ${external_path_var})
                set(external_runtime_path "${${external_path_var}}")
                string(REPLACE "\\" "\\\\" external_runtime_path_c
                               "${external_runtime_path}")
                string(REPLACE "\"" "\\\"" external_runtime_path_c
                               "${external_runtime_path_c}")
                list(APPEND COMPILE_DEFS
                    "USE_EXTERNAL_${audio_id}"
                    "OTOOL_AUDIO_FILE_PATH_${audio_id}=\"${external_runtime_path_c}\"")
            else()
                list(APPEND COMPILE_DEFS "USE_EMBEDDED_${audio_id}")
                list(APPEND EMBED_FILE_LIST
                    "${AUDIO_PLAYBACK_DIR}/${filename}")
            endif()
        endif()
    endforeach()
endif()

foreach(external_audio_id IN LISTS _otool_external_audio_ids)
    list(FIND _otool_catalog_audio_ids "${external_audio_id}" catalog_index)
    if(catalog_index EQUAL -1)
        message(FATAL_ERROR
            "External audio ID '${external_audio_id}' is not present in "
            "AUDIO_FILE_CONFIGS")
    endif()
endforeach()

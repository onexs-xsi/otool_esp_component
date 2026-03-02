/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef otool_toolbox_H
#define otool_toolbox_H

#include "esp_err.h"
#include <stdexcept>
#include "i2c_bus_tools.h"

#ifdef OTOOL_ENABLE_AUDIO
#include "audio_tools.h"
#endif

#ifdef OTOOL_ENABLE_IR
#include "ir_tools.h"
#endif

#ifdef OTOOL_ENABLE_SD
#include "sd_tools.h"
#endif

#ifdef OTOOL_ENABLE_RTC_RX8130
#include "ic_rx8130.h"
#endif

/**
 * @brief otool_toolbox 工具箱基类
 * 
 */
class otool_toolbox {
protected:
    bool initialized;    ///< 系统是否已初始化

public:
    i2c_bus_tools i2c_tools;  ///< I2C总线工具对象

#ifdef OTOOL_ENABLE_AUDIO
    audio_tools audio; ///< 音频处理工具对象
#endif

#ifdef OTOOL_ENABLE_IR
    ir_tools ir; ///< 多协议红外工具对象
#endif

#ifdef OTOOL_ENABLE_SD
    sd_tools sd_card; ///< SD卡工具对象
#endif

#ifdef OTOOL_ENABLE_RTC_RX8130
    rx8130_tools rtc_rx8130; ///< RX8130 RTC工具对象
#endif

    /**
     * @brief 构造函数
     * 
     * 创建otool_toolbox对象，初始化内部状态
     */
    otool_toolbox();

    /**
     * @brief 析构函数
     * 
     * 销毁otool_toolbox对象，清理资源
     */
    virtual ~otool_toolbox();

    /**
     * @brief 获取初始化状态
     * 
     * @return bool 返回是否已初始化
     */
    bool is_initialized() const;
};

#endif // otool_toolbox_H

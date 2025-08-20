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
#include "audio_es_tools.h"
#include "ir_nec_tools.h"

/**
 * @brief otool_toolbox 工具箱基类
 * 
 */
class otool_toolbox {
protected:
    bool initialized;    ///< 系统是否已初始化

public:
    i2c_bus_tools i2c_tools;  ///< I2C总线工具对象
    audio_es_tools audio_tools; ///< 音频处理工具对象
    ir_nec_tools ir_nec; ///< IR NEC 红外工具对象

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

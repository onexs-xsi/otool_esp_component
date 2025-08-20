/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "otool_toolbox.h"
#include "i2c_bus_tools.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "otool_toolbox";

// otool_toolbox类的构造函数
otool_toolbox::otool_toolbox() 
{
    ESP_LOGI(TAG, "otool_toolbox object created");
    initialized = false;
    // i2c_bus_tools 作为成员对象会自动构造
}

// otool_toolbox类的析构函数
otool_toolbox::~otool_toolbox() 
{
    ESP_LOGI(TAG, "otool_toolbox object destroyed");
    // i2c_bus_tools 作为成员对象会自动析构
}

// 获取初始化状态
bool otool_toolbox::is_initialized() const
{
    return initialized;
}
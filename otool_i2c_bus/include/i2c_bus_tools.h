/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __I2C_BUS_TOOLS_H__
#define __I2C_BUS_TOOLS_H__

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  #include <driver/i2c_master.h>
  #include <driver/i2c_types.h>
#else
  #include <driver/i2c.h>
#endif

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_3
#define I2C_MASTER_SCL_IO GPIO_NUM_2
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_SCAN_ADDR_NUM 128

/**
 * @brief I2C 内部上拉使能枚举
 */
typedef enum {
    I2C_PULLUP_DISABLE = 0,  ///< 禁用内部上拉
    I2C_PULLUP_ENABLE = 1,   ///< 启用内部上拉
} i2c_pullup_enable_t;

/**
 * @brief i2c_bus_tools 类
 * 
 * 提供 I2C 总线操作的功能，包括设备注册、扫描和管理
 */
class i2c_bus_tools {
private:
    i2c_bus_handle_t g_i2c_bus;                                  ///< I2C 总线句柄
    i2c_bus_device_handle_t i2c_device_all[I2C_SCAN_ADDR_NUM];   ///< I2C 设备句柄数组
    gpio_num_t i2c_sda_pin;                                      ///< I2C SDA 引脚
    gpio_num_t i2c_scl_pin;                                      ///< I2C SCL 引脚
    uint32_t i2c_frequency;                                      ///< I2C 频率
    i2c_port_t i2c_port_num;                                     ///< I2C 端口号
    i2c_pullup_enable_t i2c_pullup;                              ///< 内部上拉使能
    bool initialized;                                            ///< 初始化状态

public:
    /**
     * @brief 构造函数
     * 
     * 使用默认参数创建 i2c_bus_tools 对象
     */
    i2c_bus_tools();

    /**
     * @brief 构造函数（带参数）
     * 
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚  
     * @param frequency I2C频率
     */
    i2c_bus_tools(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency);

    /**
     * @brief 构造函数（完整参数）
     * 
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚  
     * @param frequency I2C频率
     * @param port_num I2C端口号
     */
    i2c_bus_tools(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num);

    /**
     * @brief 析构函数
     * 
     * 销毁 i2c_bus_tools 对象，清理I2C资源
     */
    ~i2c_bus_tools();

    /**
     * @brief 初始化I2C总线设备（使用已配置的参数）
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t init(void);

    /**
     * @brief 反初始化I2C总线设备，释放所有资源
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t deinit(void);

    /**
     * @brief 注册I2C总线设备（使用默认参数）
     * 
     * @deprecated 此函数已弃用，请使用 init() 代替
     * @return esp_err_t 返回操作结果
     */
    [[deprecated("请使用 init() 代替")]]
    esp_err_t register_i2c_bus_device(void);

    /**
     * @brief 注销I2C总线设备（使用内部句柄）
     * 
     * @deprecated 此函数已弃用，请使用 deinit() 代替
     * @return esp_err_t 返回操作结果
     */
    [[deprecated("请使用 deinit() 代替")]]
    esp_err_t unregister_i2c_bus_device(void);

    /**
     * @brief 扫描并添加I2C总线设备
     * 
     * @return esp_err_t 返回操作结果
     */
    esp_err_t scan_and_add_i2c_bus_devices(void);

    /**
     * @brief 获取I2C总线句柄
     * 
     * @return i2c_bus_handle_t I2C总线句柄
     */
    i2c_bus_handle_t get_i2c_bus_handle(void) const;

    /**
     * @brief 获取指定地址的I2C设备句柄
     * 
     * @param addr 设备地址
     * @return i2c_bus_device_handle_t I2C设备句柄
     */
    i2c_bus_device_handle_t get_i2c_device_handle(uint8_t addr) const;

    /**
     * @brief 获取I2C主总线句柄
     * 
     * @return i2c_master_bus_handle_t I2C主总线句柄
     */
    i2c_master_bus_handle_t get_i2c_master_bus_handle(void) const;

    /**
     * @brief 设置I2C参数
     * 
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚
     * @param frequency I2C频率
     */
    void set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency);

    /**
     * @brief 设置I2C参数（包含端口号）
     * 
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚
     * @param frequency I2C频率
     * @param port_num I2C端口号
     */
    void set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num);

    /**
     * @brief 设置I2C参数（包含端口号和上拉配置）
     * 
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚
     * @param frequency I2C频率
     * @param port_num I2C端口号
     * @param pullup 内部上拉使能（SDA和SCL同时配置）
     */
    void set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num, 
                        i2c_pullup_enable_t pullup);

    /**
     * @brief 获取当前I2C端口号
     * 
     * @return i2c_port_t 当前使用的I2C端口号
     */
    i2c_port_t get_i2c_port_num(void) const;

  /**
   * @brief 从内部表中按地址分离设备句柄（不执行删除，仅置空）
   * 
   * @param addr I2C 设备地址（0-127）
   */
  void detach_i2c_device_by_addr(uint8_t addr);

  /**
   * @brief 更新指定地址的I2C设备句柄
   * 
   * @param addr I2C 设备地址（0-127）
   * @param new_handle 新的设备句柄
   */
  void update_i2c_device_handle(uint8_t addr, i2c_bus_device_handle_t new_handle);
};

#endif // __I2C_BUS_TOOLS_H__
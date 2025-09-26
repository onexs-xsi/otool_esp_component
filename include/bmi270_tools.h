/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __BMI270_TOOLS_H__
#define __BMI270_TOOLS_H__

#include <stdint.h>
#include "math.h"
#include "esp_err.h"
#include "i2c_bus.h"
#include "i2c_bus_tools.h"

#include "bmi270.h"
#include "bmi2.h"
// #include "bmi270_maximum_fifo.h"  // Maximum FIFO 变体支持 TODO
#include "bmi270_context.h"         // BMI270 Context API 支持
#include "bmm150.h"
#include <float.h>

#define I2C_MASTER_TIMEOUT_MS 100

#define BMI2_FIFO_AUX_FRAME_COUNT        UINT8_C(185)
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(185)
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(185)

#define BMI270_READ_WRITE_LEN       46

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE 2048
#define BMI2_FIFO_RAW_DATA_USER_LENGTH 2048

#ifndef I2C_BMI270_ADDR
#define I2C_BMI270_ADDR 0x68
#endif

/**
 * @brief BMI270 6轴传感器（加速度计+陀螺仪）I2C 封装类
 * 
 * 基于 Bosch BMI270 SensorAPI 构建的面向对象封装，提供传感器初始化、
 * 配置、数据读取、中断管理等完整功能。
 */
class bmi270_tools {
public:
    // BMI270变体模式枚举
    enum bmi270_mode_t {
        MODE_CONTEXT = 0,           // Context变体 - 轻量级，专注上下文感知
        MODE_BASE = 1,              // Base/Traditional变体 - 完整功能集
        MODE_LEGACY = 2,            // Legacy变体 - 兼容老版本，丰富检测功能
        MODE_MAXIMUM_FIFO = 3       // Maximum FIFO变体 - 6KB大缓存
    };

    // 传感器类型枚举
    enum sensor_type_t {
        SENSOR_ACCEL = BMI2_ACCEL,
        SENSOR_GYRO = BMI2_GYRO,
        SENSOR_AUX = BMI2_AUX
    };

    // 加速度量程配置
    enum accel_range_t {
        ACCEL_RANGE_2G = BMI2_ACC_RANGE_2G,
        ACCEL_RANGE_4G = BMI2_ACC_RANGE_4G,
        ACCEL_RANGE_8G = BMI2_ACC_RANGE_8G,
        ACCEL_RANGE_16G = BMI2_ACC_RANGE_16G
    };

    // 陀螺仪量程配置
    enum gyro_range_t {
        GYRO_RANGE_125 = BMI2_GYR_RANGE_125,
        GYRO_RANGE_250 = BMI2_GYR_RANGE_250,
        GYRO_RANGE_500 = BMI2_GYR_RANGE_500,
        GYRO_RANGE_1000 = BMI2_GYR_RANGE_1000,
        GYRO_RANGE_2000 = BMI2_GYR_RANGE_2000
    };

    // 输出数据率配置
    enum odr_t {
        ODR_25HZ = BMI2_ACC_ODR_25HZ,
        ODR_50HZ = BMI2_ACC_ODR_50HZ,
        ODR_100HZ = BMI2_ACC_ODR_100HZ,
        ODR_200HZ = BMI2_ACC_ODR_200HZ,
        ODR_400HZ = BMI2_ACC_ODR_400HZ,
        ODR_800HZ = BMI2_ACC_ODR_800HZ,
        ODR_1600HZ = BMI2_ACC_ODR_1600HZ
    };

    // 中断引脚配置
    enum int_pin_t {
        INT_PIN_1 = BMI2_INT1,
        INT_PIN_2 = BMI2_INT2,
        INT_PIN_BOTH = BMI2_INT_BOTH
    };

    // 传感器数据结构
    struct sensor_data_t {
        float acc_x, acc_y, acc_z;      // 加速度数据 (LSB)
        float gyr_x, gyr_y, gyr_z;      // 陀螺仪数据 (LSB)
        int16_t mag_x, mag_y, mag_z;      // 磁力计数据 (LSB)
        uint32_t sensor_time;             // 传感器时间戳
        bool acc_valid;                   // 加速度数据有效标志
        bool gyr_valid;                   // 陀螺仪数据有效标志
        bool mag_valid;                   // 磁力计数据有效标志
    };

    // 磁力计数据结构
    struct mag_data_t {
        int32_t x, y, z;                  // 磁场数据 (uT)
        bool valid;                       // 磁力计数据有效标志
    };

    // 传感器配置结构
    struct sensor_config_t {
        accel_range_t acc_range;
        gyro_range_t gyr_range;
        odr_t acc_odr;
        odr_t gyr_odr;
    };

    // 运动检测配置
    struct motion_config_t {
        uint8_t duration;    // 运动持续时间（LSB = 20ms）
        uint8_t threshold;   // 运动阈值（LSB = 0.48mg）
    };

public:
    /**
     * @brief 构造函数
     * @param addr I2C 地址（默认 0x68）
     * @param bus_freq_hz I2C 频率（默认 400kHz）
     */
    explicit bmi270_tools(uint8_t addr = I2C_BMI270_ADDR, uint32_t bus_freq_hz = 400000);
    ~bmi270_tools();

    /**
     * @brief 初始化BMI270传感器
     * @param bus I2C 主总线句柄
     * @param i2c_device I2C设备句柄指针，如果为nullptr则自动创建设备句柄
     * @param enable_magnetometer 是否启用BMM150磁力计 (默认false)
     * @param mode BMI270变体模式 (默认MODE_CONTEXT)
     */
    esp_err_t init(i2c_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device = nullptr, bool enable_magnetometer = false, bmi270_mode_t mode = MODE_CONTEXT);

    /**
     * @brief 去初始化：释放设备句柄
     */
    esp_err_t deinit();

    // === 传感器配置 ===
    /**
     * @brief 启用传感器
     * @param sensors 传感器类型数组
     * @param count 传感器数量
     */
    esp_err_t enable_sensors(const sensor_type_t *sensors, uint8_t count);

    /**
     * @brief 禁用传感器
     * @param sensors 传感器类型数组
     * @param count 传感器数量
     */
    esp_err_t disable_sensors(const sensor_type_t *sensors, uint8_t count);

    /**
     * @brief 配置传感器参数
     * @param config 传感器配置结构
     */
    esp_err_t configure_sensors(const sensor_config_t &config);

    /**
     * @brief 使用默认配置启用加速度计和陀螺仪
     */
    esp_err_t enable_default_sensors();

    // === 数据读取 ===
    /**
     * @brief 读取传感器数据
     * @param data 输出数据结构
     */
    esp_err_t get_sensor_data(sensor_data_t &data);

    /**
     * @brief 读取原始传感器数据（BMI API格式）
    * @param data BMI270 API数据结构（单次最新数据）
     */
    esp_err_t get_raw_sensor_data(struct bmi2_sens_data *data);

    // === 中断和特殊功能 ===
    /**
     * @brief 检查中断状态
     * @return true 如果检测到中断
     */
    bool check_interrupt_status();

    /**
     * @brief 清除中断
     */
    esp_err_t clear_interrupt();

    /**
     * @brief 启用INT引脚中断输出
     * @param pin 中断引脚选择
     * @param active_high 中断输出电平 (true=高电平有效，false=低电平有效)
     * @param open_drain 输出模式 (true=开漏输出，false=推挽输出)
     * @param latch 锁存模式 (true=锁存直到读取，false=脉冲模式)
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t enable_interrupt(int_pin_t pin = INT_PIN_1, bool active_high = false, 
                              bool open_drain = true, bool latch = true);

    /**
     * @brief 禁用INT引脚中断输出
     * @param pin 中断引脚选择
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t disable_interrupt(int_pin_t pin = INT_PIN_1);

    /**
     * @brief 映射中断源到指定引脚（通用函数）
     * @param interrupt_type 中断类型 (BMI2_DRDY_INT, BMI2_ANY_MOTION 等)
     * @param pin 中断引脚选择
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t map_interrupt_to_pin(uint8_t interrupt_type, int_pin_t pin);

    /**
     * @brief 配置any motion中断参数
     * @param config any motion配置参数
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t configure_any_motion(const motion_config_t &config);

    // === 实用功能 ===
    /**
     * @brief 转换加速度LSB到mg
     * @param lsb 原始LSB值
     * @param range 当前量程配置
     * @return 加速度值(mg)
     */
    static float convert_accel_lsb_to_mg(int16_t lsb, accel_range_t range);

    /**
     * @brief 转换陀螺仪LSB到度/秒
     * @param lsb 原始LSB值
     * @param range 当前量程配置
     * @return 角速度值(度/秒)
     */
    static float convert_gyro_lsb_to_dps(int16_t lsb, gyro_range_t range);

    /**
     * @brief 获取芯片ID
     * @param chip_id 输出芯片ID
     */
    esp_err_t get_chip_id(uint8_t &chip_id);

    /**
     * @brief 软件复位
     */
    esp_err_t soft_reset();

    /**
     * @brief 进入suspend模式以降低功耗
     * @param suspend_accel 是否暂停加速度计 (默认true)
     * @param suspend_gyro 是否暂停陀螺仪 (默认true) 
     * @param suspend_aux 是否暂停辅助传感器 (默认true)
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t enter_suspend_mode(bool suspend_accel = true, bool suspend_gyro = true, bool suspend_aux = true);

    /**
     * @brief 退出suspend模式恢复正常操作
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t exit_suspend_mode();

    // 磁力计接口函数 (BMM150)
    int configure_magnetometer();                                  // 配置BMM150磁力计
    int read_magnetometer_data(mag_data_t* mag_data);             // 读取磁力计数据

    // === 高级融合功能 ===
    
    /**
     * @brief 校准传感器
     * @param calibration_time_ms 校准时间（毫秒）
     * @param sample_count 校准样本数量
     * @return ESP_OK 成功，其他值失败
     */
    esp_err_t calibrate_sensors(uint32_t calibration_time_ms = 10000, uint32_t sample_count = 1000);

    // 基本属性
    i2c_bus_device_handle_t device() const;
    uint8_t address() const;
    bool is_initialized() const;

private:
    // BMI270 API回调函数
    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmm_i2c_read(uint8_t reg_addr, uint8_t *aux_data, uint32_t len,  void *intf_ptr);
    static int8_t bmm_i2c_write(uint8_t reg_addr, const uint8_t *aux_data, uint32_t len,  void *intf_ptr);
    static void delay_us(uint32_t period, void *intf_ptr);

    // 内部辅助函数
    esp_err_t map_feature_interrupt(uint8_t feature_type, int_pin_t pin);
    static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
    static void print_bmi2_api_error(int8_t rslt);
    static void print_bmm150_api_error(int8_t rslt);
    
    // 检查传感器启用状态
    bool is_magnetometer_enabled() const { return (_enabled_sensors_mask & (1ULL << BMI2_AUX)) != 0; }

    // 传感器校准函数
    esp_err_t calibrate_gyroscope(uint32_t sample_count);
    esp_err_t calibrate_accelerometer(uint32_t sample_count);
    esp_err_t calibrate_magnetometer(uint32_t sample_count);
    void apply_sensor_calibration(sensor_data_t &data);

    // 复位后恢复配置
    esp_err_t restore_state_after_reset();

private:
    i2c_bus_handle_t _bus;  // 使用抽象 bus 句柄
    i2c_bus_device_handle_t _dev;
    uint8_t _addr;
    uint32_t _bus_freq;
    bool _initialized;
    bool _owns_dev;

    struct bmi2_fifo_frame fifoframe = { 0 };
    uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE];
    
    struct bmi2_dev _bmi270_dev;        // BMI270 API设备结构
    struct bmm150_dev _bmm150_dev;      // BMM150 API设备结构
    struct bmm150_settings _bmm150_mag_settings;    // BMM150 配置结构
    struct bmm150_mag_data _bmm150_mag_data;    // BMM150 磁力计数据结构
    sensor_config_t _current_config; // 当前配置
    bmi270_mode_t _current_mode;     // 当前BMI270变体模式
    // 记录已启用的传感器选择掩码（bit = BMI2_*）供复位后恢复
    uint64_t _enabled_sensors_mask = 0;

};

#endif // __BMI270_TOOLS_H__
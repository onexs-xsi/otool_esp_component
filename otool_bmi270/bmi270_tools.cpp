/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "bmi270_tools.h"
#include "i2c_bus_tools.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "bmi270_tools";

bmi270_tools::bmi270_tools(uint8_t addr, uint32_t bus_freq_hz)
    : _bus(nullptr), _dev(nullptr), _addr(addr), _bus_freq(bus_freq_hz), 
      _initialized(false), _owns_dev(false)
{
    // 初始化默认配置
    _current_config.acc_range = ACCEL_RANGE_4G;
    _current_config.gyr_range = GYRO_RANGE_1000;
    _current_config.acc_odr = ODR_200HZ;
    _current_config.gyr_odr = ODR_200HZ;
    
    // 初始化默认模式
    _current_mode = MODE_CONTEXT;
    
    // 清零 BMI270 设备结构
    memset(&_bmi270_dev, 0, sizeof(_bmi270_dev));
}

bmi270_tools::~bmi270_tools()
{
    deinit();
}

esp_err_t bmi270_tools::init(i2c_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device, bool enable_magnetometer, bmi270_mode_t mode)
{
    if (_initialized) {
        ESP_LOGW(TAG, "BMI270 already initialized @0x%02X", _addr);
        return ESP_OK;
    }
    if (bus == nullptr) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    _bus = bus;
    _current_mode = mode;  // 记录选择的模式
    
    // 判断是否需要自动创建设备句柄
    if (i2c_device == nullptr || *i2c_device == nullptr) {
        // 自动创建设备句柄
        _dev = i2c_bus_device_create(_bus, _addr, _bus_freq);
        if (_dev == nullptr) {
            ESP_LOGE(TAG, "Failed to create I2C device for BMI270 @0x%02X", _addr);
            return ESP_FAIL;
        }
        _owns_dev = true;
        ESP_LOGD(TAG, "Auto-created I2C device handle");
    } else {
        // 使用外部提供的设备句柄
        _dev = *i2c_device;
        _owns_dev = false;
        ESP_LOGD(TAG, "Using external I2C device handle");
    }

    // 设置初始化标志，这样I2C回调函数就可以工作了
    _initialized = true;

    // 配置 BMI270 API 接口
    _bmi270_dev.intf = BMI2_I2C_INTF;
    _bmi270_dev.read = bmi2_i2c_read;
    _bmi270_dev.write = bmi2_i2c_write;
    _bmi270_dev.delay_us = delay_us;
    _bmi270_dev.read_write_len = BMI270_READ_WRITE_LEN;
    _bmi270_dev.config_file_ptr = NULL;
    _bmi270_dev.intf_ptr = this; // 传递类实例指针

    // 预读芯片ID进行确认
    uint8_t raw_chip_id = 0x00;
    esp_err_t id_ret = i2c_bus_read_byte(_dev, BMI2_CHIP_ID_ADDR, &raw_chip_id);
    if (id_ret != ESP_OK) {
        ESP_LOGE(TAG, "Pre-read CHIP_ID failed (i2c err=%s)", esp_err_to_name(id_ret));
        if (_owns_dev) deinit();
        return id_ret;
    }
    ESP_LOGI(TAG, "BMI270 pre-read CHIP_ID=0x%02X (expected 0x%02X)", raw_chip_id, BMI270_CHIP_ID);
    if (raw_chip_id != BMI270_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected CHIP_ID 0x%02X, abort init", raw_chip_id);
        if (_owns_dev) deinit();
        return ESP_ERR_NOT_FOUND;
    }
    _bmi270_dev.chip_id = raw_chip_id;

    // 根据选择的模式进行初始化
    int8_t rslt = BMI2_OK;
    const char* mode_name = "";
    
    switch (_current_mode) {
        case MODE_CONTEXT:
            ESP_LOGI(TAG, "Starting BMI270 Context initialization...");
            rslt = bmi270_context_init(&_bmi270_dev);
            mode_name = "Context";
            break;
            
        case MODE_BASE:
            ESP_LOGI(TAG, "Starting BMI270 Base initialization...");
            rslt = bmi270_init(&_bmi270_dev);
            mode_name = "Base";
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported BMI270 mode: %d", _current_mode);
            if (_owns_dev) deinit();
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 %s initialization failed", mode_name);
        print_bmi2_api_error(rslt);
        _initialized = false; // 重置初始化标志
        if (_owns_dev) deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI270 %s initialized successfully (addr=0x%02X, freq=%lu)", 
             mode_name, _addr, (unsigned long)_bus_freq);
    
    // 如果启用磁力计，则自动配置BMM150
    if (enable_magnetometer) {

        // 先启用内部上拉
        uint8_t pupsel = BMI2_ASDA_PUPSEL_2K; // 2k 内部上拉
        int8_t trim_rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &pupsel, 1, &_bmi270_dev);
        if (trim_rslt != BMI2_OK) {
            ESP_LOGW(TAG, "Set internal AUX pull-up failed rslt=%d", trim_rslt);
        } else {
            ESP_LOGI(TAG, "Internal AUX pull-up enabled (2k)");
        }

        ESP_LOGI(TAG, "Auto configuring BMM150 magnetometer...");
        int mag_ret = configure_magnetometer();
        if (mag_ret == 0) {
            ESP_LOGI(TAG, "BMM150 magnetometer configured successfully");
        } else {
            ESP_LOGW(TAG, "BMM150 magnetometer config failed: %d (continue BMI270 init)", mag_ret);
        }
    }
    
    return ESP_OK;
}

esp_err_t bmi270_tools::deinit()
{
    if (_dev && _owns_dev) {
        i2c_bus_device_delete(&_dev);
    }
    _dev = nullptr;
    _bus = nullptr;
    _initialized = false;
    _owns_dev = false;
    memset(&_bmi270_dev, 0, sizeof(_bmi270_dev));
    return ESP_OK;
}

esp_err_t bmi270_tools::enable_sensors(const sensor_type_t *sensors, uint8_t count)
{
    if (!_initialized || sensors == nullptr || count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // 避免 VLA：最大同时启用传感器数量（加速度/陀螺/aux/feature），这里保守取 8
    if (count > 8) return ESP_ERR_INVALID_ARG;
    uint8_t sens_list[8];
    for (uint8_t i = 0; i < count; i++) {
        sens_list[i] = (uint8_t)sensors[i];
        _enabled_sensors_mask |= (uint64_t)1 << sens_list[i];
    }

    // 根据当前模式调用相应的sensor enable函数
    int8_t rslt = BMI2_OK;
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_sensor_enable(sens_list, count, &_bmi270_dev);
            break;
            
        case MODE_BASE:
            rslt = bmi270_sensor_enable(sens_list, count, &_bmi270_dev);
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for sensor enable: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Enabled %d sensors", count);
    return ESP_OK;
}

esp_err_t bmi270_tools::disable_sensors(const sensor_type_t *sensors, uint8_t count)
{
    if (!_initialized || sensors == nullptr || count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (count > 8) return ESP_ERR_INVALID_ARG;
    uint8_t sens_list[8];
    for (uint8_t i = 0; i < count; i++) {
        sens_list[i] = (uint8_t)sensors[i];
        _enabled_sensors_mask &= ~((uint64_t)1 << sens_list[i]);
    }

    // 根据当前模式调用相应的sensor disable函数
    int8_t rslt = BMI2_OK;
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_sensor_disable(sens_list, count, &_bmi270_dev);
            break;
            
        case MODE_BASE:
            rslt = bmi270_sensor_disable(sens_list, count, &_bmi270_dev);
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for sensor disable: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }

    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to disable sensors");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Disabled %d sensors", count);
    return ESP_OK;
}

esp_err_t bmi270_tools::configure_sensors(const sensor_config_t &config)
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi2_sens_config sens_config[2];
    sens_config[0].type = BMI2_ACCEL;
    sens_config[1].type = BMI2_GYRO;

    // 获取默认配置
    int8_t rslt = bmi2_get_sensor_config(sens_config, 2, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get sensor config");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 配置加速度计
    sens_config[0].cfg.acc.odr = (uint8_t)config.acc_odr;
    sens_config[0].cfg.acc.range = (uint8_t)config.acc_range;

    // 配置陀螺仪
    sens_config[1].cfg.gyr.odr = (uint8_t)config.gyr_odr;
    sens_config[1].cfg.gyr.range = (uint8_t)config.gyr_range;

    // 根据当前模式调用相应的set sensor config函数
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_set_sensor_config(sens_config, 2, &_bmi270_dev);
            break;
            
        case MODE_BASE:
            rslt = bmi270_set_sensor_config(sens_config, 2, &_bmi270_dev);
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for sensor config: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to set sensor config");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    _current_config = config;
    ESP_LOGI(TAG, "Sensor configuration updated");
    return ESP_OK;
}

esp_err_t bmi270_tools::enable_default_sensors()
{
    // 配置传感器
    esp_err_t ret = configure_sensors(_current_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // 启用加速度计、陀螺仪和辅助传感器
    sensor_type_t sensors[] = {SENSOR_ACCEL, SENSOR_GYRO, SENSOR_AUX};
    esp_err_t enable_ret = enable_sensors(sensors, 3);
    
    ESP_LOGI(TAG, "Default sensors enabled (ACC, GYR, AUX): %s", 
             enable_ret == ESP_OK ? "SUCCESS" : "FAILED");
    
    return enable_ret;
}

esp_err_t bmi270_tools::get_sensor_data(sensor_data_t &data)
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi2_sens_data sensor_data;
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get sensor data");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    data.sensor_time = (uint32_t)(esp_timer_get_time() / 1000); // 使用ESP32启动时间(ms)

    // 解析加速度数据
    // 注意：BMI2_DRDY_ACC 对应加速度，BMI2_DRDY_GYR 对应陀螺。之前误用了 AUX 标志读取 accel。
    if (sensor_data.status & BMI2_DRDY_ACC) {
        accel_range_t ar = _current_config.acc_range;
        float mg_x = convert_accel_lsb_to_mg(sensor_data.acc.x, ar);
        float mg_y = convert_accel_lsb_to_mg(sensor_data.acc.y, ar);
        float mg_z = convert_accel_lsb_to_mg(sensor_data.acc.z, ar);
        data.acc_x = mg_x * 0.001f * GRAVITY_EARTH; // 转换为 m/s^2
        data.acc_y = mg_y * 0.001f * GRAVITY_EARTH;
        data.acc_z = mg_z * 0.001f * GRAVITY_EARTH;
        data.acc_valid = true;
    }
    if (sensor_data.status & BMI2_DRDY_GYR) {
        gyro_range_t gr = _current_config.gyr_range;
        data.gyr_x = convert_gyro_lsb_to_dps(sensor_data.gyr.x, gr);
        data.gyr_y = convert_gyro_lsb_to_dps(sensor_data.gyr.y, gr);
        data.gyr_z = convert_gyro_lsb_to_dps(sensor_data.gyr.z, gr);
        data.gyr_valid = true;
    }
#if 0
    if (data.acc_valid) {
        ESP_LOGD(TAG, "ACC(m/s2)=%.3f %.3f %.3f", data.acc_x, data.acc_y, data.acc_z);
    }
    if (data.gyr_valid) {
        ESP_LOGD(TAG, "GYR(dps)=%.3f %.3f %.3f", data.gyr_x, data.gyr_y, data.gyr_z);
    }
#endif

    if(sensor_data.status & BMI2_DRDY_AUX)
    {    
        struct bmm150_mag_data mag_data;
        rslt = bmm150_aux_mag_data(sensor_data.aux_data, &mag_data, &_bmm150_dev);
        if (rslt == BMM150_OK) {
            data.mag_x = (int16_t)mag_data.x;
            data.mag_y = (int16_t)mag_data.y;
            data.mag_z = (int16_t)mag_data.z;
            data.mag_valid = true;
        }

    }


    return ESP_OK;
}

esp_err_t bmi270_tools::get_raw_sensor_data(struct bmi2_sens_data *data)
{
    if (!_initialized || data == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    int8_t rslt = bmi2_get_sensor_data(data, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool bmi270_tools::check_interrupt_status()
{
    if (!_initialized) {
        return false;
    }

    uint16_t int_status = 0;
    int8_t rslt = bmi2_get_int_status(&int_status, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        return false;
    }

    bool interrupt_detected = false;

    // 检查显著运动检测中断
    if (int_status & BMI270_SIG_MOT_STATUS_MASK) {
        ESP_LOGI(TAG, "Significant motion detected");
        interrupt_detected = true;
    }

    // 检查计步器中断
    if (int_status & BMI270_STEP_CNT_STATUS_MASK) {
        ESP_LOGI(TAG, "Step counter interrupt");
        interrupt_detected = true;
    }

    // 检查活动识别中断
    if (int_status & BMI270_STEP_ACT_STATUS_MASK) {
        ESP_LOGI(TAG, "Activity recognition interrupt");
        interrupt_detected = true;
    }

    // 检查手腕唤醒中断
    if (int_status & BMI270_WRIST_WAKE_UP_STATUS_MASK) {
        ESP_LOGI(TAG, "Wrist wake-up detected");
        interrupt_detected = true;
    }

    // 检查手腕手势中断
    if (int_status & BMI270_WRIST_GEST_STATUS_MASK) {
        ESP_LOGI(TAG, "Wrist gesture detected");
        interrupt_detected = true;
    }

    // 检查无运动检测中断
    if (int_status & BMI270_NO_MOT_STATUS_MASK) {
        ESP_LOGI(TAG, "No motion detected");
        interrupt_detected = true;
    }

    // 检查任意运动检测中断
    if (int_status & BMI270_ANY_MOT_STATUS_MASK) {
        ESP_LOGI(TAG, "Any motion detected");
        interrupt_detected = true;
    }

    return interrupt_detected;
}

esp_err_t bmi270_tools::clear_interrupt()
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 根据BMI270官方API，读取中断状态寄存器会自动清除中断状态
    // 这是BMI270芯片的硬件特性，不需要重新配置中断引脚
    uint16_t int_status = 0;
    int8_t rslt = bmi2_get_int_status(&int_status, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status for clearing");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Interrupt status cleared: 0x%04X", int_status);
    return ESP_OK;
}

esp_err_t bmi270_tools::enable_interrupt(int_pin_t pin, bool active_high, bool open_drain, bool latch)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi2_int_pin_config pin_config = {};

    // 配置中断引脚类型
    pin_config.pin_type = pin;
    
    // 配置锁存模式
    pin_config.int_latch = latch ? BMI2_INT_LATCH : BMI2_INT_NON_LATCH;

    // 根据引脚类型配置相应的引脚参数
    auto configure_pin = [&](uint8_t index) {
        pin_config.pin_cfg[index].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[index].lvl = active_high ? BMI2_INT_ACTIVE_HIGH : BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[index].od = open_drain ? BMI2_INT_OPEN_DRAIN : BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[index].output_en = BMI2_INT_OUTPUT_ENABLE;
    };

    switch (pin) {
        case INT_PIN_1:
            configure_pin(0);
            break;
        case INT_PIN_2:
            configure_pin(1);
            break;
        case INT_PIN_BOTH:
            configure_pin(0);
            configure_pin(1);
            break;
    }

    int8_t rslt = bmi2_set_int_pin_config(&pin_config, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to enable interrupt pin");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Interrupt enabled on pin %d (active_%s, %s, %s)", 
             pin, active_high ? "high" : "low",
             open_drain ? "open_drain" : "push_pull",
             latch ? "latch" : "non_latch");

    return ESP_OK;
}

esp_err_t bmi270_tools::disable_interrupt(int_pin_t pin)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi2_int_pin_config pin_config = {};

    // 配置中断引脚类型
    pin_config.pin_type = pin;
    pin_config.int_latch = BMI2_INT_NON_LATCH;

    // 根据引脚类型禁用相应的引脚输出
    auto disable_pin = [&](uint8_t index) {
        pin_config.pin_cfg[index].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[index].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[index].od = BMI2_INT_OPEN_DRAIN;
        pin_config.pin_cfg[index].output_en = BMI2_INT_OUTPUT_DISABLE;  // 关键：禁用输出
    };

    switch (pin) {
        case INT_PIN_1:
            disable_pin(0);
            break;
        case INT_PIN_2:
            disable_pin(1);
            break;
        case INT_PIN_BOTH:
            disable_pin(0);
            disable_pin(1);
            break;
    }

    int8_t rslt = bmi2_set_int_pin_config(&pin_config, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to disable interrupt pin");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Interrupt disabled on pin %d", pin);
    return ESP_OK;
}

esp_err_t bmi270_tools::map_interrupt_to_pin(uint8_t interrupt_type, int_pin_t pin)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int8_t rslt = BMI2_OK;

    // 统一使用特征中断映射，避免BMI2_ANY_MOTION与BMI2_DRDY_INT值冲突问题
    struct bmi2_sens_int_config sens_int = {
        .type = interrupt_type,
        .hw_int_pin = (enum bmi2_hw_int_pin)pin
    };
    
    // 根据当前模式选择正确的映射函数
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_map_feat_int(&sens_int, 1, &_bmi270_dev);
            break;
            
        case MODE_BASE:
            rslt = bmi270_map_feat_int(&sens_int, 1, &_bmi270_dev);
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for feature interrupt mapping: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to map feature interrupt %d to pin %d", interrupt_type, pin);
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Feature interrupt %d (%s) mapped to pin %d", 
             interrupt_type, 
             (interrupt_type == BMI2_ANY_MOTION) ? "ANY_MOTION" :
             (interrupt_type == BMI2_NO_MOTION) ? "NO_MOTION" :
             (interrupt_type == BMI2_SIG_MOTION) ? "SIG_MOTION" : "OTHER", 
             pin);

    return ESP_OK;
}

esp_err_t bmi270_tools::configure_any_motion(const motion_config_t &config)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Configuring any-motion detection with duration=%u, threshold=%u", config.duration, config.threshold);

    // 创建传感器配置结构
    struct bmi2_sens_config sens_config = {0};
    sens_config.type = BMI2_ANY_MOTION;

    // 创建中断引脚配置结构
    struct bmi2_int_pin_config pin_config = {0};

    int8_t rslt = BMI2_OK;

    // 根据当前模式获取传感器配置
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_get_sensor_config(&sens_config, 1, &_bmi270_dev);
            break;
        case MODE_BASE:
            rslt = bmi270_get_sensor_config(&sens_config, 1, &_bmi270_dev);
            break;
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for any-motion configuration: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }

    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get default any-motion configuration");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 获取中断引脚配置
    rslt = bmi2_get_int_pin_config(&pin_config, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get interrupt pin configuration");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 配置any-motion参数
    sens_config.cfg.any_motion.duration = config.duration;
    sens_config.cfg.any_motion.threshold = config.threshold;

    // 设置传感器配置
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_set_sensor_config(&sens_config, 1, &_bmi270_dev);
            break;
        case MODE_BASE:
            rslt = bmi270_set_sensor_config(&sens_config, 1, &_bmi270_dev);
            break;
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for any-motion configuration: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }

    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to set any-motion configuration");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Any-motion configuration applied successfully");
    return ESP_OK;
}

float bmi270_tools::convert_accel_lsb_to_mg(int16_t lsb, accel_range_t range)
{
    float sensitivity = 0.0f;
    switch (range) {
        case ACCEL_RANGE_2G:  sensitivity = 16384.0f; break; // 2^15 / 2g
        case ACCEL_RANGE_4G:  sensitivity = 8192.0f; break;  // 2^15 / 4g
        case ACCEL_RANGE_8G:  sensitivity = 4096.0f; break;  // 2^15 / 8g
        case ACCEL_RANGE_16G: sensitivity = 2048.0f; break;  // 2^15 / 16g
        default: sensitivity = 8192.0f; break;
    }
    return (float)lsb / sensitivity * 1000.0f; // 转换为mg
}

float bmi270_tools::convert_gyro_lsb_to_dps(int16_t lsb, gyro_range_t range)
{
    float sensitivity = 0.0f;
    switch (range) {
        case GYRO_RANGE_125:  sensitivity = 262.14f; break; // 2^15 / 125
        case GYRO_RANGE_250:  sensitivity = 131.07f; break; // 2^15 / 250
        case GYRO_RANGE_500:  sensitivity = 65.54f; break;  // 2^15 / 500
        case GYRO_RANGE_1000: sensitivity = 32.77f; break;  // 2^15 / 1000
        case GYRO_RANGE_2000: sensitivity = 16.38f; break;  // 2^15 / 2000
        default: sensitivity = 32.77f; break;
    }
    return (float)lsb / sensitivity;
}

esp_err_t bmi270_tools::get_chip_id(uint8_t &chip_id)
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data = 0;
    esp_err_t ret = i2c_bus_read_byte(_dev, BMI2_CHIP_ID_ADDR, &data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    chip_id = data;
    return ESP_OK;
}

esp_err_t bmi270_tools::soft_reset()
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // 记录当前配置已保存在 _current_config / _enabled_sensors_mask / _current_mode
    int8_t rslt = bmi2_soft_reset(&_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 根据当前模式重新初始化
    const char* mode_name = "";
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_init(&_bmi270_dev);
            mode_name = "Context";
            break;
            
        case MODE_BASE:
            rslt = bmi270_init(&_bmi270_dev);
            mode_name = "Base";
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for soft reset: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Re-init BMI270 %s after reset failed", mode_name);
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 恢复配置
    esp_err_t ret = restore_state_after_reset();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "State restore partial failure");
    }

    ESP_LOGI(TAG, "Soft reset completed and %s state restored", mode_name);
    return ESP_OK;
}

esp_err_t bmi270_tools::restore_state_after_reset()
{
    // 重新写入传感器配置
    esp_err_t ret = configure_sensors(_current_config);
    if (ret != ESP_OK) return ret;

    // 重新启用之前启用的基础传感器（只处理 ACC/GYR/AUX - 其他 feature 需按需扩展）
    uint8_t sens_list[3];
    uint8_t cnt = 0;
    if (_enabled_sensors_mask & (1ull << BMI2_ACCEL)) sens_list[cnt++] = BMI2_ACCEL;
    if (_enabled_sensors_mask & (1ull << BMI2_GYRO))  sens_list[cnt++] = BMI2_GYRO;
    if (_enabled_sensors_mask & (1ull << BMI2_AUX))   sens_list[cnt++] = BMI2_AUX;
    if (cnt) {
        // 根据当前模式调用相应的sensor enable函数
        int8_t rslt = BMI2_OK;
        switch (_current_mode) {
            case MODE_CONTEXT:
                rslt = bmi270_context_sensor_enable(sens_list, cnt, &_bmi270_dev);
                break;
                
            case MODE_BASE:
                rslt = bmi270_sensor_enable(sens_list, cnt, &_bmi270_dev);
                break;
                
            case MODE_LEGACY:
            case MODE_MAXIMUM_FIFO:
            default:
                ESP_LOGE(TAG, "Unsupported mode for restore sensors: %d", _current_mode);
                return ESP_ERR_NOT_SUPPORTED;
        }
        
        if (rslt != BMI2_OK) {
            ESP_LOGE(TAG, "Failed to restore sensor enable state");
            print_bmi2_api_error(rslt);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

i2c_bus_device_handle_t bmi270_tools::device() const
{
    return _dev;
}

uint8_t bmi270_tools::address() const
{
    return _addr;
}

bool bmi270_tools::is_initialized() const
{
    return _initialized;
}

// === 静态回调函数实现 ===

int8_t bmi270_tools::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (reg_data == nullptr || len == 0 || intf_ptr == nullptr) {
        ESP_LOGE(TAG, "I2C read param error: data=%p, len=%lu", reg_data, len);
        return -1;
    }

    // 增加FIFO读取的长度限制
    if (len > BMI2_FIFO_RAW_DATA_BUFFER_SIZE) {
        ESP_LOGE(TAG, "I2C read length too large: %lu (max: %d)", len, BMI2_FIFO_RAW_DATA_BUFFER_SIZE);
        return -1;
    }

    bmi270_tools *instance = static_cast<bmi270_tools*>(intf_ptr);
    if (!instance->_initialized || instance->_dev == nullptr) {
        ESP_LOGE(TAG, "I2C read state error: initialized=%d, dev=%p", instance->_initialized, instance->_dev);
        return -1;
    }

    // 使用 i2c_bus 库的读取函数，参数顺序：dev_handle, mem_address, data_len, data
    esp_err_t ret = i2c_bus_read_bytes(instance->_dev, reg_addr, len, reg_data);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, len=%lu, err=%s", reg_addr, len, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

int8_t bmi270_tools::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (reg_data == nullptr || len == 0 || intf_ptr == nullptr) {
        ESP_LOGE(TAG, "I2C write param error: data=%p, len=%lu", reg_data, len);
        return -1;
    }

    // 写入操作通常不需要像FIFO读取那样大的数据量，保持合理的限制
    if (len > BMI2_FIFO_RAW_DATA_BUFFER_SIZE) {
        ESP_LOGE(TAG, "I2C write length too large: %lu (max: %d)", len, BMI2_FIFO_RAW_DATA_BUFFER_SIZE);
        return -1;
    }

    bmi270_tools *instance = static_cast<bmi270_tools*>(intf_ptr);
    if (!instance->_initialized || instance->_dev == nullptr) {
        ESP_LOGE(TAG, "I2C write state error: initialized=%d, dev=%p", instance->_initialized, instance->_dev);
        return -1;
    }

    // 使用 i2c_bus 库的写入函数，参数顺序：dev_handle, mem_address, data_len, data
    esp_err_t ret = i2c_bus_write_bytes(instance->_dev, reg_addr, len, reg_data);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: reg=0x%02X, len=%lu, err=%s", reg_addr, len, esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

void bmi270_tools::delay_us(uint32_t period, void *intf_ptr)
{
    if (period == 0) return;
    
    // 对于大于10ms的延时使用任务延时
    if (period >= 10000) {
        uint32_t ms = period / 1000;
        if (ms == 0) ms = 1;
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    
    // 对于小延时使用精确的忙等待
    esp_rom_delay_us(period);
}

// === 预留函数1：SPI接口读取函数 ===
int8_t bmi270_tools::bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // TODO: 实现SPI读取功能
    ESP_LOGW(TAG, "SPI read not implemented yet: reg=0x%02X, len=%lu", reg_addr, len);
    return BMI2_E_COM_FAIL;
}

// === 预留函数2：SPI接口写入函数 ===
int8_t bmi270_tools::bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // TODO: 实现SPI写入功能
    ESP_LOGW(TAG, "SPI write not implemented yet: reg=0x%02X, len=%lu", reg_addr, len);
    return BMI2_E_COM_FAIL;
}

int8_t bmi270_tools::bmm_i2c_read(uint8_t reg_addr, uint8_t *aux_data, uint32_t len, void *intf_ptr)
{
    if (aux_data == nullptr || len == 0 || intf_ptr == nullptr) {
        ESP_LOGE(TAG, "BMM I2C read param error: aux_data=%p, len=%lu, intf_ptr=%p", aux_data, len, intf_ptr);
        return BMI2_E_NULL_PTR;
    }

    // 参数校验
    if (len > 32) {  // BMM150初始化时可能需要读取较多数据，限制为32字节
        ESP_LOGE(TAG, "BMM I2C read length too large: %lu", len);
        return BMI2_E_OUT_OF_RANGE;
    }

    bmi270_tools *instance = static_cast<bmi270_tools*>(intf_ptr);
    if (!instance->_initialized || instance->_dev == nullptr) {
        ESP_LOGE(TAG, "BMM I2C read state error: initialized=%d, dev=%p", instance->_initialized, instance->_dev);
        return BMI2_E_NULL_PTR;
    }

    ESP_LOGD(TAG, "BMM I2C read: reg=0x%02X, len=%lu", reg_addr, len);

    // 使用BMI2 API的辅助传感器手动模式读取函数
    int8_t rslt = bmi2_read_aux_man_mode(reg_addr, aux_data, (uint16_t)len, &instance->_bmi270_dev);
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMM I2C read failed: reg=0x%02X, len=%lu, rslt=%d", reg_addr, len, rslt);
        return rslt;
    }

    // 打印读取的数据用于调试（限制打印长度避免日志过多）
    if (len <= 16) {
        ESP_LOGD(TAG, "BMM I2C read OK: reg=0x%02X, data=[", reg_addr);
        for (uint32_t i = 0; i < len; i++) {
            ESP_LOGD(TAG, "0x%02X%s", aux_data[i], (i < len-1) ? " " : "]");
        }
    } else {
        ESP_LOGD(TAG, "BMM I2C read OK: reg=0x%02X, len=%lu (data not printed due to length)", reg_addr, len);
    }

    return BMI2_OK;
}

int8_t bmi270_tools::bmm_i2c_write(uint8_t reg_addr, const uint8_t *aux_data, uint32_t len, void *intf_ptr)
{
    if (aux_data == nullptr || len == 0 || intf_ptr == nullptr) {
        ESP_LOGE(TAG, "BMM I2C write param error: aux_data=%p, len=%lu, intf_ptr=%p", aux_data, len, intf_ptr);
        return BMI2_E_NULL_PTR;
    }

    // 参数校验
    if (len > 32) {  // BMM150写入操作也放宽限制到32字节
        ESP_LOGE(TAG, "BMM I2C write length too large: %lu", len);
        return BMI2_E_OUT_OF_RANGE;
    }

    bmi270_tools *instance = static_cast<bmi270_tools*>(intf_ptr);
    if (!instance->_initialized || instance->_dev == nullptr) {
        ESP_LOGE(TAG, "BMM I2C write state error: initialized=%d, dev=%p", instance->_initialized, instance->_dev);
        return BMI2_E_NULL_PTR;
    }

    // 打印要写入的数据用于调试（限制打印长度避免日志过多）
    if (len <= 16) {
        ESP_LOGD(TAG, "BMM I2C write: reg=0x%02X, len=%lu, data=[", reg_addr, len);
        for (uint32_t i = 0; i < len; i++) {
            ESP_LOGD(TAG, "0x%02X%s", aux_data[i], (i < len-1) ? " " : "]");
        }
    } else {
        ESP_LOGD(TAG, "BMM I2C write: reg=0x%02X, len=%lu (data not printed due to length)", reg_addr, len);
    }

    // 使用BMI2 API的辅助传感器手动模式写入函数
    int8_t rslt = bmi2_write_aux_man_mode(reg_addr, aux_data, (uint16_t)len, &instance->_bmi270_dev);
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMM I2C write failed: reg=0x%02X, len=%lu, rslt=%d", reg_addr, len, rslt);
        return rslt;
    }

    ESP_LOGD(TAG, "BMM I2C write OK: reg=0x%02X, len=%lu", reg_addr, len);
    return BMI2_OK;
}

// === 私有辅助函数实现 ===

esp_err_t bmi270_tools::map_feature_interrupt(uint8_t feature_type, int_pin_t pin)
{
    struct bmi2_sens_int_config sens_int = {
        .type = feature_type, 
        .hw_int_pin = static_cast<enum bmi2_hw_int_pin>(pin)
    };
    
    int8_t rslt = bmi270_map_feat_int(&sens_int, 1, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to map feature interrupt");
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void bmi270_tools::print_bmi2_api_error(int8_t rslt)
{
    switch (rslt) {
        case BMI2_OK:
            break;
        case BMI2_W_FIFO_EMPTY:
            ESP_LOGW(TAG, "Warning [%d]: FIFO empty", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            ESP_LOGW(TAG, "Warning [%d]: FIFO partial read", rslt);
            break;
        case BMI2_E_NULL_PTR:
            ESP_LOGE(TAG, "Error [%d]: Null pointer error", rslt);
            break;
        case BMI2_E_COM_FAIL:
            ESP_LOGE(TAG, "Error [%d]: Communication failure", rslt);
            break;
        case BMI2_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG, "Error [%d]: Device not found", rslt);
            break;
        case BMI2_E_INVALID_SENSOR:
            ESP_LOGE(TAG, "Error [%d]: Invalid sensor", rslt);
            break;
        case BMI2_E_SELF_TEST_FAIL:
            ESP_LOGE(TAG, "Error [%d]: Self-test failed", rslt);
            break;
        case BMI2_E_INVALID_INT_PIN:
            ESP_LOGE(TAG, "Error [%d]: Invalid interrupt pin", rslt);
            break;
        case BMI2_E_OUT_OF_RANGE:
            ESP_LOGE(TAG, "Error [%d]: Out of range", rslt);
            break;
        case BMI2_E_ACC_INVALID_CFG:
            ESP_LOGE(TAG, "Error [%d]: Invalid accelerometer configuration", rslt);
            break;
        case BMI2_E_GYRO_INVALID_CFG:
            ESP_LOGE(TAG, "Error [%d]: Invalid gyroscope configuration", rslt);
            break;
        case BMI2_E_CONFIG_LOAD:
            ESP_LOGE(TAG, "Error [%d]: Configuration load error", rslt);
            break;
        case BMI2_E_INVALID_PAGE:
            ESP_LOGE(TAG, "Error [%d]: Invalid page", rslt);
            break;
        case BMI2_E_INVALID_INPUT:
            ESP_LOGE(TAG, "Error [%d]: Invalid input", rslt);
            break;
        case BMI2_E_INVALID_STATUS:
            ESP_LOGE(TAG, "Error [%d]: Invalid status", rslt);
            break;
        case BMI2_E_WRITE_CYCLE_TIMEOUT:
            ESP_LOGE(TAG, "Error [%d]: Write cycle timeout", rslt);
            break;
        case BMI2_E_AUX_INVALID_CFG:
            ESP_LOGE(TAG, "Error [%d]: Auxiliary sensor invalid configuration", rslt);
            break;
        default:
            ESP_LOGE(TAG, "Error [%d]: Unknown error code", rslt);
            break;
    }
}

void bmi270_tools::print_bmm150_api_error(int8_t rslt)
{
    if (rslt != BMM150_OK) {
        switch (rslt) {
            case BMM150_E_NULL_PTR:
                ESP_LOGE(TAG, "Error [%d]: Null pointer error", rslt);
                ESP_LOGE(TAG, "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.");
                break;
            case BMM150_E_COM_FAIL:
                ESP_LOGE(TAG, "Error [%d]: Communication failure error", rslt);
                ESP_LOGE(TAG, "It occurs due to read/write operation failure and also due to power failure during communication");
                break;
            case BMM150_E_DEV_NOT_FOUND:
                ESP_LOGE(TAG, "Error [%d]: Device not found error. It occurs when the device chip id is incorrectly read", rslt);
                break;
            case BMM150_E_INVALID_CONFIG:
                ESP_LOGE(TAG, "Error [%d]: Invalid sensor configuration", rslt);
                ESP_LOGE(TAG, "It occurs when there is a mismatch in the requested feature with the available one");
                break;
            default:
                ESP_LOGE(TAG, "Error [%d]: Unknown error code", rslt);
                break;
        }
    }
}

/**
 * @brief 配置BMM150磁力计
 * @return 0: 成功, 负值: 错误码
 */
int bmi270_tools::configure_magnetometer() {
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return BMI2_E_NULL_PTR;
    }

    ESP_LOGI(TAG, "Starting BMM150 magnetometer configuration...");

    int8_t rslt = BMI2_OK;
    _bmm150_dev.chip_id = BMM150_CHIP_ID;
    _bmm150_dev.read = bmm_i2c_read;
    _bmm150_dev.write = bmm_i2c_write;
    _bmm150_dev.delay_us = delay_us;
    _bmm150_dev.intf_ptr = this;
    _bmm150_dev.intf = BMM150_I2C_INTF;

    // 使用动态分配减少栈使用
    struct bmi2_sens_config *config = (struct bmi2_sens_config*)malloc(3 * sizeof(struct bmi2_sens_config));
    if (!config) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor config");
        return BMI2_E_NULL_PTR;
    }
    
    config[BMI2_ACCEL].type = BMI2_ACCEL;
    config[BMI2_GYRO].type = BMI2_GYRO;
    config[BMI2_AUX].type = BMI2_AUX;
    uint8_t sensor_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };

    rslt = bmi270_context_get_sensor_config(config, 3, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get sensor config before BMM150 setup: %d", rslt);
        print_bmi2_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "Sensor configuration retrieved for BMM150 setup");
    }

    // BMM150 接口初始化
    ESP_LOGI(TAG, "BMM150 interface configured for auxiliary sensor access");

    /* Configurations for accel. */
    config[BMI2_ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[BMI2_ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[BMI2_ACCEL].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    config[BMI2_ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[BMI2_GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[BMI2_GYRO].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;
    config[BMI2_GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[BMI2_GYRO].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    config[BMI2_GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[BMI2_GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Configurations for aux. */
    config[BMI2_AUX].cfg.aux.odr = BMI2_AUX_ODR_50HZ;
    config[BMI2_AUX].cfg.aux.aux_en = BMI2_ENABLE;
    config[BMI2_AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[BMI2_AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[BMI2_AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    config[BMI2_AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;
    config[BMI2_AUX].cfg.aux.manual_en = BMI2_ENABLE;

    // 应用配置
    rslt = bmi270_context_set_sensor_config(config, 3, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to set sensor config for BMM150 setup: %d", rslt);
        print_bmi2_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "Sensor configuration applied for BMM150 setup");
    }

    // 1. 先使能基础传感器 (加速度计和陀螺仪)，否则AUX无法工作
    rslt = bmi270_context_sensor_enable(sensor_list, 3, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors before BMM150 setup: %d", rslt);
        print_bmi2_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "Base sensors (ACC, GYR, AUX) enabled for BMM150 setup");
    }

    // 初始化BMM150
    rslt = bmm150_init(&_bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMM150: %d", rslt);
        print_bmm150_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "BMM150 initialized successfully");
    }

    // 配置BMM150 preset_mode 参数
    _bmm150_mag_settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt = bmm150_set_presetmode(&_bmm150_mag_settings, &_bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to set BMM150 preset mode: %d", rslt);
        print_bmm150_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "BMM150 preset mode set to ENHANCED");
    }

    // 配置BMM150 op_mode 参数
    _bmm150_mag_settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&_bmm150_mag_settings, &_bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to set BMM150 power mode: %d", rslt);
        print_bmm150_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "BMM150 power mode set to NORMAL");
    }

    // 重新读取配置以关闭AUX手动状态
    rslt = bmi270_context_get_sensor_config(config, 3, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to get sensor config after BMM150 setup: %d", rslt);
        print_bmi2_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "Sensor configuration retrieved after BMM150 setup");
    }
    config[BMI2_AUX].cfg.aux.manual_en = BMI2_DISABLE;
    rslt = bmi270_context_set_sensor_config(&config[BMI2_AUX], 1, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to disable AUX manual mode after BMM150 setup: %d", rslt);
        print_bmi2_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "AUX manual mode disabled after BMM150 setup");
    }

    // 将数据就绪中断映射到INT1引脚
    // rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGE(TAG, "Failed to map data ready interrupt for BMM150: %d", rslt);
    //     print_bmi2_api_error(rslt);
    //     free(config);
    //     return rslt;
    // }
    // else{
    //     ESP_LOGI(TAG, "Data ready interrupt mapped to INT1 for BMM150");
    // }
    
    // 释放内存
    free(config);

    // 禁用高级省电模式以支持
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to enable advanced power save mode: %d", rslt);
        print_bmi2_api_error(rslt);
    } else {
        ESP_LOGI(TAG, "Advanced power save mode enabled");
    }

    // 关闭FIFO
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &_bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to disable FIFO: %d", rslt);
        print_bmi2_api_error(rslt);
    }
    else{
        ESP_LOGI(TAG, "FIFO disabled before BMM150 setup");
    }

 
//     fifoframe.data = fifo_data;
//     fifoframe.length = BMI2_FIFO_RAW_DATA_USER_LENGTH;

//     /* Set FIFO configuration by enabling accel, gyro and timestamp.
//      * NOTE 1: The header mode is enabled by default.
//      * NOTE 2: By default the FIFO operating mode is in FIFO mode.
//      * NOTE 3: Sensortime is enabled by default */

//     // 使能FIFO
//     rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_ENABLE, &_bmi270_dev);
//     if (rslt != BMI2_OK) {
//         ESP_LOGW(TAG, "Failed to enable FIFO: %d", rslt);
//         print_bmi2_api_error(rslt);
//     }
//     else{
//         ESP_LOGI(TAG, "FIFO enabled for BMM150 operation");
//     }

//     rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &_bmi270_dev);
//     if (rslt != BMI2_OK) {
//         ESP_LOGW(TAG, "Failed to disable FIFO header: %d", rslt);
//         print_bmi2_api_error(rslt);
//     }
//     else{
//         ESP_LOGI(TAG, "FIFO header disabled for BMM150 operation");
//     }

//     fifoframe.data_int_map = BMI2_FFULL_INT;
//     rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &_bmi270_dev);
//     if (rslt != BMI2_OK) {
//         ESP_LOGW(TAG, "Failed to map FIFO full interrupt: %d", rslt);
//         print_bmi2_api_error(rslt);
//     }
//     else{
//         ESP_LOGI(TAG, "FIFO full interrupt mapped to INT1");
//     }

//     // 验证BMM150磁力计是否正确识别并处理FIFO数据
// #ifdef BMM150_USE_FIXED_POINT
//     ESP_LOGI(TAG, "Magnetometer data contains fraction part (last 4 bits) and decimal part");
// #endif

//     if (_bmm150_dev.chip_id == BMM150_CHIP_ID) {
//         ESP_LOGI(TAG, "Valid BMM150 (Aux) sensor - Chip ID : 0x%x", _bmm150_dev.chip_id);

//         uint16_t int_status = 0;
//         uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
//         uint16_t gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;
//         uint16_t aux_frame_length = BMI2_FIFO_AUX_FRAME_COUNT;
//         uint16_t fifo_length = 0;
//         uint16_t index = 0;
//         int8_t try_count = 1;

//         // FIFO数据缓冲区结构体
//         struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };
//         struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };
//         struct bmi2_aux_fifo_data fifo_aux_data[BMI2_FIFO_AUX_FRAME_COUNT] = { { { 0 } } };
//         struct bmm150_mag_data mag_data = {0};

//         // 等待传感器开始产生数据并填充FIFO
//         ESP_LOGI(TAG, "Waiting for FIFO to accumulate data...");
//         vTaskDelay(pdMS_TO_TICKS(1000)); // 等待1秒让传感器开始工作

//         while (try_count <= 10) {  // 给FIFO足够时间填满，最多尝试10次
//             /* Read FIFO data on interrupt. */
//             rslt = bmi2_get_int_status(&int_status, &_bmi270_dev);
//             print_bmi2_api_error(rslt);

//             if ((rslt == BMI2_OK) && (int_status & BMI2_FFULL_INT_STATUS_MASK)) {
//                 ESP_LOGI(TAG, "FIFO processing iteration : %d", try_count);

//                 accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
//                 gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;
//                 aux_frame_length = BMI2_FIFO_AUX_FRAME_COUNT;

//                 rslt = bmi2_get_fifo_length(&fifo_length, &_bmi270_dev);
//                 print_bmi2_api_error(rslt);

//                 /* Updating FIFO length to be read based on available length and dummy byte updation */
//                 fifoframe.length = fifo_length + _bmi270_dev.dummy_byte;

//                 ESP_LOGI(TAG, "FIFO data bytes available : %d", fifo_length);
//                 ESP_LOGI(TAG, "FIFO data bytes requested : %d", fifoframe.length);

//                 /* Read FIFO data. */
//                 rslt = bmi2_read_fifo_data(&fifoframe, &_bmi270_dev);
//                 print_bmi2_api_error(rslt);

//                 /* Read FIFO data on interrupt. */
//                 rslt = bmi2_get_int_status(&int_status, &_bmi270_dev);
//                 print_bmi2_api_error(rslt);

//                 if (rslt == BMI2_OK) {
//                     ESP_LOGI(TAG, "FIFO accel frames requested : %d", accel_frame_length);

//                     /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
//                     rslt = bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &_bmi270_dev);
//                     ESP_LOGI(TAG, "FIFO accel frames extracted : %d", accel_frame_length);

//                     ESP_LOGI(TAG, "FIFO gyro frames requested : %d", gyro_frame_length);

//                     /* Parse the FIFO data to extract gyro data from the FIFO buffer. */
//                     (void)bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &_bmi270_dev);
//                     ESP_LOGI(TAG, "FIFO gyro frames extracted : %d", gyro_frame_length);

//                     ESP_LOGI(TAG, "FIFO aux frames requested : %d", aux_frame_length);

//                     /* Parse the FIFO data to extract aux data from the FIFO buffer. */
//                     (void)bmi2_extract_aux(fifo_aux_data, &aux_frame_length, &fifoframe, &_bmi270_dev);
//                     ESP_LOGI(TAG, "FIFO aux frames extracted : %d", aux_frame_length);

//                     ESP_LOGI(TAG, "Extracted accel frames");
//                     ESP_LOGI(TAG, "ACCEL_DATA, X, Y, Z");

//                     /* Print the parsed accelerometer data from the FIFO buffer. */
//                     for (index = 0; index < accel_frame_length && index < 5; index++) {  // 限制输出数量
//                         ESP_LOGI(TAG, "%d, %d, %d, %d",
//                                index,
//                                fifo_accel_data[index].x,
//                                fifo_accel_data[index].y,
//                                fifo_accel_data[index].z);
//                     }

//                     ESP_LOGI(TAG, "Extracted gyro frames");
//                     ESP_LOGI(TAG, "GYRO_DATA, X, Y, Z");

//                     /* Print the parsed gyro data from the FIFO buffer. */
//                     for (index = 0; index < gyro_frame_length && index < 5; index++) {  // 限制输出数量
//                         ESP_LOGI(TAG, "%d, %d, %d, %d",
//                                index,
//                                fifo_gyro_data[index].x,
//                                fifo_gyro_data[index].y,
//                                fifo_gyro_data[index].z);
//                     }

//                     ESP_LOGI(TAG, "Extracted AUX frames");
//                     ESP_LOGI(TAG, "AUX_DATA, Mag_uT_X, Mag_uT_Y, Mag_uT_Z");

//                     /* Print the parsed aux data from the FIFO buffer. */
//                     for (index = 0; index < aux_frame_length && index < 5; index++) {  // 限制输出数量
//                         /* Compensating the raw auxiliary data available from the BMM150 API. */
//                         rslt = bmm150_aux_mag_data(fifo_aux_data[index].data, &mag_data, &_bmm150_dev);
//                         print_bmm150_api_error(rslt);

//                         ESP_LOGI(TAG, "%d, %ld, %ld, %ld",
//                                index,
//                                (long int)mag_data.x,
//                                (long int)mag_data.y,
//                                (long int)mag_data.z);
//                     }

//                     try_count++;
//                 }
//             } else {
//                 ESP_LOGD(TAG, "FIFO not full yet (try %d/%d), waiting for more data...", try_count, 10);
//                 // 等待 FIFO 积累更多数据
//                 vTaskDelay(pdMS_TO_TICKS(500)); // 等待 500ms
//                 try_count++;
//             }
//         }
        
//         ESP_LOGI(TAG, "FIFO processing completed after %d attempts", try_count - 1);
//     } else {
//         ESP_LOGW(TAG, "Invalid BMM150 (Aux) sensor - Chip ID : 0x%x", _bmm150_dev.chip_id);
//     }


    // // 2. 使能辅助传感器
    // uint8_t sens_list = BMI2_AUX;
    // rslt = bmi2_sensor_enable(&sens_list, 1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGE(TAG, "Failed to enable BMM150 aux sensor: %d", rslt);
    //     print_bmi2_api_error(rslt);
    //     return rslt;
    // }

    // // 短暂延时等待AUX接口稳定
    // vTaskDelay(pdMS_TO_TICKS(20));

    // // 3. 先上电再读 Chip ID: 写 PWR_CTRL (0x4B) 再尝试读取 0x40，多次重试
    // uint8_t bmm150_power = 0x01; // Normal power on
    // rslt = bmi2_write_aux_man_mode(0x4B, &bmm150_power, 1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGE(TAG, "Failed to pre-power BMM150: %d", rslt);
    //     print_bmi2_api_error(rslt);
    //     return rslt;
    // }
    // vTaskDelay(pdMS_TO_TICKS(6)); // 上电稳定

    // uint8_t bmm150_chip_id = 0;
    // const uint8_t primary_addr = 0x10;
    // const uint8_t secondary_addr = 0x11; // 备用地址 (SDO/CS 连接差异)
    // bool id_ok = false;
    // for (int attempt = 0; attempt < 4 && !id_ok; ++attempt) {
    //     rslt = bmi2_read_aux_man_mode(0x40, &bmm150_chip_id, 1, &_bmi270_dev);
    //     if (rslt == BMI2_OK && bmm150_chip_id == 0x32) {
    //         id_ok = true;
    //         ESP_LOGI(TAG, "BMM150 Chip ID OK (0x32) after attempt %d @addr 0x%02X", attempt+1, aux_cfg.cfg.aux.i2c_device_addr);
    //         break;
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(5));
    //     ESP_LOGW(TAG, "BMM150 Chip ID read attempt %d failed: rslt=%d id=0x%02X", attempt+1, rslt, bmm150_chip_id);
    //     // 第2次失败后尝试备用地址
    //     if (attempt == 1 && aux_cfg.cfg.aux.i2c_device_addr == primary_addr) {
    //         ESP_LOGW(TAG, "Switching AUX to secondary address 0x%02X and retry", secondary_addr);
    //         aux_cfg.cfg.aux.i2c_device_addr = secondary_addr;
    //         int8_t set_ret = bmi2_set_sensor_config(&aux_cfg, 1, &_bmi270_dev);
    //         if (set_ret != BMI2_OK) {
    //             ESP_LOGW(TAG, "Switch to secondary addr set_config failed %d", set_ret);
    //         }
    //         // 重新enable一次（容错）
    //         uint8_t sens_tmp = BMI2_AUX; bmi2_sensor_enable(&sens_tmp, 1, &_bmi270_dev);
    //         vTaskDelay(pdMS_TO_TICKS(5));
    //     }
    // }
    // if (!id_ok) {
    //     ESP_LOGW(TAG, "BMM150 Chip ID not detected (last id=0x%02X) - abort AUX setup", bmm150_chip_id);
    //     return BMI2_E_DEV_NOT_FOUND;
    // }

    // // 4. (已上电) 再次确保模式寄存器可写
    // vTaskDelay(pdMS_TO_TICKS(4));

    // // 5. 设置重复次数 (顺序：重复次数应在进入连续测量模式前设置)
    // uint8_t bmm150_rep_xy = 0x01; // XY轴重复次数: 3次 (Low Power)
    // uint8_t bmm150_rep_z = 0x02;  // Z轴重复次数: 3次 (Low Power)

    // rslt = bmi2_write_aux_man_mode(0x51, &bmm150_rep_xy, 1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGW(TAG, "Set BMM150 REP_XY fail: %d", rslt);
    // }
    // rslt = bmi2_write_aux_man_mode(0x52, &bmm150_rep_z, 1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGW(TAG, "Set BMM150 REP_Z fail: %d", rslt);
    // }

    // // 6. 配置BMM150数据率和模式 (Normal continuous). 0x4C[2:0]=000 Normal.
    // // 为确保重复寄存器生效，稍作延时再写模式寄存器。
    // vTaskDelay(pdMS_TO_TICKS(4));
    // uint8_t bmm150_mode = 0x00; // Normal mode, low data rate(默认 10Hz)。后续如需提高速率需改写 op mode + preset。
    // rslt = bmi2_write_aux_man_mode(0x4C, &bmm150_mode, 1, &_bmi270_dev);
    // if (rslt != BMI2_OK) {
    //     ESP_LOGE(TAG, "Failed to set BMM150 operation mode: %d", rslt);
    //     print_bmi2_api_error(rslt);
    //     return rslt;
    // }
    // vTaskDelay(pdMS_TO_TICKS(6));

    // // 7. 重新验证BMM150连接 (仍在手动模式, 再次读取 ID 确认地址切换后稳定)
    // bmm150_chip_id = 0;
    // rslt = bmi2_read_aux_man_mode(0x40, &bmm150_chip_id, 1, &_bmi270_dev);
    // if (rslt == BMI2_OK) {
    //     ESP_LOGI(TAG, "BMM150 final verification - Chip ID: 0x%02X", bmm150_chip_id);
    // } else {
    //     ESP_LOGW(TAG, "BMM150 final verification failed: %d", rslt);
    // }

    // // 8. 验证AUX传感器配置 (manual_en 应为 ENABLE)
    // struct bmi2_sens_config verify_aux_cfg;
    // verify_aux_cfg.type = BMI2_AUX;
    // rslt = bmi2_get_sensor_config(&verify_aux_cfg, 1, &_bmi270_dev);
    // if (rslt == BMI2_OK) {
    // ESP_LOGI(TAG, "AUX config verification - ODR: %d, enabled: %d, addr: 0x%02X, manual_en: %d", 
    //      verify_aux_cfg.cfg.aux.odr, 
    //      verify_aux_cfg.cfg.aux.aux_en,
    //      verify_aux_cfg.cfg.aux.i2c_device_addr,
    //      verify_aux_cfg.cfg.aux.manual_en);
    // }

    // // 8.1 读取内部上拉配置寄存器 0x68
    // uint8_t trim_reg = 0;
    // int8_t trim_rslt = bmi2_get_regs(BMI2_AUX_IF_TRIM, &trim_reg, 1, &_bmi270_dev);
    // if (trim_rslt == BMI2_OK) {
    //     ESP_LOGI(TAG, "AUX_IF_TRIM(0x68)=0x%02X (expect 0x02 for 10k)", trim_reg);
    // } else {
    //     ESP_LOGW(TAG, "Read AUX_IF_TRIM failed %d", trim_rslt);
    // }

    // // 8.2 读取 BMM150 电源/模式寄存器
    // uint8_t dbg_buf[2] = {0};
    // if (bmi2_read_aux_man_mode(0x4B, &dbg_buf[0], 1, &_bmi270_dev) == BMI2_OK &&
    //     bmi2_read_aux_man_mode(0x4C, &dbg_buf[1], 1, &_bmi270_dev) == BMI2_OK) {
    //     ESP_LOGI(TAG, "BMM150 PWR_CTRL(0x4B)=0x%02X, OPR_MODE(0x4C)=0x%02X", dbg_buf[0], dbg_buf[1]);
    // }

    // // 9. 检查传感器启用状态
    // uint8_t sensor_status = 0;
    // rslt = bmi2_get_internal_status(&sensor_status, &_bmi270_dev);
    // if (rslt == BMI2_OK) {
    //     ESP_LOGI(TAG, "BMI270 internal status: 0x%02X", sensor_status);
    // }

    // // 10. 退出手动模式: 关闭 manual_en 让 BMI270 自动采集 aux_data
    // rslt = bmi2_get_sensor_config(&verify_aux_cfg, 1, &_bmi270_dev);
    // if (rslt == BMI2_OK) {
    //     if (verify_aux_cfg.cfg.aux.manual_en == BMI2_ENABLE) {
    //         verify_aux_cfg.cfg.aux.manual_en = BMI2_DISABLE;
    //         rslt = bmi2_set_sensor_config(&verify_aux_cfg, 1, &_bmi270_dev);
    //         if (rslt == BMI2_OK) {
    //             ESP_LOGI(TAG, "AUX manual mode -> data mode (manual_en DISABLE)");
    //         } else {
    //             ESP_LOGW(TAG, "Disable manual_en failed: %d", rslt);
    //         }
    //     }
    // } else {
    //     ESP_LOGW(TAG, "Re-get AUX config before disabling manual_en failed: %d", rslt);
    // }

    // // 最终延时确保配置稳定
    // vTaskDelay(pdMS_TO_TICKS(30));

    // // 11. 映射数据就绪中断（内部刷新 DRDY 标志）
    // int8_t map_rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &_bmi270_dev);
    // if (map_rslt != BMI2_OK) {
    //     ESP_LOGW(TAG, "Map data ready int failed: %d", map_rslt);
    // }

    // // 12. 轮询等待首次 AUX DRDY（最多200ms）
    // bool drdy_ok = false;
    // for (int i = 0; i < 20; ++i) {
    //     struct bmi2_sens_data tmp = {0};
    //     if (bmi2_get_sensor_data(&tmp, &_bmi270_dev) == BMI2_OK) {
    //         if (tmp.status & BMI2_DRDY_AUX) { drdy_ok = true; break; }
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // ESP_LOGI(TAG, "AUX auto mode DRDY %s", drdy_ok ? "READY" : "TIMEOUT");
    // if (!drdy_ok) {
    //     ESP_LOGW(TAG, "AUX DRDY not seen; fallback path will be used until DRDY appears");
    // }

    // ESP_LOGI(TAG, "BMM150 magnetometer configured successfully (25Hz ODR, Low Power preset)");
    return BMI2_OK;
}

/**
 * @brief 读取BMM150磁力计数据
 * @param mag_data 磁力计数据结构指针
 * @return 0: 成功, 负值: 错误码
 */
int bmi270_tools::read_magnetometer_data(mag_data_t* mag_data) {
    if (!_initialized || !mag_data) {
        ESP_LOGE(TAG, "BMI270 not initialized or invalid parameter");
        return BMI2_E_NULL_PTR;
    }

    int8_t rslt = BMI2_OK;
    struct bmi2_sens_data sensor_data = {0};
    
    // 读取辅助传感器数据
    rslt = bmi2_get_sensor_data(&sensor_data, &_bmi270_dev);
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to read BMM150 data: %d", rslt);
        print_bmi2_api_error(rslt);
        mag_data->valid = false;
        return rslt;
    }

    // 添加详细的状态调试信息
    ESP_LOGI(TAG, "BMM150 status register: 0x%02X, DRDY_AUX: %s", 
             sensor_data.status, 
             (sensor_data.status & BMI2_DRDY_AUX) ? "READY" : "NOT_READY");

    // 检查辅助传感器数据是否准备就绪
    if (sensor_data.status & BMI2_DRDY_AUX) {
        // 输出原始aux_data以供调试
        ESP_LOGI(TAG, "BMM150 raw aux_data: [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
                 sensor_data.aux_data[0], sensor_data.aux_data[1], sensor_data.aux_data[2], sensor_data.aux_data[3],
                 sensor_data.aux_data[4], sensor_data.aux_data[5], sensor_data.aux_data[6], sensor_data.aux_data[7]);

        // BMM150数据格式: 前6字节为磁力计数据 (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
        // 检查数据有效性 - BMM150的状态字节通常在第7字节
        uint8_t bmm150_status = sensor_data.aux_data[6];
        ESP_LOGI(TAG, "BMM150 internal status: 0x%02X", bmm150_status);

        // 组合16位数据
        int16_t raw_x = (int16_t)((sensor_data.aux_data[1] << 8) | sensor_data.aux_data[0]);
        int16_t raw_y = (int16_t)((sensor_data.aux_data[3] << 8) | sensor_data.aux_data[2]);
        int16_t raw_z = (int16_t)((sensor_data.aux_data[5] << 8) | sensor_data.aux_data[4]);

        ESP_LOGI(TAG, "BMM150 raw values: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);

        // 移除LSB中的状态位 (BMM150数据格式)
        // X和Y轴: 13位有效数据 (bit 15-3)
        // Z轴: 15位有效数据 (bit 15-1)
        raw_x = raw_x >> 3;
        raw_y = raw_y >> 3;
        raw_z = raw_z >> 1;

        // 检查溢出标志 (BMM150 特定)
        bool overflow_x = (sensor_data.aux_data[0] & 0x01) != 0;
        bool overflow_y = (sensor_data.aux_data[2] & 0x01) != 0;
        bool overflow_z = (sensor_data.aux_data[4] & 0x02) != 0;

        if (overflow_x || overflow_y || overflow_z) {
            ESP_LOGW(TAG, "BMM150 data overflow detected: X=%d Y=%d Z=%d", overflow_x, overflow_y, overflow_z);
        }

        // 转换为微特斯拉 (uT)
        // BMM150默认量程: ±1300uT
        // 分辨率约为: ±1300uT / 4096 ≈ 0.32uT/LSB (对于13位数据)
        // Z轴分辨率约为: ±2500uT / 16384 ≈ 0.15uT/LSB (对于15位数据)
        mag_data->x = (int32_t)(raw_x * 0.32f);
        mag_data->y = (int32_t)(raw_y * 0.32f);
        mag_data->z = (int32_t)(raw_z * 0.15f);
        mag_data->valid = true;

        ESP_LOGI(TAG, "BMM150 data: X=%ld uT, Y=%ld uT, Z=%ld uT", 
                 mag_data->x, mag_data->y, mag_data->z);
    } 
    // else {
    //     // 数据不就绪 -> 回退手动读并解析一次，可提供近似值
    //     ESP_LOGI(TAG, "BMM150 data not ready, status=0x%02X -> fallback manual probe", sensor_data.status);
    //     mag_data->valid = false;
    //     struct bmi2_sens_config aux_cfg = {0};
    //     aux_cfg.type = BMI2_AUX;
    //     if (bmi2_get_sensor_config(&aux_cfg, 1, &_bmi270_dev) == BMI2_OK) {
    //         bool need_restore = false;
    //         if (aux_cfg.cfg.aux.manual_en == BMI2_DISABLE) {
    //             aux_cfg.cfg.aux.manual_en = BMI2_ENABLE;
    //             if (bmi2_set_sensor_config(&aux_cfg, 1, &_bmi270_dev) == BMI2_OK) {
    //                 need_restore = true;
    //                 vTaskDelay(pdMS_TO_TICKS(2));
    //             } else {
    //                 ESP_LOGW(TAG, "Enable manual for fallback failed");
    //             }
    //         }
    //         uint8_t buf[8] = {0};
    //         if (bmi2_read_aux_man_mode(0x42, buf, 8, &_bmi270_dev) == BMI2_OK) {
    //             ESP_LOGI(TAG, "Manual aux block: [%02X %02X %02X %02X %02X %02X %02X %02X]", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    //             auto se = [](int32_t v,int bits){int32_t m=1<<(bits-1);return (v^m)-m;};
    //             int32_t x13 = se(((int32_t)buf[1]<<5)|(buf[0]>>3),13);
    //             int32_t y13 = se(((int32_t)buf[3]<<5)|(buf[2]>>3),13);
    //             int32_t z15 = se(((int32_t)buf[5]<<7)|(buf[4]>>1),15);
    //             mag_data->x = (int32_t)(x13 * 3 / 10); // ≈0.3uT/LSB
    //             mag_data->y = (int32_t)(y13 * 3 / 10);
    //             mag_data->z = (int32_t)(z15 * 15 / 100); // 0.15uT/LSB
    //             mag_data->valid = true; // 标记手动读取值
    //             ESP_LOGI(TAG, "Fallback mag approx(uT): X=%ld Y=%ld Z=%ld", mag_data->x, mag_data->y, mag_data->z);
    //         } else {
    //             ESP_LOGW(TAG, "Manual aux block read failed");
    //         }
    //         if (need_restore) {
    //             aux_cfg.cfg.aux.manual_en = BMI2_DISABLE;
    //             if (bmi2_set_sensor_config(&aux_cfg, 1, &_bmi270_dev) != BMI2_OK) {
    //                 ESP_LOGW(TAG, "Restore manual_en disable failed");
    //             }
    //         }
    //     }
    // }

    return BMI2_OK;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float  bmi270_tools::lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

// === 高级融合功能实现 ===

esp_err_t bmi270_tools::get_quaternion_data(quaternion_data_t &quat_data)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 NOT initialized");
        quat_data.valid = false;
        return ESP_FAIL;
    }

    // 读取传感器数据
    sensor_data_t sensor_data;
    esp_err_t rslt = get_sensor_data(sensor_data);
    
    if (rslt != ESP_OK || !sensor_data.acc_valid) {
        ESP_LOGE(TAG, "Failed to read sensor data for quaternion calculation");
        quat_data.valid = false;
        return ESP_FAIL;
    }

    // 获取加速度计数据（已经是物理单位 m/s²）
    float ax = sensor_data.acc_x / GRAVITY_EARTH;  // 转换为重力加速度单位
    float ay = sensor_data.acc_y / GRAVITY_EARTH;
    float az = sensor_data.acc_z / GRAVITY_EARTH;
    
    // 检查加速度计数据有效性
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        ESP_LOGW(TAG, "Invalid accelerometer data, cannot calculate quaternion");
        quat_data.valid = false;
        return ESP_FAIL;
    }
    
    // 归一化加速度向量
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        ESP_LOGW(TAG, "Accelerometer norm too small, cannot calculate quaternion");
        quat_data.valid = false;
        return ESP_FAIL;
    }
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // 使用简化的倾斜补偿算法计算四元数
    // 假设设备主要受重力影响，计算设备相对于重力向量的旋转
    
    // 计算 Roll 和 Pitch 角度
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    
    // 将欧拉角转换为四元数 (假设 Yaw = 0)
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = 1.0f;  // cos(yaw/2) = cos(0) = 1
    float sy = 0.0f;  // sin(yaw/2) = sin(0) = 0
    
    // 四元数计算：q = qyaw * qpitch * qroll
    quat_data.w = cr * cp * cy + sr * sp * sy;
    quat_data.x = sr * cp * cy - cr * sp * sy;
    quat_data.y = cr * sp * cy + sr * cp * sy;
    quat_data.z = cr * cp * sy - sr * sp * cy;
    
    // 归一化四元数
    float q_norm = sqrtf(quat_data.w * quat_data.w + quat_data.x * quat_data.x + 
                        quat_data.y * quat_data.y + quat_data.z * quat_data.z);
    
    if (q_norm < 1e-6f) {
        ESP_LOGW(TAG, "Quaternion norm too small");
        quat_data.valid = false;
        return ESP_FAIL;
    }
    
    quat_data.w /= q_norm;
    quat_data.x /= q_norm;
    quat_data.y /= q_norm;
    quat_data.z /= q_norm;
    quat_data.valid = true;
    
    ESP_LOGD(TAG, "Calculated quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
             quat_data.w, quat_data.x, quat_data.y, quat_data.z);
    
    return ESP_OK;
}

bmi270_tools::absolute_orientation_t bmi270_tools::quaternion_to_xyz_rotation(const quaternion_data_t &quat)
{
    absolute_orientation_t xyz_rotation;
    
    if (!quat.valid) {
        xyz_rotation.x_rotation = xyz_rotation.y_rotation = xyz_rotation.z_rotation = 0.0f;
        xyz_rotation.valid = false;
        return xyz_rotation;
    }
    
    float x = quat.x, y = quat.y, z = quat.z, w = quat.w;
    
    // 改进的四元数到欧拉角转换，增加数值稳定性
    
    // X轴旋转角度 (Roll) - 使用更稳定的计算方法
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    xyz_rotation.x_rotation = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;
    
    // Y轴旋转角度 (Pitch) - 防止反正弦函数的数值不稳定
    float sinp = 2.0f * (w * y - z * x);
    // 限制sinp的范围，防止asin输入超出[-1,1]
    if (sinp >= 1.0f) {
        xyz_rotation.y_rotation = 90.0f;  // 使用90度而不是PI/2来避免精度问题
    } else if (sinp <= -1.0f) {
        xyz_rotation.y_rotation = -90.0f;
    } else {
        xyz_rotation.y_rotation = asinf(sinp) * 180.0f / (float)M_PI;
    }
    
    // Z轴旋转角度 (Yaw) - 使用更稳定的计算方法
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    xyz_rotation.z_rotation = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;
    
    xyz_rotation.valid = true;
    return xyz_rotation;
}

esp_err_t bmi270_tools::get_absolute_orientation(absolute_orientation_t &orientation)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        orientation.valid = false;
        return ESP_FAIL;
    }

    // 读取原始传感器数据
    sensor_data_t raw_data;
    if (get_sensor_data(raw_data) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        orientation.valid = false;
        return ESP_FAIL;
    }

    // 验证传感器数据有效性
    if (!validate_sensor_data(raw_data)) {
        ESP_LOGW(TAG, "Invalid sensor data detected");
        // 返回上次有效结果
        if (_last_orientation.valid) {
            orientation = _last_orientation;
            return ESP_OK;
        } else {
            orientation.valid = false;
            return ESP_FAIL;
        }
    }

    // 应用传感器校准
    sensor_data_t calibrated_data = raw_data;
    apply_sensor_calibration(calibrated_data);

    // 更新运动检测器
    update_motion_detector(calibrated_data);
    
    // 更新陀螺仪零偏校准
    update_gyro_bias_calibration(calibrated_data);
    
    // 应用陀螺仪零偏校正
    apply_gyro_bias_correction(calibrated_data);

    // 应用低通滤波器平滑传感器数据
    apply_low_pass_filter_3d(_filters.acc_filter, calibrated_data.acc_x, calibrated_data.acc_y, calibrated_data.acc_z);
    apply_low_pass_filter_3d(_filters.gyro_filter, calibrated_data.gyr_x, calibrated_data.gyr_y, calibrated_data.gyr_z);
    
    bool mag_available = is_magnetometer_enabled() && calibrated_data.mag_valid;
    if (mag_available) {
        float mag_x = (float)calibrated_data.mag_x;
        float mag_y = (float)calibrated_data.mag_y;
        float mag_z = (float)calibrated_data.mag_z;
        apply_low_pass_filter_3d(_filters.mag_filter, mag_x, mag_y, mag_z);
        calibrated_data.mag_x = (int16_t)mag_x;
        calibrated_data.mag_y = (int16_t)mag_y;
        calibrated_data.mag_z = (int16_t)mag_z;
    }

    // 检查是否静止
    bool is_stationary = is_device_stationary();
    
    // 声明在函数作用域内使用的变量
    float gyro_norm_deg = 0.0f;  // 用于日志输出
    
    // 使用Madgwick融合算法（9DOF或6DOF）
    if (calibrated_data.gyr_valid) {
        // 初始化Madgwick滤波器
        if (!_madgwick_filter.initialized) {
            _madgwick_filter.beta = 0.1f;  // 降低初始增益
            _madgwick_filter.adaptive_gain = 0.1f;
            _madgwick_filter.q0 = 1.0f; 
            _madgwick_filter.q1 = 0.0f; 
            _madgwick_filter.q2 = 0.0f; 
            _madgwick_filter.q3 = 0.0f;
            _madgwick_filter.last_update_ms = esp_timer_get_time() / 1000;
            _madgwick_filter.mag_norm_ref = 50.0f;  // 假设地磁场强度约50uT
            _madgwick_filter.stable_count = 0;
            _madgwick_filter.initialized = true;
            ESP_LOGI(TAG, "Madgwick filter initialized with drift compensation");
        }

        // 计算时间差
        uint32_t now_ms = esp_timer_get_time() / 1000;
        float dt = (now_ms - _madgwick_filter.last_update_ms) / 1000.0f;
        _madgwick_filter.last_update_ms = now_ms;
        
        // 限制时间差范围，避免积分发散
        if (dt > 0.1f) dt = 0.01f;  // 如果间隔过长，使用默认值
        if (dt < 0.001f) dt = 0.001f;

        // 单位转换
        float gx = calibrated_data.gyr_x * (float)M_PI / 180.0f;  // 转换为弧度/秒
        float gy = calibrated_data.gyr_y * (float)M_PI / 180.0f;
        float gz = calibrated_data.gyr_z * (float)M_PI / 180.0f;
        float ax = calibrated_data.acc_x / GRAVITY_EARTH;  // 转换为重力单位
        float ay = calibrated_data.acc_y / GRAVITY_EARTH;
        float az = calibrated_data.acc_z / GRAVITY_EARTH;

        // 计算运动强度
        float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz) * 180.0f / (float)M_PI; // 转换为°/s
        float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
        gyro_norm_deg = gyro_norm; // 保存用于日志输出

        // 如果设备静止，大幅降低陀螺仪权重，增加加速度计权重
        if (is_stationary) {
            // 静止状态：几乎完全信任加速度计，忽略陀螺仪微小噪声
            gx *= 0.05f;  // 大幅降低陀螺仪输入
            gy *= 0.05f;
            gz *= 0.05f;
            _madgwick_filter.beta = 1.5f;  // 增加加速度计校正力度
            _madgwick_filter.stable_count++;
        } else {
            // 运动状态：正常融合，但根据运动强度调整
            float motion_intensity = gyro_norm;  // 使用陀螺仪模长作为运动强度指标
            if (motion_intensity < 10.0f) {
                // 轻微运动：较强的加速度计校正
                _madgwick_filter.beta = 0.5f;
            } else if (motion_intensity < 50.0f) {
                // 中等运动：适中的校正
                _madgwick_filter.beta = 0.3f;
            } else {
                // 剧烈运动：较弱的加速度计校正，更信任陀螺仪
                _madgwick_filter.beta = 0.1f;
            }
            _madgwick_filter.stable_count = 0;
        }

        // 计算自适应增益
        float mag_norm = 0.0f;
        
        if (mag_available) {
            float mx = (float)calibrated_data.mag_x;
            float my = (float)calibrated_data.mag_y;
            float mz = (float)calibrated_data.mag_z;
            mag_norm = sqrtf(mx*mx + my*my + mz*mz);
            
            // 磁力计数据归一化
            if (mag_norm > 1e-3f) {
                mx /= mag_norm; my /= mag_norm; mz /= mag_norm;
            }
            
            // 9DOF融合
            madgwick_update_9dof(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        } else {
            // 6DOF融合
            madgwick_update_6dof(gx, gy, gz, ax, ay, az, dt);
        }

        // 从四元数转换为欧拉角
        quaternion_data_t quat_data;
        quat_data.w = _madgwick_filter.q0;
        quat_data.x = _madgwick_filter.q1;
        quat_data.y = _madgwick_filter.q2;
        quat_data.z = _madgwick_filter.q3;
        quat_data.valid = true;
        
        // 应用四元数级别的滤波，减少欧拉角转换时的不稳定性
        apply_quaternion_filter(quat_data.w, quat_data.x, quat_data.y, quat_data.z, is_stationary);
        
        // 使用改进的稳定四元数到欧拉角转换（包含新的角度跟踪算法）
        orientation = quaternion_to_xyz_rotation_stable(quat_data);
        
        // 添加额外的平滑滤波（可选）
        if (_last_orientation.valid) {
            // 使用加权平均进行额外平滑
            float smooth_factor = is_stationary ? 0.9f : 0.7f;
            
            // 对每个轴单独处理，确保正确处理角度差
            orientation.x_rotation = smooth_angle_transition(
                orientation.x_rotation, 
                _last_orientation.x_rotation, 
                smooth_factor
            );
            orientation.y_rotation = smooth_angle_transition(
                orientation.y_rotation, 
                _last_orientation.y_rotation, 
                smooth_factor
            );
            orientation.z_rotation = smooth_angle_transition(
                orientation.z_rotation, 
                _last_orientation.z_rotation, 
                smooth_factor
            );
        }
        
        // 如果长时间静止，执行漂移重置
        if (is_stationary && _madgwick_filter.stable_count > 100) {  // 静止超过100个周期
            ESP_LOGI(TAG, "Device stationary for extended period, resetting orientation drift");
            reset_orientation_drift();
            _madgwick_filter.stable_count = 0;
        }
        
        // 检查姿态变化的合理性，避免突跳
        if (_last_orientation.valid) {
            float roll_diff = fabsf(angle_difference(orientation.x_rotation, _last_orientation.x_rotation));
            float pitch_diff = fabsf(angle_difference(orientation.y_rotation, _last_orientation.y_rotation));
            float yaw_diff = fabsf(angle_difference(orientation.z_rotation, _last_orientation.z_rotation));
            
            // 动态调整最大变化阈值
            float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz) * 180.0f / (float)M_PI; // 转换为°/s
            float base_threshold = is_stationary ? 8.0f : 40.0f; // 提高基础阈值
            float dynamic_threshold = base_threshold + gyro_norm * 0.3f; // 降低运动敏感度
            
            // 限制动态阈值范围
            if (dynamic_threshold > 80.0f) dynamic_threshold = 80.0f;
            if (dynamic_threshold < 8.0f) dynamic_threshold = 8.0f;
            
            // 只有在极端情况下才认为是异常（提高异常检测阈值）
            bool extreme_change = (roll_diff > 120.0f || pitch_diff > 120.0f || yaw_diff > 120.0f);
            
            if (extreme_change) {
                ESP_LOGW(TAG, "Extreme angle change detected: roll=%.1f°, pitch=%.1f°, yaw=%.1f°", 
                         roll_diff, pitch_diff, yaw_diff);
                
                // 只在极端情况下才进行渐变混合
                float blend_factor = is_stationary ? 0.2f : 0.7f;
                orientation.x_rotation = _last_orientation.x_rotation + 
                    angle_difference(orientation.x_rotation, _last_orientation.x_rotation) * blend_factor;
                orientation.y_rotation = _last_orientation.y_rotation + 
                    angle_difference(orientation.y_rotation, _last_orientation.y_rotation) * blend_factor;
                orientation.z_rotation = _last_orientation.z_rotation + 
                    angle_difference(orientation.z_rotation, _last_orientation.z_rotation) * blend_factor;
            } else if (roll_diff > dynamic_threshold || pitch_diff > dynamic_threshold || yaw_diff > dynamic_threshold) {
                // 记录大变化但不进行强制混合，允许正常的快速运动
                ESP_LOGD(TAG, "Large angle change: roll=%.1f°, pitch=%.1f°, yaw=%.1f° (threshold=%.1f°)", 
                         roll_diff, pitch_diff, yaw_diff, dynamic_threshold);
            }
        }
        
        // 应用角度低通滤波（静止时更强的滤波）
        float angle_alpha = is_stationary ? 0.95f : 0.8f;
        apply_low_pass_filter_1d(_filters.angle_filter.x, orientation.x_rotation, angle_alpha);
        apply_low_pass_filter_1d(_filters.angle_filter.y, orientation.y_rotation, angle_alpha);
        apply_low_pass_filter_1d(_filters.angle_filter.z, orientation.z_rotation, angle_alpha);
        
        orientation.x_rotation = _filters.angle_filter.x;
        orientation.y_rotation = _filters.angle_filter.y;
        orientation.z_rotation = _filters.angle_filter.z;
        
    } else {
        // 陀螺仪不可用，使用静态解算（仅限Roll和Pitch）
        ESP_LOGW(TAG, "Gyroscope not available, using static calculation");
        
        float ax = calibrated_data.acc_x / GRAVITY_EARTH;
        float ay = calibrated_data.acc_y / GRAVITY_EARTH;
        float az = calibrated_data.acc_z / GRAVITY_EARTH;
        float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
        
        if (acc_norm > 0.5f && acc_norm < 1.5f) { // 检查是否接近1g，确保是重力主导
            // 归一化
            ax /= acc_norm; ay /= acc_norm; az /= acc_norm;
            
            // 静态解算Roll和Pitch
            orientation.x_rotation = atan2f(ay, az) * 180.0f / (float)M_PI;
            orientation.y_rotation = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;
            
            // Yaw计算（需要磁力计）
            if (mag_available) {
                float mx = (float)calibrated_data.mag_x;
                float my = (float)calibrated_data.mag_y;
                float mz = (float)calibrated_data.mag_z;
                float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
                
                if (mag_norm > 1e-3f) {
                    mx /= mag_norm; my /= mag_norm; mz /= mag_norm;
                    
                    // 倾斜补偿的磁力计Yaw计算
                    float roll_rad = orientation.x_rotation * (float)M_PI / 180.0f;
                    float pitch_rad = orientation.y_rotation * (float)M_PI / 180.0f;
                    float cos_r = cosf(roll_rad), sin_r = sinf(roll_rad);
                    float cos_p = cosf(pitch_rad), sin_p = sinf(pitch_rad);
                    
                    float mx_comp = mx * cos_p + mz * sin_p;
                    float my_comp = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p;
                    
                    orientation.z_rotation = atan2f(-my_comp, mx_comp) * 180.0f / (float)M_PI;
                } else {
                    orientation.z_rotation = 0.0f;
                }
            } else {
                orientation.z_rotation = 0.0f;
            }
            
            // 应用角度滤波
            if (_filters.angle_filter.initialized) {
                apply_low_pass_filter_1d(_filters.angle_filter.x, orientation.x_rotation, 0.9f);
                apply_low_pass_filter_1d(_filters.angle_filter.y, orientation.y_rotation, 0.9f);
                apply_low_pass_filter_1d(_filters.angle_filter.z, orientation.z_rotation, 0.9f);
                
                orientation.x_rotation = _filters.angle_filter.x;
                orientation.y_rotation = _filters.angle_filter.y;
                orientation.z_rotation = _filters.angle_filter.z;
            }
        } else {
            ESP_LOGW(TAG, "Accelerometer data suggests non-gravitational acceleration (norm=%.3f)", acc_norm);
            if (_last_orientation.valid) {
                orientation = _last_orientation;
                return ESP_OK;
            } else {
                orientation.valid = false;
                return ESP_FAIL;
            }
        }
    }

    // 角度归一化
    normalize_angles(orientation);
    orientation.valid = true;
    
    // 保存有效结果
    _last_orientation = orientation;
    
    ESP_LOGD(TAG, "Orientation: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f° (stationary=%s, beta=%.3f, motion=%.1f°/s)", 
             orientation.x_rotation, orientation.y_rotation, orientation.z_rotation, 
             is_stationary ? "YES" : "NO", _madgwick_filter.beta, gyro_norm_deg);
    
    return ESP_OK;
}

// === 高精度Madgwick姿态融合算法实现 ===

esp_err_t bmi270_tools::get_quaternion_madgwick(quaternion_data_t &quat_data, bool use_magnetometer)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        quat_data.valid = false;
        return ESP_FAIL;
    }

    // 读取传感器数据
    sensor_data_t sensor_data;
    esp_err_t rslt = get_sensor_data(sensor_data);
    
    if (rslt != ESP_OK || !sensor_data.acc_valid) {
        ESP_LOGE(TAG, "Failed to read sensor data for Madgwick filter");
        quat_data.valid = false;
        return ESP_FAIL;
    }

    // 初始化Madgwick滤波器
    if (!_madgwick_filter.initialized) {
        _madgwick_filter.beta = 0.5f;          // 默认增益
        _madgwick_filter.q0 = 1.0f;            // 初始四元数
        _madgwick_filter.q1 = 0.0f;
        _madgwick_filter.q2 = 0.0f;
        _madgwick_filter.q3 = 0.0f;
        _madgwick_filter.last_update_ms = esp_timer_get_time() / 1000;
        _madgwick_filter.initialized = true;
        ESP_LOGI(TAG, "Madgwick filter initialized");
    }

    // 计算时间差
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    float dt = (current_time_ms - _madgwick_filter.last_update_ms) / 1000.0f;
    _madgwick_filter.last_update_ms = current_time_ms;

    // 限制时间差范围
    if (dt > 0.1f) dt = 0.01f;  // 限制最大时间差为100ms
    if (dt < 0.001f) dt = 0.001f;  // 限制最小时间差为1ms

    // 应用传感器校准
    sensor_data_t calibrated_data = sensor_data;
    apply_sensor_calibration(calibrated_data);

    // 转换数据单位
    float gx = calibrated_data.gyr_x * M_PI / 180.0f;  // 转换为弧度/秒
    float gy = calibrated_data.gyr_y * M_PI / 180.0f;
    float gz = calibrated_data.gyr_z * M_PI / 180.0f;
    
    float ax = calibrated_data.acc_x / GRAVITY_EARTH;   // 转换为重力单位
    float ay = calibrated_data.acc_y / GRAVITY_EARTH;
    float az = calibrated_data.acc_z / GRAVITY_EARTH;

    // 根据是否使用磁力计选择算法
    if (use_magnetometer && is_magnetometer_enabled() && calibrated_data.mag_valid) {
        // 9轴融合
        float mx = (float)calibrated_data.mag_x;
        float my = (float)calibrated_data.mag_y;
        float mz = (float)calibrated_data.mag_z;
        
        madgwick_update_9dof(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        ESP_LOGD(TAG, "Using 9-DOF Madgwick fusion");
    } else {
        // 6轴融合
        madgwick_update_6dof(gx, gy, gz, ax, ay, az, dt);
        ESP_LOGD(TAG, "Using 6-DOF Madgwick fusion");
    }

    // 输出四元数
    quat_data.w = _madgwick_filter.q0;
    quat_data.x = _madgwick_filter.q1;
    quat_data.y = _madgwick_filter.q2;
    quat_data.z = _madgwick_filter.q3;
    quat_data.valid = true;

    ESP_LOGD(TAG, "Madgwick quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f", 
             quat_data.w, quat_data.x, quat_data.y, quat_data.z);

    return ESP_OK;
}

void bmi270_tools::madgwick_update_6dof(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 获取当前四元数
    float q0 = _madgwick_filter.q0;
    float q1 = _madgwick_filter.q1;
    float q2 = _madgwick_filter.q2;
    float q3 = _madgwick_filter.q3;

    // 四元数的导数
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 如果加速度计有有效数据，进行梯度下降校正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // 归一化加速度计数据
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 辅助变量
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 梯度下降算法，目标函数
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // 归一化步长
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈
        qDot1 -= _madgwick_filter.beta * s0;
        qDot2 -= _madgwick_filter.beta * s1;
        qDot3 -= _madgwick_filter.beta * s2;
        qDot4 -= _madgwick_filter.beta * s3;
    }

    // 积分四元数导数
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 归一化四元数
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    _madgwick_filter.q0 = q0 * recipNorm;
    _madgwick_filter.q1 = q1 * recipNorm;
    _madgwick_filter.q2 = q2 * recipNorm;
    _madgwick_filter.q3 = q3 * recipNorm;
}

void bmi270_tools::madgwick_update_9dof(float gx, float gy, float gz, float ax, float ay, float az, 
                                        float mx, float my, float mz, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // 获取当前四元数
    float q0 = _madgwick_filter.q0;
    float q1 = _madgwick_filter.q1;
    float q2 = _madgwick_filter.q2;
    float q3 = _madgwick_filter.q3;

    // 四元数的导数
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 如果有有效的加速度计和磁力计数据
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        
        // 归一化加速度计
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 归一化磁力计
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 参考方向
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 梯度下降算法
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        // 归一化
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈
        qDot1 -= _madgwick_filter.beta * s0;
        qDot2 -= _madgwick_filter.beta * s1;
        qDot3 -= _madgwick_filter.beta * s2;
        qDot4 -= _madgwick_filter.beta * s3;
    }

    // 积分四元数导数
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 归一化四元数
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    _madgwick_filter.q0 = q0 * recipNorm;
    _madgwick_filter.q1 = q1 * recipNorm;
    _madgwick_filter.q2 = q2 * recipNorm;
    _madgwick_filter.q3 = q3 * recipNorm;
}

float bmi270_tools::inv_sqrt(float x)
{
    // 快速平方根倒数算法 (Quake III)
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void bmi270_tools::set_madgwick_beta(float beta)
{
    if (beta >= 0.1f && beta <= 2.0f) {
        _madgwick_filter.beta = beta;
        ESP_LOGI(TAG, "Madgwick filter beta set to %.3f", beta);
    } else {
        ESP_LOGW(TAG, "Invalid beta value %.3f, valid range is 0.1-2.0", beta);
    }
}

void bmi270_tools::reset_attitude_filter()
{
    _madgwick_filter.q0 = 1.0f;
    _madgwick_filter.q1 = 0.0f;
    _madgwick_filter.q2 = 0.0f;
    _madgwick_filter.q3 = 0.0f;
    _madgwick_filter.last_update_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Attitude filter reset");
}

// === 传感器校准实现 ===

esp_err_t bmi270_tools::calibrate_sensors(uint32_t calibration_time_ms, uint32_t sample_count)
{
    ESP_LOGI(TAG, "Starting sensor calibration (time: %ldms, samples: %ld)", calibration_time_ms, sample_count);
    
    esp_err_t result = ESP_OK;
    
    // 校准陀螺仪
    if (calibrate_gyroscope(sample_count) != ESP_OK) {
        ESP_LOGW(TAG, "Gyroscope calibration failed");
        result = ESP_FAIL;
    }
    
    // 校准加速度计
    if (calibrate_accelerometer(sample_count) != ESP_OK) {
        ESP_LOGW(TAG, "Accelerometer calibration failed");
        result = ESP_FAIL;
    }
    
    // 校准磁力计 (如果启用)
    if (is_magnetometer_enabled()) {
        if (calibrate_magnetometer(sample_count) != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer calibration failed");
            result = ESP_FAIL;
        }
    }
    
    if (result == ESP_OK) {
        _calibration.calibrated = true;
        ESP_LOGI(TAG, "Sensor calibration completed successfully");
    } else {
        ESP_LOGW(TAG, "Sensor calibration completed with errors");
    }
    
    return result;
}

esp_err_t bmi270_tools::calibrate_gyroscope(uint32_t sample_count)
{
    ESP_LOGI(TAG, "Calibrating gyroscope... Keep device stationary");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t valid_samples = 0;
    
    for (uint32_t i = 0; i < sample_count; i++) {
        sensor_data_t data;
        if (get_sensor_data(data) == ESP_OK && data.gyr_valid) {
            sum_x += data.gyr_x;
            sum_y += data.gyr_y;
            sum_z += data.gyr_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms采样间隔
    }
    
    if (valid_samples < sample_count / 2) {
        ESP_LOGE(TAG, "Insufficient gyroscope samples for calibration");
        return ESP_FAIL;
    }
    
    _calibration.gyro_offset[0] = sum_x / valid_samples;
    _calibration.gyro_offset[1] = sum_y / valid_samples;
    _calibration.gyro_offset[2] = sum_z / valid_samples;
    
    ESP_LOGI(TAG, "Gyroscope bias: X=%.3f, Y=%.3f, Z=%.3f deg/s", 
             _calibration.gyro_offset[0], _calibration.gyro_offset[1], _calibration.gyro_offset[2]);
    
    return ESP_OK;
}

esp_err_t bmi270_tools::calibrate_accelerometer(uint32_t sample_count)
{
    ESP_LOGI(TAG, "Calibrating accelerometer... Keep device flat on level surface");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t valid_samples = 0;
    
    for (uint32_t i = 0; i < sample_count; i++) {
        sensor_data_t data;
        if (get_sensor_data(data) == ESP_OK && data.acc_valid) {
            sum_x += data.acc_x;
            sum_y += data.acc_y;
            sum_z += data.acc_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms采样间隔
    }
    
    if (valid_samples < sample_count / 2) {
        ESP_LOGE(TAG, "Insufficient accelerometer samples for calibration");
        return ESP_FAIL;
    }
    
    _calibration.acc_offset[0] = sum_x / valid_samples;
    _calibration.acc_offset[1] = sum_y / valid_samples;
    _calibration.acc_offset[2] = sum_z / valid_samples - GRAVITY_EARTH;  // Z轴减去重力
    
    // 简化的缩放校准 (假设各轴一致)
    _calibration.acc_scale[0] = 1.0f;
    _calibration.acc_scale[1] = 1.0f;
    _calibration.acc_scale[2] = 1.0f;
    
    ESP_LOGI(TAG, "Accelerometer bias: X=%.3f, Y=%.3f, Z=%.3f m/s²", 
             _calibration.acc_offset[0], _calibration.acc_offset[1], _calibration.acc_offset[2]);
    
    return ESP_OK;
}

esp_err_t bmi270_tools::calibrate_magnetometer(uint32_t sample_count)
{
    ESP_LOGI(TAG, "Calibrating magnetometer... Rotate device in figure-8 pattern");
    
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;
    uint32_t valid_samples = 0;
    
    for (uint32_t i = 0; i < sample_count; i++) {
        sensor_data_t data;
        if (get_sensor_data(data) == ESP_OK && data.mag_valid) {
            float mx = (float)data.mag_x;
            float my = (float)data.mag_y;
            float mz = (float)data.mag_z;
            
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;
            
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms采样间隔
    }
    
    if (valid_samples < sample_count / 2) {
        ESP_LOGE(TAG, "Insufficient magnetometer samples for calibration");
        return ESP_FAIL;
    }
    
    // 硬铁偏移校准
    _calibration.mag_offset[0] = (max_x + min_x) / 2.0f;
    _calibration.mag_offset[1] = (max_y + min_y) / 2.0f;
    _calibration.mag_offset[2] = (max_z + min_z) / 2.0f;
    
    // 简化的软铁校准 (对角矩阵)
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float range_z = max_z - min_z;
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    
    // 初始化为单位矩阵
    memset(_calibration.mag_scale, 0, sizeof(_calibration.mag_scale));
    _calibration.mag_scale[0][0] = avg_range / range_x;
    _calibration.mag_scale[1][1] = avg_range / range_y;
    _calibration.mag_scale[2][2] = avg_range / range_z;
    
    ESP_LOGI(TAG, "Magnetometer hard iron offset: X=%.1f, Y=%.1f, Z=%.1f uT", 
             _calibration.mag_offset[0], _calibration.mag_offset[1], _calibration.mag_offset[2]);
    ESP_LOGI(TAG, "Magnetometer scale factors: X=%.3f, Y=%.3f, Z=%.3f", 
             _calibration.mag_scale[0][0], _calibration.mag_scale[1][1], _calibration.mag_scale[2][2]);
    
    return ESP_OK;
}

void bmi270_tools::apply_sensor_calibration(sensor_data_t &data)
{
    if (!_calibration.calibrated) return;
    
    // 应用陀螺仪偏移校准
    if (data.gyr_valid) {
        data.gyr_x -= _calibration.gyro_offset[0];
        data.gyr_y -= _calibration.gyro_offset[1];
        data.gyr_z -= _calibration.gyro_offset[2];
    }
    
    // 应用加速度计偏移和缩放校准
    if (data.acc_valid) {
        data.acc_x = (data.acc_x - _calibration.acc_offset[0]) * _calibration.acc_scale[0];
        data.acc_y = (data.acc_y - _calibration.acc_offset[1]) * _calibration.acc_scale[1];
        data.acc_z = (data.acc_z - _calibration.acc_offset[2]) * _calibration.acc_scale[2];
    }
    
    // 应用磁力计硬铁和软铁校准
    if (data.mag_valid) {
        float mx = (float)data.mag_x - _calibration.mag_offset[0];
        float my = (float)data.mag_y - _calibration.mag_offset[1];
        float mz = (float)data.mag_z - _calibration.mag_offset[2];
        
        // 应用软铁校正矩阵
        float mx_cal = _calibration.mag_scale[0][0] * mx + _calibration.mag_scale[0][1] * my + _calibration.mag_scale[0][2] * mz;
        float my_cal = _calibration.mag_scale[1][0] * mx + _calibration.mag_scale[1][1] * my + _calibration.mag_scale[1][2] * mz;
        float mz_cal = _calibration.mag_scale[2][0] * mx + _calibration.mag_scale[2][1] * my + _calibration.mag_scale[2][2] * mz;
        
        data.mag_x = (int16_t)mx_cal;
        data.mag_y = (int16_t)my_cal;
        data.mag_z = (int16_t)mz_cal;
    }
}

// === 新增的辅助函数实现 ===

void bmi270_tools::apply_low_pass_filter_3d(low_pass_filter_t &filter, float &x, float &y, float &z)
{
    if (!filter.initialized) {
        filter.x = x;
        filter.y = y;
        filter.z = z;
        filter.alpha = 0.8f;  // 低通滤波系数，越小滤波越强
        filter.initialized = true;
    } else {
        filter.x = filter.alpha * x + (1.0f - filter.alpha) * filter.x;
        filter.y = filter.alpha * y + (1.0f - filter.alpha) * filter.y;
        filter.z = filter.alpha * z + (1.0f - filter.alpha) * filter.z;
        x = filter.x;
        y = filter.y;
        z = filter.z;
    }
}

void bmi270_tools::apply_low_pass_filter_1d(float &current, float new_value, float alpha)
{
    current = alpha * new_value + (1.0f - alpha) * current;
}

bool bmi270_tools::validate_sensor_data(const sensor_data_t &data)
{
    // 检查加速度计数据合理性
    if (data.acc_valid) {
        float acc_norm = sqrtf(data.acc_x * data.acc_x + data.acc_y * data.acc_y + data.acc_z * data.acc_z);
        // 加速度计范围检查：应该在0.2g到3g之间（考虑运动加速度）
        if (acc_norm < 2.0f || acc_norm > 30.0f) {
            ESP_LOGW(TAG, "Accelerometer data out of range: norm=%.2f m/s²", acc_norm);
            return false;
        }
    }
    
    // 检查陀螺仪数据合理性
    if (data.gyr_valid) {
        float gyro_norm = sqrtf(data.gyr_x * data.gyr_x + data.gyr_y * data.gyr_y + data.gyr_z * data.gyr_z);
        // 陀螺仪范围检查：不应超过2000度/秒
        if (gyro_norm > 2000.0f) {
            ESP_LOGW(TAG, "Gyroscope data out of range: norm=%.2f °/s", gyro_norm);
            return false;
        }
    }
    
    // 检查磁力计数据合理性
    if (data.mag_valid) {
        float mag_norm = sqrtf((float)data.mag_x * data.mag_x + (float)data.mag_y * data.mag_y + (float)data.mag_z * data.mag_z);
        // 磁力计范围检查：地磁场强度通常在20-70uT
        if (mag_norm < 10.0f || mag_norm > 200.0f) {
            ESP_LOGD(TAG, "Magnetometer data may be interfered: norm=%.1f uT", mag_norm);
            // 磁力计异常不直接返回false，因为可能受到干扰但其他数据仍有效
        }
    }
    
    return true;
}

float bmi270_tools::calculate_adaptive_gain(float acc_norm, float gyro_norm, float mag_norm)
{
    // 基础增益
    float base_gain = 0.2f;
    
    // 根据加速度计接近1g的程度调整增益
    float acc_confidence = 1.0f;
    if (acc_norm > 0.1f) {
        float acc_error = fabsf(acc_norm - 1.0f);
        acc_confidence = expf(-acc_error * 2.0f);  // 接近1g时置信度高
    }
    
    // 根据陀螺仪运动程度调整增益
    float gyro_confidence = 1.0f;
    if (gyro_norm > 0.1f) {
        // 运动越剧烈，对加速度计和磁力计的信任度越低
        gyro_confidence = expf(-gyro_norm / 50.0f);  // 50°/s作为参考
    }
    
    // 根据磁力计数据质量调整增益
    float mag_confidence = 1.0f;
    if (mag_norm > 0.1f) {
        // 检查磁场强度是否接近地磁场
        float mag_error = fabsf(mag_norm - 50.0f) / 50.0f;  // 假设地磁场50uT
        mag_confidence = expf(-mag_error * 1.0f);
    }
    
    // 综合置信度
    float total_confidence = acc_confidence * gyro_confidence * mag_confidence;
    
    // 动态调整增益：置信度高时使用较高增益，置信度低时使用较低增益
    float adaptive_gain = base_gain + (0.6f - base_gain) * total_confidence;
    
    // 限制增益范围
    if (adaptive_gain < 0.05f) adaptive_gain = 0.05f;
    if (adaptive_gain > 0.8f) adaptive_gain = 0.8f;
    
    return adaptive_gain;
}

void bmi270_tools::normalize_angles(absolute_orientation_t &orientation)
{
    // 将角度归一化到[-180, 180]度范围
    while (orientation.x_rotation > 180.0f) orientation.x_rotation -= 360.0f;
    while (orientation.x_rotation < -180.0f) orientation.x_rotation += 360.0f;
    
    while (orientation.y_rotation > 180.0f) orientation.y_rotation -= 360.0f;
    while (orientation.y_rotation < -180.0f) orientation.y_rotation += 360.0f;
    
    while (orientation.z_rotation > 180.0f) orientation.z_rotation -= 360.0f;
    while (orientation.z_rotation < -180.0f) orientation.z_rotation += 360.0f;
}

float bmi270_tools::angle_difference(float angle1, float angle2)
{
    // 计算两个角度之间的最小差值（考虑-180/180度边界）
    float diff = angle1 - angle2;
    
    // 标准化到[-180, 180]范围
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    return diff;
}

// === 运动检测和校准函数实现 ===

void bmi270_tools::update_motion_detector(const sensor_data_t &data)
{
    // 初始化运动检测器
    if (_motion_detector.sample_count == 0) {
        _motion_detector.acc_threshold = 0.3f;     // 增加加速度变化阈值 (m/s²)
        _motion_detector.gyro_threshold = 5.0f;    // 增加陀螺仪变化阈值 (°/s)
        _motion_detector.stable_time_ms = 2000;    // 增加稳定时间阈值 (2秒)
        _motion_detector.last_motion_time = esp_timer_get_time() / 1000;
        _motion_detector.is_stationary = false;
        
        // 初始化累积值
        for (int i = 0; i < 3; i++) {
            _motion_detector.acc_sum[i] = 0.0f;
            _motion_detector.acc_sum_sq[i] = 0.0f;
            _motion_detector.gyro_sum[i] = 0.0f;
            _motion_detector.gyro_sum_sq[i] = 0.0f;
            _motion_detector.acc_variance[i] = 0.0f;
            _motion_detector.gyro_variance[i] = 0.0f;
        }
    }

    // 计算当前数据的方差（滑动窗口）
    const uint32_t window_size = 50;  // 50个样本的滑动窗口
    
    // 累积当前样本
    _motion_detector.acc_sum[0] += data.acc_x;
    _motion_detector.acc_sum[1] += data.acc_y;
    _motion_detector.acc_sum[2] += data.acc_z;
    _motion_detector.acc_sum_sq[0] += data.acc_x * data.acc_x;
    _motion_detector.acc_sum_sq[1] += data.acc_y * data.acc_y;
    _motion_detector.acc_sum_sq[2] += data.acc_z * data.acc_z;
    
    _motion_detector.gyro_sum[0] += data.gyr_x;
    _motion_detector.gyro_sum[1] += data.gyr_y;
    _motion_detector.gyro_sum[2] += data.gyr_z;
    _motion_detector.gyro_sum_sq[0] += data.gyr_x * data.gyr_x;
    _motion_detector.gyro_sum_sq[1] += data.gyr_y * data.gyr_y;
    _motion_detector.gyro_sum_sq[2] += data.gyr_z * data.gyr_z;
    
    _motion_detector.sample_count++;
    
    // 当样本数达到窗口大小时，计算方差
    if (_motion_detector.sample_count >= window_size) {
        for (int i = 0; i < 3; i++) {
            float acc_mean = _motion_detector.acc_sum[i] / window_size;
            float acc_mean_sq = _motion_detector.acc_sum_sq[i] / window_size;
            _motion_detector.acc_variance[i] = acc_mean_sq - acc_mean * acc_mean;
            
            float gyro_mean = _motion_detector.gyro_sum[i] / window_size;
            float gyro_mean_sq = _motion_detector.gyro_sum_sq[i] / window_size;
            _motion_detector.gyro_variance[i] = gyro_mean_sq - gyro_mean * gyro_mean;
        }
        
        // 判断是否检测到运动
        float acc_total_variance = _motion_detector.acc_variance[0] + 
                                  _motion_detector.acc_variance[1] + 
                                  _motion_detector.acc_variance[2];
        float gyro_total_variance = _motion_detector.gyro_variance[0] + 
                                   _motion_detector.gyro_variance[1] + 
                                   _motion_detector.gyro_variance[2];
        
        bool motion_detected = (sqrtf(acc_total_variance) > _motion_detector.acc_threshold) ||
                              (sqrtf(gyro_total_variance) > _motion_detector.gyro_threshold);
        
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        if (motion_detected) {
            _motion_detector.last_motion_time = current_time;
            _motion_detector.is_stationary = false;
        } else {
            // 检查是否已经静止足够长时间
            if ((current_time - _motion_detector.last_motion_time) > _motion_detector.stable_time_ms) {
                _motion_detector.is_stationary = true;
            }
        }
        
        // 重置滑动窗口（保持部分历史数据）
        if (_motion_detector.sample_count > window_size * 1.5f) {
            float factor = 0.5f;  // 保留一半的历史数据
            for (int i = 0; i < 3; i++) {
                _motion_detector.acc_sum[i] *= factor;
                _motion_detector.acc_sum_sq[i] *= factor;
                _motion_detector.gyro_sum[i] *= factor;
                _motion_detector.gyro_sum_sq[i] *= factor;
            }
            _motion_detector.sample_count = window_size / 2;
        }
    }
}

bool bmi270_tools::is_device_stationary()
{
    return _motion_detector.is_stationary;
}

void bmi270_tools::update_gyro_bias_calibration(const sensor_data_t &data)
{
    // 只在设备静止时进行陀螺仪零偏校准
    if (!is_device_stationary()) {
        return;
    }
    
    // 初始化校准
    if (!_gyro_bias.calibrated && _gyro_bias.calibration_samples == 0) {
        _gyro_bias.calibration_sum[0] = 0.0f;
        _gyro_bias.calibration_sum[1] = 0.0f;
        _gyro_bias.calibration_sum[2] = 0.0f;
        ESP_LOGI(TAG, "Starting gyroscope bias calibration...");
    }
    
    // 累积样本
    _gyro_bias.calibration_sum[0] += data.gyr_x;
    _gyro_bias.calibration_sum[1] += data.gyr_y;
    _gyro_bias.calibration_sum[2] += data.gyr_z;
    _gyro_bias.calibration_samples++;
    
    // 达到足够样本数时计算零偏
    const uint32_t required_samples = 200;  // 需要200个静止样本
    if (_gyro_bias.calibration_samples >= required_samples) {
        _gyro_bias.bias[0] = _gyro_bias.calibration_sum[0] / required_samples;
        _gyro_bias.bias[1] = _gyro_bias.calibration_sum[1] / required_samples;
        _gyro_bias.bias[2] = _gyro_bias.calibration_sum[2] / required_samples;
        _gyro_bias.calibrated = true;
        
        ESP_LOGI(TAG, "Gyroscope bias calibration completed: X=%.3f°/s, Y=%.3f°/s, Z=%.3f°/s",
                 _gyro_bias.bias[0], _gyro_bias.bias[1], _gyro_bias.bias[2]);
        
        // 重置计数器以便后续更新校准
        _gyro_bias.calibration_samples = 0;
        _gyro_bias.calibration_sum[0] = 0.0f;
        _gyro_bias.calibration_sum[1] = 0.0f;
        _gyro_bias.calibration_sum[2] = 0.0f;
    }
}

void bmi270_tools::apply_gyro_bias_correction(sensor_data_t &data)
{
    if (_gyro_bias.calibrated) {
        data.gyr_x -= _gyro_bias.bias[0];
        data.gyr_y -= _gyro_bias.bias[1];
        data.gyr_z -= _gyro_bias.bias[2];
    }
}

void bmi270_tools::reset_orientation_drift()
{
    // 基于当前加速度计数据重新计算姿态角，用于重置漂移
    sensor_data_t current_data;
    if (get_sensor_data(current_data) == ESP_OK && current_data.acc_valid) {
        float ax = current_data.acc_x / GRAVITY_EARTH;
        float ay = current_data.acc_y / GRAVITY_EARTH;
        float az = current_data.acc_z / GRAVITY_EARTH;
        float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
        
        if (acc_norm > 0.8f && acc_norm < 1.2f) {  // 确保接近1g
            // 归一化
            ax /= acc_norm; ay /= acc_norm; az /= acc_norm;
            
            // 计算基于重力的Roll和Pitch
            float target_roll = atan2f(ay, az);
            float target_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
            
            // 将Roll和Pitch转换为四元数，保持当前Yaw
            float current_yaw = atan2f(2.0f * (_madgwick_filter.q1 * _madgwick_filter.q2 + _madgwick_filter.q0 * _madgwick_filter.q3),
                                      _madgwick_filter.q0 * _madgwick_filter.q0 + _madgwick_filter.q1 * _madgwick_filter.q1 - 
                                      _madgwick_filter.q2 * _madgwick_filter.q2 - _madgwick_filter.q3 * _madgwick_filter.q3);
            
            // 重新构建四元数
            float cr = cosf(target_roll * 0.5f);
            float sr = sinf(target_roll * 0.5f);
            float cp = cosf(target_pitch * 0.5f);
            float sp = sinf(target_pitch * 0.5f);
            float cy = cosf(current_yaw * 0.5f);
            float sy = sinf(current_yaw * 0.5f);
            
            _madgwick_filter.q0 = cr * cp * cy + sr * sp * sy;
            _madgwick_filter.q1 = sr * cp * cy - cr * sp * sy;
            _madgwick_filter.q2 = cr * sp * cy + sr * cp * sy;
            _madgwick_filter.q3 = cr * cp * sy - sr * sp * cy;
            
            // 归一化四元数
            float norm = sqrtf(_madgwick_filter.q0 * _madgwick_filter.q0 + 
                              _madgwick_filter.q1 * _madgwick_filter.q1 + 
                              _madgwick_filter.q2 * _madgwick_filter.q2 + 
                              _madgwick_filter.q3 * _madgwick_filter.q3);
            _madgwick_filter.q0 /= norm;
            _madgwick_filter.q1 /= norm;
            _madgwick_filter.q2 /= norm;
            _madgwick_filter.q3 /= norm;
            
            ESP_LOGI(TAG, "Orientation drift reset based on gravity vector");
        }
    }
}

void bmi270_tools::ensure_angle_continuity(absolute_orientation_t &current, const absolute_orientation_t &previous)
{
    if (!previous.valid) return;
    
    // 改进的角度连续性修正算法
    auto fix_continuity_robust = [](float &current_angle, float previous_angle) {
        // 计算最小角度差
        float diff = current_angle - previous_angle;
        
        // 标准化差值到[-180, 180]范围
        while (diff > 180.0f) {
            diff -= 360.0f;
        }
        while (diff < -180.0f) {
            diff += 360.0f;
        }
        
        // 如果差值仍然很大（超过90度），可能是计算错误
        if (fabsf(diff) > 90.0f) {
            ESP_LOGW(TAG, "Large angle discontinuity detected: %.1f° -> %.1f° (diff=%.1f°)", 
                     previous_angle, current_angle, diff);
            
            // 尝试其他可能的连续角度
            float candidates[3] = {
                current_angle,
                current_angle + 360.0f,
                current_angle - 360.0f
            };
            
            float min_diff = fabsf(diff);
            float best_angle = current_angle;
            
            for (int i = 1; i < 3; i++) {
                float test_diff = fabsf(candidates[i] - previous_angle);
                if (test_diff < min_diff) {
                    min_diff = test_diff;
                    best_angle = candidates[i];
                }
            }
            
            if (best_angle != current_angle) {
                ESP_LOGI(TAG, "Corrected angle from %.1f° to %.1f°", current_angle, best_angle);
                current_angle = best_angle;
            }
        } else {
            // 小幅调整以保持连续性
            current_angle = previous_angle + diff;
        }
    };
    
    fix_continuity_robust(current.x_rotation, previous.x_rotation);
    fix_continuity_robust(current.y_rotation, previous.y_rotation);
    fix_continuity_robust(current.z_rotation, previous.z_rotation);
}

void bmi270_tools::apply_quaternion_filter(float &q0, float &q1, float &q2, float &q3, bool is_stationary)
{
    // 初始化四元数滤波器
    if (!_quat_filter.initialized) {
        _quat_filter.q0 = q0;
        _quat_filter.q1 = q1;
        _quat_filter.q2 = q2;
        _quat_filter.q3 = q3;
        _quat_filter.alpha = 0.8f;  // 默认滤波系数
        _quat_filter.initialized = true;
        return;
    }
    
    // 根据运动状态调整滤波强度
    float alpha = is_stationary ? 0.95f : 0.7f;  // 静止时更强的滤波
    
    // 检查四元数符号一致性（四元数q和-q表示相同的旋转）
    float dot = _quat_filter.q0 * q0 + _quat_filter.q1 * q1 + _quat_filter.q2 * q2 + _quat_filter.q3 * q3;
    if (dot < 0.0f) {
        // 翻转四元数符号以保持一致性
        q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3;
    }
    
    // 球面线性插值 (SLERP) 或简单线性插值
    _quat_filter.q0 = alpha * q0 + (1.0f - alpha) * _quat_filter.q0;
    _quat_filter.q1 = alpha * q1 + (1.0f - alpha) * _quat_filter.q1;
    _quat_filter.q2 = alpha * q2 + (1.0f - alpha) * _quat_filter.q2;
    _quat_filter.q3 = alpha * q3 + (1.0f - alpha) * _quat_filter.q3;
    
    // 重新归一化四元数
    float norm = sqrtf(_quat_filter.q0 * _quat_filter.q0 + _quat_filter.q1 * _quat_filter.q1 + 
                      _quat_filter.q2 * _quat_filter.q2 + _quat_filter.q3 * _quat_filter.q3);
    if (norm > 1e-6f) {
        _quat_filter.q0 /= norm;
        _quat_filter.q1 /= norm;
        _quat_filter.q2 /= norm;
        _quat_filter.q3 /= norm;
    }
    
    // 输出滤波后的四元数
    q0 = _quat_filter.q0;
    q1 = _quat_filter.q1;
    q2 = _quat_filter.q2;
    q3 = _quat_filter.q3;
}

float bmi270_tools::unwrap_angle(float angle, int axis)
{
    // 初始化unwrapper
    if (!_angle_unwrapper.initialized) {
        for (int i = 0; i < 3; i++) {
            _angle_unwrapper.cumulative_offset[i] = 0.0f;
            _angle_unwrapper.last_wrapped_angle[i] = 0.0f;
            _angle_unwrapper.last_unwrapped_angle[i] = 0.0f;
        }
        _angle_unwrapper.initialized = true;
        _angle_unwrapper.last_wrapped_angle[axis] = angle;
        _angle_unwrapper.last_unwrapped_angle[axis] = angle;
        return angle;
    }
    
    // 计算与上次角度的差值
    float diff = angle - _angle_unwrapper.last_wrapped_angle[axis];
    
    // 检测跨越±180°边界的跳变
    if (diff > 180.0f) {
        // 从+180跳到-180 (实际是负方向运动)
        _angle_unwrapper.cumulative_offset[axis] -= 360.0f;
    } else if (diff < -180.0f) {
        // 从-180跳到+180 (实际是正方向运动)
        _angle_unwrapper.cumulative_offset[axis] += 360.0f;
    }
    
    // 计算unwrapped角度
    float unwrapped = angle + _angle_unwrapper.cumulative_offset[axis];
    
    // 更新历史记录
    _angle_unwrapper.last_wrapped_angle[axis] = angle;
    _angle_unwrapper.last_unwrapped_angle[axis] = unwrapped;
    
    return unwrapped;
}

bmi270_tools::absolute_orientation_t bmi270_tools::quaternion_to_xyz_rotation_stable(const quaternion_data_t &quat)
{
    absolute_orientation_t xyz_rotation;
    
    if (!quat.valid) {
        xyz_rotation.x_rotation = xyz_rotation.y_rotation = xyz_rotation.z_rotation = 0.0f;
        xyz_rotation.valid = false;
        return xyz_rotation;
    }
    
    float x = quat.x, y = quat.y, z = quat.z, w = quat.w;
    
    // 使用更稳定的四元数到欧拉角转换方法
    // 参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
    // Roll (X轴旋转)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;
    
    // Pitch (Y轴旋转)
    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(90.0f, sinp); // 使用 ±90° 如果超出范围
    } else {
        pitch = asinf(sinp) * 180.0f / (float)M_PI;
    }
    
    // Yaw (Z轴旋转)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;
    
    // 新的角度连续性保持策略
    if (_angle_tracker.initialized) {
        // 对每个轴进行智能角度修正
        roll = smart_angle_unwrap(roll, _angle_tracker.last_roll);
        pitch = smart_angle_unwrap(pitch, _angle_tracker.last_pitch);
        yaw = smart_angle_unwrap(yaw, _angle_tracker.last_yaw);
        
        // 平滑过渡检查
        float roll_rate = (roll - _angle_tracker.last_roll) / 0.01f; // 假设10ms更新周期
        float pitch_rate = (pitch - _angle_tracker.last_pitch) / 0.01f;
        float yaw_rate = (yaw - _angle_tracker.last_yaw) / 0.01f;
        
        // 如果角速度超过合理范围（例如1000°/s），使用预测值
        const float max_rate = 1000.0f;
        if (fabsf(roll_rate) > max_rate) {
            ESP_LOGW(TAG, "Roll rate too high: %.1f°/s, using prediction", roll_rate);
            roll = _angle_tracker.last_roll + _angle_tracker.roll_rate * 0.01f;
        } else {
            // 更新角速度估计（低通滤波）
            _angle_tracker.roll_rate = 0.8f * _angle_tracker.roll_rate + 0.2f * roll_rate;
        }
        
        if (fabsf(pitch_rate) > max_rate) {
            ESP_LOGW(TAG, "Pitch rate too high: %.1f°/s, using prediction", pitch_rate);
            pitch = _angle_tracker.last_pitch + _angle_tracker.pitch_rate * 0.01f;
        } else {
            _angle_tracker.pitch_rate = 0.8f * _angle_tracker.pitch_rate + 0.2f * pitch_rate;
        }
        
        if (fabsf(yaw_rate) > max_rate) {
            ESP_LOGW(TAG, "Yaw rate too high: %.1f°/s, using prediction", yaw_rate);
            yaw = _angle_tracker.last_yaw + _angle_tracker.yaw_rate * 0.01f;
        } else {
            _angle_tracker.yaw_rate = 0.8f * _angle_tracker.yaw_rate + 0.2f * yaw_rate;
        }
    } else {
        // 初始化角度跟踪器
        _angle_tracker.initialized = true;
        _angle_tracker.roll_rate = 0.0f;
        _angle_tracker.pitch_rate = 0.0f;
        _angle_tracker.yaw_rate = 0.0f;
    }
    
    // 更新历史值
    _angle_tracker.last_roll = roll;
    _angle_tracker.last_pitch = pitch;
    _angle_tracker.last_yaw = yaw;
    
    xyz_rotation.x_rotation = roll;
    xyz_rotation.y_rotation = pitch;
    xyz_rotation.z_rotation = yaw;
    xyz_rotation.valid = true;
    
    return xyz_rotation;
}

void bmi270_tools::reset_angle_unwrapper()
{
    _angle_unwrapper.initialized = false;
    for (int i = 0; i < 3; i++) {
        _angle_unwrapper.cumulative_offset[i] = 0.0f;
        _angle_unwrapper.last_wrapped_angle[i] = 0.0f;
        _angle_unwrapper.last_unwrapped_angle[i] = 0.0f;
    }
    ESP_LOGI(TAG, "Angle unwrapper reset");
}

void bmi270_tools::reset_angle_tracking()
{
    reset_angle_unwrapper();
    _last_orientation.valid = false;
    _quat_filter.initialized = false;
    // 重置新的角度跟踪器
    _angle_tracker.initialized = false;
    ESP_LOGI(TAG, "Angle tracking system reset");
}

// 添加新的智能角度展开函数
float bmi270_tools::smart_angle_unwrap(float current_angle, float previous_angle)
{
    // 计算最短路径差值
    float diff = current_angle - previous_angle;
    
    // 如果差值超过180度，说明可能发生了跳变
    if (diff > 180.0f) {
        // 实际上是向负方向移动，调整当前角度
        current_angle -= 360.0f;
    } else if (diff < -180.0f) {
        // 实际上是向正方向移动，调整当前角度
        current_angle += 360.0f;
    }
    
    // 二次检查：如果调整后的差值仍然很大，可能是真实的快速旋转
    diff = current_angle - previous_angle;
    if (fabsf(diff) > 90.0f) {
        // 记录但不修改，让上层函数决定如何处理
        ESP_LOGD(TAG, "Large angle change detected: %.1f° -> %.1f° (diff=%.1f°)", 
                 previous_angle, current_angle, diff);
    }
    
    return current_angle;
}

// 添加平滑角度过渡函数
float bmi270_tools::smooth_angle_transition(float new_angle, float old_angle, float factor)
{
    // 计算最短路径差值
    float diff = new_angle - old_angle;
    
    // 归一化差值到 [-180, 180]
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    // 应用平滑因子
    float smoothed = old_angle + diff * (1.0f - factor);
    
    // 归一化结果到 [-180, 180]
    while (smoothed > 180.0f) smoothed -= 360.0f;
    while (smoothed < -180.0f) smoothed += 360.0f;
    
    return smoothed;
}

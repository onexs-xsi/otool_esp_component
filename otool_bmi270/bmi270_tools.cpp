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
        deinit();
        return id_ret;
    }
    ESP_LOGI(TAG, "BMI270 pre-read CHIP_ID=0x%02X (expected 0x%02X)", raw_chip_id, BMI270_CHIP_ID);
    if (raw_chip_id != BMI270_CHIP_ID) {
        ESP_LOGW(TAG, "Unexpected CHIP_ID 0x%02X, continue with Bosch SensorAPI init", raw_chip_id);
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
            deinit();
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 %s initialization failed", mode_name);
        print_bmi2_api_error(rslt);
        _initialized = false; // 重置初始化标志
        deinit();
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

    // 更新已启用传感器掩码
    for (uint8_t i = 0; i < count; i++) {
        _enabled_sensors_mask |= (1ULL << sens_list[i]);
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

    // 更新已启用传感器掩码
    for (uint8_t i = 0; i < count; i++) {
        _enabled_sensors_mask &= ~(1ULL << sens_list[i]);
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
        // This bit can remain asserted while the board is moving. Avoid
        // flooding the shared serial transport and starving live UI updates.
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
    struct bmi2_sens_config sens_config = {};
    sens_config.type = BMI2_ANY_MOTION;

    // 创建中断引脚配置结构
    struct bmi2_int_pin_config pin_config = {};

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

esp_err_t bmi270_tools::enter_suspend_mode(bool suspend_accel, bool suspend_gyro, bool suspend_aux)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Entering suspend mode (ACC=%s, GYR=%s, AUX=%s)", 
             suspend_accel ? "OFF" : "ON", 
             suspend_gyro ? "OFF" : "ON", 
             suspend_aux ? "OFF" : "ON");

    // 创建要暂停的传感器列表
    uint8_t suspend_list[3];
    uint8_t suspend_count = 0;

    if (suspend_accel) {
        suspend_list[suspend_count++] = BMI2_ACCEL;
    }
    if (suspend_gyro) {
        suspend_list[suspend_count++] = BMI2_GYRO;
    }
    if (suspend_aux) {
        suspend_list[suspend_count++] = BMI2_AUX;
    }

    if (suspend_count == 0) {
        ESP_LOGW(TAG, "No sensors selected for suspend");
        return ESP_OK;
    }

    // 根据当前模式调用相应的sensor disable函数
    int8_t rslt = BMI2_OK;
    switch (_current_mode) {
        case MODE_CONTEXT:
            rslt = bmi270_context_sensor_disable(suspend_list, suspend_count, &_bmi270_dev);
            break;
            
        case MODE_BASE:
            rslt = bmi270_sensor_disable(suspend_list, suspend_count, &_bmi270_dev);
            break;
            
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            ESP_LOGE(TAG, "Unsupported mode for suspend: %d", _current_mode);
            return ESP_ERR_NOT_SUPPORTED;
    }

    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Failed to suspend sensors: %d", rslt);
        print_bmi2_api_error(rslt);
        return ESP_FAIL;
    }

    // 更新已启用传感器掩码
    if (suspend_accel) {
        _enabled_sensors_mask &= ~(1ULL << BMI2_ACCEL);
    }
    if (suspend_gyro) {
        _enabled_sensors_mask &= ~(1ULL << BMI2_GYRO);
    }
    if (suspend_aux) {
        _enabled_sensors_mask &= ~(1ULL << BMI2_AUX);
    }

    ESP_LOGI(TAG, "Successfully entered suspend mode for %d sensors", suspend_count);
    return ESP_OK;
}

esp_err_t bmi270_tools::prepare_for_sleep(bool keep_motion_interrupt)
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_err = ESP_OK;
    auto keep_first_error = [&first_err](esp_err_t err) {
        if (first_err == ESP_OK && err != ESP_OK) {
            first_err = err;
        }
    };

    ESP_LOGI(TAG, "Preparing BMI270 for sleep (motion_wake=%s)", keep_motion_interrupt ? "on" : "off");

    // 先读取一次中断状态，清除可能锁存/挂起的BMI事件。
    keep_first_error(clear_interrupt());

    const bool aux_was_enabled = is_magnetometer_enabled();

    // 如果曾经通过AUX配置过BMM150，先切到AUX手动模式，把BMM150自身放入suspend。
    // 仅关闭BMI270的PWR_CTRL/AUX位不一定会清除BMM150内部运行状态。
    if (aux_was_enabled) {
        struct bmi2_sens_config aux_config = {};
        aux_config.type = BMI2_AUX;
        int8_t aux_cfg_rslt = BMI2_OK;
        switch (_current_mode) {
            case MODE_CONTEXT:
                aux_cfg_rslt = bmi270_context_get_sensor_config(&aux_config, 1, &_bmi270_dev);
                break;
            case MODE_BASE:
                aux_cfg_rslt = bmi270_get_sensor_config(&aux_config, 1, &_bmi270_dev);
                break;
            case MODE_LEGACY:
            case MODE_MAXIMUM_FIFO:
            default:
                aux_cfg_rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
        if (aux_cfg_rslt == BMI2_OK) {
            aux_config.cfg.aux.aux_en = BMI2_ENABLE;
            aux_config.cfg.aux.manual_en = BMI2_ENABLE;
            switch (_current_mode) {
                case MODE_CONTEXT:
                    aux_cfg_rslt = bmi270_context_set_sensor_config(&aux_config, 1, &_bmi270_dev);
                    break;
                case MODE_BASE:
                    aux_cfg_rslt = bmi270_set_sensor_config(&aux_config, 1, &_bmi270_dev);
                    break;
                case MODE_LEGACY:
                case MODE_MAXIMUM_FIFO:
                default:
                    aux_cfg_rslt = BMI2_E_INVALID_SENSOR;
                    break;
            }
        }
        if (aux_cfg_rslt == BMI2_OK) {
            struct bmm150_settings bmm_sleep_settings = _bmm150_mag_settings;
            bmm_sleep_settings.pwr_mode = BMM150_POWERMODE_SUSPEND;
            int8_t bmm_rslt = bmm150_set_op_mode(&bmm_sleep_settings, &_bmm150_dev);
            if (bmm_rslt != BMM150_OK) {
                ESP_LOGW(TAG, "BMM150 suspend via API failed: %d", bmm_rslt);
                uint8_t bmm_power_off = BMM150_POWER_CNTRL_DISABLE;
                int8_t aux_wr_rslt = bmi2_write_aux_man_mode(BMM150_REG_POWER_CONTROL,
                                                             &bmm_power_off,
                                                             1,
                                                             &_bmi270_dev);
                if (aux_wr_rslt != BMI2_OK) {
                    ESP_LOGW(TAG, "BMM150 power off fallback failed: %d", aux_wr_rslt);
                    keep_first_error(ESP_FAIL);
                }
            } else {
                ESP_LOGI(TAG, "BMM150 entered suspend mode");
            }
        } else {
            ESP_LOGW(TAG, "Skip BMM150 suspend, AUX config unavailable: %d", aux_cfg_rslt);
        }
    } else {
        ESP_LOGI(TAG, "BMM150 was not enabled, skip suspend command");
    }

    if (!keep_motion_interrupt) {
        // 不需要BMI唤醒时，断开特征中断映射并关闭INT输出，避免INT1继续影响PM1 GPIO0。
        struct bmi2_sens_int_config int_unmap[3] = {};
        int_unmap[0].type = BMI2_SIG_MOTION;
        int_unmap[0].hw_int_pin = BMI2_INT_NONE;
        int_unmap[1].type = BMI2_ANY_MOTION;
        int_unmap[1].hw_int_pin = BMI2_INT_NONE;
        int_unmap[2].type = BMI2_NO_MOTION;
        int_unmap[2].hw_int_pin = BMI2_INT_NONE;

        int8_t map_rslt = BMI2_OK;
        switch (_current_mode) {
            case MODE_CONTEXT:
                map_rslt = bmi270_context_map_feat_int(int_unmap, 3, &_bmi270_dev);
                break;
            case MODE_BASE:
                map_rslt = bmi270_map_feat_int(int_unmap, 3, &_bmi270_dev);
                break;
            case MODE_LEGACY:
            case MODE_MAXIMUM_FIFO:
            default:
                map_rslt = BMI2_E_INVALID_SENSOR;
                break;
        }
        if (map_rslt != BMI2_OK) {
            ESP_LOGW(TAG, "Failed to unmap BMI270 motion interrupts: %d", map_rslt);
            keep_first_error(ESP_FAIL);
        }

        keep_first_error(disable_interrupt(INT_PIN_BOTH));
    } else {
        // 保留运动唤醒时，只保持INT1为开漏低有效，避免推挽输出和PM1侧冲突。
        // 使用 ANY_MOTION（=4）而非 SIG_MOTION：SIG_MOTION 要求设备先静止数分钟才能触发，
        // 不适合唤醒场景；ANY_MOTION 检测任意超阈值运动，响应更及时。
        keep_first_error(enable_interrupt(INT_PIN_1, false, true, false));
        keep_first_error(map_interrupt_to_pin(BMI2_ANY_MOTION, INT_PIN_1));
    }

    // 睡前关闭BMM150/AUX相关上拉。init(true)中曾配置2k AUX pull-up，
    // 若L3B随后断电，AUX上拉可能通过BMM150侧反灌，造成mA级漏电。
    uint8_t aux_pull_off = BMI2_ASDA_PUPSEL_OFF;
    int8_t trim_rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &aux_pull_off, 1, &_bmi270_dev);
    if (trim_rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to disable BMI270 AUX pull-up: %d", trim_rslt);
        keep_first_error(ESP_FAIL);
    } else {
        ESP_LOGI(TAG, "BMI270 AUX pull-up disabled");
    }

    // 明确关闭AUX接口，避免ASDA/ASCL在L3B断电后继续保持驱动或上拉状态。
    uint8_t if_conf = 0;
    int8_t if_rslt = bmi2_get_regs(BMI2_IF_CONF_ADDR, &if_conf, 1, &_bmi270_dev);
    if (if_rslt == BMI2_OK) {
        if_conf &= ~BMI2_AUX_IF_EN_MASK;
        if_rslt = bmi2_set_regs(BMI2_IF_CONF_ADDR, &if_conf, 1, &_bmi270_dev);
    }
    if (if_rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to disable BMI270 AUX interface: %d", if_rslt);
        keep_first_error(ESP_FAIL);
    } else {
        ESP_LOGI(TAG, "BMI270 AUX interface disabled");
    }

    // 关闭不需要的传感器。保留BMI唤醒时仅保留加速度/运动特征，其它全部关闭。
    uint8_t sleep_sensors[6] = {};
    uint8_t sleep_count = 0;
    if (!keep_motion_interrupt) {
        sleep_sensors[sleep_count++] = BMI2_SIG_MOTION;
        sleep_sensors[sleep_count++] = BMI2_ANY_MOTION;
        sleep_sensors[sleep_count++] = BMI2_NO_MOTION;
        sleep_sensors[sleep_count++] = BMI2_ACCEL;
    }
    sleep_sensors[sleep_count++] = BMI2_GYRO;
    sleep_sensors[sleep_count++] = BMI2_AUX;

    int8_t dis_rslt = BMI2_OK;
    switch (_current_mode) {
        case MODE_CONTEXT:
            dis_rslt = bmi270_context_sensor_disable(sleep_sensors, sleep_count, &_bmi270_dev);
            break;
        case MODE_BASE:
            dis_rslt = bmi270_sensor_disable(sleep_sensors, sleep_count, &_bmi270_dev);
            break;
        case MODE_LEGACY:
        case MODE_MAXIMUM_FIFO:
        default:
            dis_rslt = BMI2_E_INVALID_SENSOR;
            break;
    }
    if (dis_rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to disable BMI270 sleep sensors: %d", dis_rslt);
        keep_first_error(ESP_FAIL);
    }

    for (uint8_t i = 0; i < sleep_count; ++i) {
        _enabled_sensors_mask &= ~(1ULL << sleep_sensors[i]);
    }

    // 最后直接收敛PWR_CTRL，确保ACC/GYR/AUX电源位处于期望状态。
    uint8_t pwr_ctrl = keep_motion_interrupt ? BMI2_ACC_EN_MASK : 0x00;
    int8_t pwr_rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &pwr_ctrl, 1, &_bmi270_dev);
    if (pwr_rslt != BMI2_OK) {
        ESP_LOGW(TAG, "Failed to set BMI270 PWR_CTRL for sleep: %d", pwr_rslt);
        keep_first_error(ESP_FAIL);
    } else {
        ESP_LOGI(TAG, "BMI270 PWR_CTRL set to 0x%02X", pwr_ctrl);
    }

    if (!keep_motion_interrupt) {
        // 不需要BMI唤醒时，做一次不可恢复式sleep shutdown：清掉feature engine、FIFO和中断状态。
        // 注意：不要调用soft_reset()封装，它会复位后恢复运行态配置；这里要让BMI270停在默认低功耗态。
        int8_t fifo_rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &_bmi270_dev);
        if (fifo_rslt != BMI2_OK) {
            ESP_LOGW(TAG, "Failed to disable BMI270 FIFO before shutdown: %d", fifo_rslt);
            keep_first_error(ESP_FAIL);
        }

        int8_t reset_rslt = bmi2_soft_reset(&_bmi270_dev);
        if (reset_rslt != BMI2_OK) {
            ESP_LOGW(TAG, "BMI270 sleep soft reset failed: %d", reset_rslt);
            keep_first_error(ESP_FAIL);
        } else {
            ESP_LOGI(TAG, "BMI270 sleep soft reset done");
        }

        uint8_t reg_zero = 0x00;
        uint8_t reg_if_conf = 0x00;
        uint8_t reg_aux_trim = BMI2_ASDA_PUPSEL_OFF;
        int8_t post_rslt = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &reg_zero, 1, &_bmi270_dev);
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_INT1_IO_CTRL_ADDR, &reg_zero, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_INT2_IO_CTRL_ADDR, &reg_zero, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_INT1_MAP_FEAT_ADDR, &reg_zero, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_INT2_MAP_FEAT_ADDR, &reg_zero, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_INT_MAP_DATA_ADDR, &reg_zero, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &reg_aux_trim, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_get_regs(BMI2_IF_CONF_ADDR, &reg_if_conf, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            reg_if_conf &= ~BMI2_AUX_IF_EN_MASK;
            post_rslt = bmi2_set_regs(BMI2_IF_CONF_ADDR, &reg_if_conf, 1, &_bmi270_dev);
        }
        if (post_rslt == BMI2_OK) {
            post_rslt = bmi2_set_adv_power_save(BMI2_ENABLE, &_bmi270_dev);
        }
        if (post_rslt != BMI2_OK) {
            ESP_LOGW(TAG, "BMI270 post-reset shutdown register setup failed: %d", post_rslt);
            keep_first_error(ESP_FAIL);
        } else {
            ESP_LOGI(TAG, "BMI270 post-reset shutdown registers applied");
        }

        _enabled_sensors_mask = 0;
        _initialized = false;
    }

    uint8_t reg_pwr_ctrl = 0xFF;
    uint8_t reg_aux_trim = 0xFF;
    uint8_t reg_if_conf = 0xFF;
    uint8_t reg_int1 = 0xFF;
    uint8_t reg_int2 = 0xFF;
    (void)bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &reg_pwr_ctrl, 1, &_bmi270_dev);
    (void)bmi2_get_regs(BMI2_AUX_IF_TRIM, &reg_aux_trim, 1, &_bmi270_dev);
    (void)bmi2_get_regs(BMI2_IF_CONF_ADDR, &reg_if_conf, 1, &_bmi270_dev);
    (void)bmi2_get_regs(BMI2_INT1_IO_CTRL_ADDR, &reg_int1, 1, &_bmi270_dev);
    (void)bmi2_get_regs(BMI2_INT2_IO_CTRL_ADDR, &reg_int2, 1, &_bmi270_dev);
    ESP_LOGI(TAG, "BMI sleep regs: PWR_CTRL=0x%02X AUX_TRIM=0x%02X IF_CONF=0x%02X INT1=0x%02X INT2=0x%02X",
             reg_pwr_ctrl, reg_aux_trim, reg_if_conf, reg_int1, reg_int2);

    return first_err;
}

esp_err_t bmi270_tools::exit_suspend_mode()
{
    if (!_initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Exiting suspend mode and restoring sensor state...");

    // 恢复传感器配置
    esp_err_t ret = configure_sensors(_current_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore sensor configuration");
        return ret;
    }

    // 重新启用之前启用的基础传感器
    uint8_t sens_list[3];
    uint8_t cnt = 0;
    if (_enabled_sensors_mask & (1ULL << BMI2_ACCEL)) sens_list[cnt++] = BMI2_ACCEL;
    if (_enabled_sensors_mask & (1ULL << BMI2_GYRO))  sens_list[cnt++] = BMI2_GYRO;
    if (_enabled_sensors_mask & (1ULL << BMI2_AUX))   sens_list[cnt++] = BMI2_AUX;
    
    if (cnt > 0) {
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
                ESP_LOGE(TAG, "Unsupported mode for suspend exit: %d", _current_mode);
                return ESP_ERR_NOT_SUPPORTED;
        }
        
        if (rslt != BMI2_OK) {
            ESP_LOGE(TAG, "Failed to restore sensor enable state: %d", rslt);
            print_bmi2_api_error(rslt);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Successfully restored %d sensors", cnt);
    } else {
        ESP_LOGW(TAG, "No sensors to restore");
    }

    ESP_LOGI(TAG, "Successfully exited suspend mode");
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
    // REGULAR(REPXY=4/REPZ=15) 而非 ENHANCED(REPXY=7)：重复次数更低，单次测量更快，
    // 磁场实际数据率更接近 AUX_ODR 50Hz，指南针指针新鲜度更高；精度对指南针足够。
    _bmm150_mag_settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    rslt = bmm150_set_presetmode(&_bmm150_mag_settings, &_bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to set BMM150 preset mode: %d", rslt);
        print_bmm150_api_error(rslt);
        free(config);
        return rslt;
    }
    else{
        ESP_LOGI(TAG, "BMM150 preset mode set to REGULAR");
    }

    // REGULAR preset resets the BMM150 itself to 10 Hz even though BMI270 AUX
    // is configured for 50 Hz. Use a real 20 Hz conversion rate so Factory P4
    // receives fresh magnetic samples at each 50 ms monitor cycle.
    _bmm150_mag_settings.data_rate = BMM150_DATA_RATE_20HZ;
    rslt = bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE,
                                      &_bmm150_mag_settings,
                                      &_bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to set BMM150 data rate to 20 Hz: %d", rslt);
        print_bmm150_api_error(rslt);
        free(config);
        return rslt;
    }
    ESP_LOGI(TAG, "BMM150 data rate set to 20 Hz");

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
    struct bmi2_sens_data sensor_data = {};
    
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



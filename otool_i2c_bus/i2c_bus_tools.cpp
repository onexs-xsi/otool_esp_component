/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "i2c_bus_tools.h"

static const char *TAG = "i2c_bus_tools";

// 构造函数（默认参数）
i2c_bus_tools::i2c_bus_tools()
{
    ESP_LOGI(TAG, "i2c_bus_tools object created with default parameters");
    g_i2c_bus = NULL;
    i2c_sda_pin = I2C_MASTER_SDA_IO;
    i2c_scl_pin = I2C_MASTER_SCL_IO;
    i2c_frequency = I2C_MASTER_FREQ_HZ;
    i2c_port_num = I2C_NUM_0;  // 默认使用 I2C_NUM_0
    initialized = false;
    
    // 初始化设备句柄数组
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        i2c_device_all[i] = NULL;
    }
}

// 构造函数（带参数）
i2c_bus_tools::i2c_bus_tools(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency)
{
    ESP_LOGI(TAG, "i2c_bus_tools object created with custom parameters");
    g_i2c_bus = NULL;
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    i2c_frequency = frequency;
    i2c_port_num = I2C_NUM_0;  // 默认使用 I2C_NUM_0
    initialized = false;
    
    // 初始化设备句柄数组
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        i2c_device_all[i] = NULL;
    }
}

// 构造函数（完整参数）
i2c_bus_tools::i2c_bus_tools(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num)
{
    ESP_LOGI(TAG, "i2c_bus_tools object created with full parameters (port: %d)", port_num);
    g_i2c_bus = NULL;
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    i2c_frequency = frequency;
    i2c_port_num = port_num;
    initialized = false;
    
    // 初始化设备句柄数组
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        i2c_device_all[i] = NULL;
    }
}

// 析构函数
i2c_bus_tools::~i2c_bus_tools()
{
    ESP_LOGI(TAG, "i2c_bus_tools object destroyed");
    // 清理I2C资源
    unregister_i2c_bus_device();
}

esp_err_t i2c_bus_tools::register_i2c_bus_device(i2c_bus_handle_t bus, uint32_t speed, gpio_num_t i2c_sda_pin, gpio_num_t i2c_scl_pin)
{
    // Initialize I2C bus    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {
            .clk_speed = speed,
        },
        .clk_flags = 0
    };
    g_i2c_bus = i2c_bus_create(i2c_port_num, &conf);
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus on port %d", i2c_port_num);
        return ESP_FAIL;
    }
    else {
        ESP_LOGI(TAG, "I2C bus created successfully on port %d", i2c_port_num);
        initialized = true;
        return ESP_OK;
    }
}

esp_err_t i2c_bus_tools::register_i2c_bus_device(void)
{
    if (g_i2c_bus != NULL) {
        ESP_LOGW(TAG, "I2C bus already registered");
        return ESP_OK;  // 已经注册，返回成功
    }

    ESP_LOGI(TAG, "Registering I2C bus with SDA=%d, SCL=%d, Freq=%lu, Port=%d", 
             i2c_sda_pin, i2c_scl_pin, i2c_frequency, i2c_port_num);

    // Register I2C bus device
    esp_err_t ret = register_i2c_bus_device(g_i2c_bus, i2c_frequency, i2c_sda_pin, i2c_scl_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register I2C bus device");
        return ret;
    }
    
    // Initialize I2C devices
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        i2c_device_all[i] = NULL;
    }
    
    ESP_LOGI(TAG, "I2C bus and devices initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_bus_tools::unregister_i2c_bus_device(i2c_bus_handle_t bus)
{
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Unregister all I2C devices FIRST (to drop references to the bus)
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        if (i2c_device_all[i] != NULL) {
            i2c_bus_device_delete(&i2c_device_all[i]);
            i2c_device_all[i] = NULL;
        }
    }

    // Then unregister I2C bus
    esp_err_t ret = i2c_bus_delete(&bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_bus_delete returned %s (bus may still be referenced elsewhere)", esp_err_to_name(ret));
    }
    g_i2c_bus = NULL;

    ESP_LOGI(TAG, "I2C devices unregistered, bus delete attempted");
    initialized = false;
    return ret;
}

esp_err_t i2c_bus_tools::unregister_i2c_bus_device(void)
{
    if (g_i2c_bus == NULL) {
        ESP_LOGW(TAG, "I2C bus not registered");
        return ESP_ERR_INVALID_STATE;
    }

    // Delete all I2C devices FIRST
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        if (i2c_device_all[i] != NULL) {
            i2c_bus_device_delete(&i2c_device_all[i]);
            i2c_device_all[i] = NULL;
        }
    }

    // Then delete I2C bus
    esp_err_t ret = i2c_bus_delete(&g_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_bus_delete returned %s (bus may still be referenced elsewhere)", esp_err_to_name(ret));
    }
    g_i2c_bus = NULL;

    ESP_LOGI(TAG, "I2C devices unregistered, bus delete attempted");
    initialized = false;
    return ret;
}

esp_err_t i2c_bus_tools::scan_and_add_i2c_bus_devices(void)
{
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // SCAN
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    // Scan I2C bus for devices
    uint8_t addrs[I2C_SCAN_ADDR_NUM] = {0};
    uint8_t i2c_device_num = 0;
    i2c_device_num = i2c_bus_scan(g_i2c_bus, addrs, I2C_SCAN_ADDR_NUM);

    // ADD
    for (uint8_t i = 0; i < i2c_device_num; i++) {
        uint8_t addr = addrs[i];
        if (addr < I2C_SCAN_ADDR_NUM && i2c_device_all[addr] == NULL) {
            if (i2c_device_all[addr] != NULL) {
                ESP_LOGI(TAG, "I2C device at address 0x%02X already exists", addr);
                continue;
            }
            i2c_device_all[addr] = i2c_bus_device_create(g_i2c_bus, addr, i2c_frequency);
            if (i2c_device_all[addr] != NULL) {
                ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
            } else {
                ESP_LOGE(TAG, "Failed to create I2C device at address 0x%02X", addr);
            }
        }
    }

    ESP_LOGI(TAG, "I2C bus scan completed");
    return ESP_OK;
}

i2c_bus_handle_t i2c_bus_tools::get_i2c_bus_handle(void) const
{
    ESP_LOGI(TAG, "Getting I2C bus handle: %p, initialized: %s", 
             g_i2c_bus, initialized ? "YES" : "NO");
    return g_i2c_bus;
}

i2c_bus_device_handle_t i2c_bus_tools::get_i2c_device_handle(uint8_t addr) const
{
    if (addr >= I2C_SCAN_ADDR_NUM) {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02X", addr);
        return NULL;
    }
    return i2c_device_all[addr];
}

i2c_master_bus_handle_t i2c_bus_tools::get_i2c_master_bus_handle(void) const
{
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return NULL;
    }
    return i2c_bus_get_internal_bus_handle(g_i2c_bus);
}

void i2c_bus_tools::set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency)
{
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    i2c_frequency = frequency;
    ESP_LOGI(TAG, "I2C config updated: SDA=%d, SCL=%d, Freq=%lu", sda_pin, scl_pin, frequency);
}

void i2c_bus_tools::set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num)
{
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    i2c_frequency = frequency;
    i2c_port_num = port_num;
    ESP_LOGI(TAG, "I2C config updated: SDA=%d, SCL=%d, Freq=%lu, Port=%d", sda_pin, scl_pin, frequency, port_num);
}

i2c_port_t i2c_bus_tools::get_i2c_port_num(void) const
{
    ESP_LOGI(TAG, "Current I2C port: %d", i2c_port_num);
    return i2c_port_num;
}

void i2c_bus_tools::detach_i2c_device_by_addr(uint8_t addr)
{
    if (addr >= I2C_SCAN_ADDR_NUM) {
        ESP_LOGE(TAG, "detach_i2c_device_by_addr: invalid addr 0x%02X", addr);
        return;
    }
    if (i2c_device_all[addr] != NULL) {
        ESP_LOGI(TAG, "Detaching I2C device at 0x%02X from internal registry (no delete)", addr);
        i2c_device_all[addr] = NULL; // Do not delete here; owning module will delete
    }
}

void i2c_bus_tools::update_i2c_device_handle(uint8_t addr, i2c_bus_device_handle_t new_handle)
{
    if (addr >= I2C_SCAN_ADDR_NUM) {
        ESP_LOGE(TAG, "update_i2c_device_handle: invalid addr 0x%02X", addr);
        return;
    }
    
    if (i2c_device_all[addr] != NULL) {
        ESP_LOGW(TAG, "Updating I2C device handle at 0x%02X (old: %p -> new: %p)", 
                 addr, i2c_device_all[addr], new_handle);
    } else {
        ESP_LOGI(TAG, "Setting I2C device handle at 0x%02X to %p", addr, new_handle);
    }
    
    i2c_device_all[addr] = new_handle;
}
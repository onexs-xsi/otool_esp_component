/*
 * BMI270 + BMM150 磁力计使用示例
 * 
 * 此示例展示如何配置和读取BMM150磁力计数据
 * BMM150通过BMI270的辅助传感器接口连接
 */

#include "bmi270_tools.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BMM150_EXAMPLE";

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== BMI270 + BMM150 磁力计示例 ===");

    // 1. 初始化I2C总线
    i2c_bus_handle_t i2c_bus = nullptr;
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_15,      // 根据实际硬件修改
        .scl_io_num = GPIO_NUM_16,      // 根据实际硬件修改
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000,        // 400kHz
        },
    };

    i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C总线创建失败");
        return;
    }

    // 2. 创建BMI270传感器对象
    BMI270 sensor(i2c_bus, BMI270_I2C_ADDR_PRIMARY);

    // 3. 初始化BMI270
    int ret = sensor.initialize(50.0f, 50.0f);  // 50Hz采样率
    if (ret != 0) {
        ESP_LOGE(TAG, "BMI270初始化失败: %d", ret);
        i2c_bus_delete(&i2c_bus);
        return;
    }
    ESP_LOGI(TAG, "BMI270初始化成功");

    // 4. 配置BMM150磁力计
    ret = sensor.configure_magnetometer();
    if (ret != 0) {
        ESP_LOGE(TAG, "BMM150磁力计配置失败: %d", ret);
        i2c_bus_delete(&i2c_bus);
        return;
    }
    ESP_LOGI(TAG, "BMM150磁力计配置成功");

    // 5. 等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 6. 主循环: 读取传感器数据
    BMI270::sensor_data_t imu_data;
    BMI270::mag_data_t mag_data;
    int read_count = 0;
    int valid_mag_count = 0;

    while (true) {
        // 读取BMI270六轴数据
        ret = sensor.read_sensor_data(&imu_data);
        if (ret == 0 && (imu_data.acc_valid || imu_data.gyr_valid)) {
            ESP_LOGI(TAG, "IMU数据 - 加速度:[%d,%d,%d] 陀螺仪:[%d,%d,%d]",
                     imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                     imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z);
        }

        // 读取BMM150磁力计数据
        ret = sensor.read_magnetometer_data(&mag_data);
        read_count++;
        
        if (ret == 0 && mag_data.valid) {
            valid_mag_count++;
            ESP_LOGI(TAG, "磁力计数据 - X:%ld uT, Y:%ld uT, Z:%ld uT", 
                     mag_data.x, mag_data.y, mag_data.z);
                     
            // 计算磁场强度
            float magnitude = sqrtf(mag_data.x * mag_data.x + 
                                  mag_data.y * mag_data.y + 
                                  mag_data.z * mag_data.z);
            ESP_LOGI(TAG, "磁场强度: %.1f uT", magnitude);
            
            // 计算航向角 (简化版)
            float heading = atan2f(mag_data.y, mag_data.x) * 180.0f / M_PI;
            if (heading < 0) heading += 360.0f;
            ESP_LOGI(TAG, "航向角: %.1f 度", heading);
        } else if (ret != 0) {
            ESP_LOGW(TAG, "磁力计数据读取失败: %d", ret);
        }

        // 每10次读取输出一次统计信息
        if (read_count % 10 == 0) {
            ESP_LOGI(TAG, "磁力计数据有效率: %d/%d (%.1f%%)", 
                     valid_mag_count, read_count, 
                     (float)valid_mag_count * 100.0f / read_count);
        }

        // 延时200ms (5Hz显示频率)
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 清理资源
    i2c_bus_delete(&i2c_bus);
}

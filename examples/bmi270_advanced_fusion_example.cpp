/*
 * BMI270 高精度姿态融合算法使用示例
 * 
 * 此示例展示如何使用完整的9轴传感器融合算法：
 * - Madgwick 滤波器 (6轴/9轴)
 * - 传感器校准
 * - 高精度四元数和姿态计算
 */

#include "bmi270_tools.h"
#include "i2c_bus_tools.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "BMI270_ADVANCED_FUSION";

extern "C" void app_main() {
    ESP_LOGI(TAG, "BMI270 高精度姿态融合算法示例");

    // 1. 初始化I2C总线
    i2c_bus_handle_t i2c_bus = nullptr;
    i2c_config_t i2c_config = i2c_bus_tools::get_default_config();
    i2c_config.sda_io_num = GPIO_NUM_8;  // 根据硬件修改
    i2c_config.scl_io_num = GPIO_NUM_9;  // 根据硬件修改
    
    int ret = i2c_bus_tools::create_bus(&i2c_config, &i2c_bus);
    if (ret != ESP_OK || !i2c_bus) {
        ESP_LOGE(TAG, "I2C总线初始化失败: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "I2C总线初始化成功");

    // 2. 初始化BMI270传感器
    bmi270_tools sensor;
    ret = sensor.init(i2c_bus, true);  // 启用磁力计
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270传感器初始化失败: %d", ret);
        i2c_bus_delete(&i2c_bus);
        return;
    }
    ESP_LOGI(TAG, "BMI270传感器初始化成功");

    // 3. 启用所有传感器
    bmi270_tools::sensor_type_t sensors[] = {
        bmi270_tools::SENSOR_ACCEL,
        bmi270_tools::SENSOR_GYRO,
        bmi270_tools::SENSOR_AUX  // 磁力计
    };
    ret = sensor.enable_sensors(sensors, sizeof(sensors)/sizeof(sensors[0]));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "传感器启用失败: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "所有传感器启用成功");

    // 4. 配置传感器参数
    bmi270_tools::sensor_config_t config = {
        .acc_range = bmi270_tools::ACCEL_RANGE_4G,
        .gyr_range = bmi270_tools::GYRO_RANGE_1000,
        .acc_odr = bmi270_tools::ODR_200HZ,
        .gyr_odr = bmi270_tools::ODR_200HZ
    };
    sensor.configure_sensors(config);
    ESP_LOGI(TAG, "传感器配置完成");

    // 5. 等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 6. 传感器校准（可选，但强烈推荐）
    ESP_LOGI(TAG, "开始传感器校准...");
    ESP_LOGI(TAG, "请保持设备静止10秒进行校准");
    
    ret = sensor.calibrate_sensors(10000, 500);  // 10秒，500个样本
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "传感器校准完成");
    } else {
        ESP_LOGW(TAG, "传感器校准失败，继续使用默认参数");
    }

    // 7. 设置Madgwick滤波器参数
    sensor.set_madgwick_beta(0.5f);  // 调整收敛速度
    ESP_LOGI(TAG, "Madgwick滤波器配置完成");

    // 8. 主循环: 高精度姿态解算
    ESP_LOGI(TAG, "开始高精度姿态解算...");
    
    int loop_count = 0;
    const int display_interval = 50;  // 每50次循环显示一次结果
    
    while (true) {
        // 方法1: 使用高精度Madgwick四元数融合
        bmi270_tools::quaternion_data_t madgwick_quat;
        ret = sensor.get_quaternion_madgwick(madgwick_quat, true);  // 9轴融合
        
        if (ret == ESP_OK && madgwick_quat.valid) {
            // 转换为欧拉角
            bmi270_tools::absolute_orientation_t orientation = 
                bmi270_tools::quaternion_to_xyz_rotation(madgwick_quat);
            
            // 每隔一段时间显示结果
            if (loop_count % display_interval == 0) {
                ESP_LOGI(TAG, "=== 高精度Madgwick 9轴融合结果 ===");
                ESP_LOGI(TAG, "四元数: w=%.4f, x=%.4f, y=%.4f, z=%.4f", 
                         madgwick_quat.w, madgwick_quat.x, madgwick_quat.y, madgwick_quat.z);
                ESP_LOGI(TAG, "姿态角: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°", 
                         orientation.x_rotation, orientation.y_rotation, orientation.z_rotation);
                
                // 比较：使用简化算法的结果
                bmi270_tools::quaternion_data_t simple_quat;
                ret = sensor.get_quaternion_data(simple_quat);
                if (ret == ESP_OK && simple_quat.valid) {
                    bmi270_tools::absolute_orientation_t simple_orientation = 
                        bmi270_tools::quaternion_to_xyz_rotation(simple_quat);
                    ESP_LOGI(TAG, "简化算法: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°", 
                             simple_orientation.x_rotation, simple_orientation.y_rotation, simple_orientation.z_rotation);
                }
                
                ESP_LOGI(TAG, "");
            }
        }
        
        // 方法2: 一键获取绝对方向（包含倾斜补偿的磁力计偏航角）
        if (loop_count % display_interval == 25) {  // 错开显示时间
            bmi270_tools::absolute_orientation_t abs_orientation;
            ret = sensor.get_absolute_orientation(abs_orientation);
            if (ret == ESP_OK && abs_orientation.valid) {
                ESP_LOGI(TAG, "=== 倾斜补偿磁力计偏航角结果 ===");
                ESP_LOGI(TAG, "绝对方向: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°", 
                         abs_orientation.x_rotation, abs_orientation.y_rotation, abs_orientation.z_rotation);
                ESP_LOGI(TAG, "");
            }
        }
        
        loop_count++;
        
        // 高频率更新 (200Hz 对应 5ms)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/*
使用说明：

1. 硬件连接：
   - BMI270 通过I2C连接到ESP32
   - BMM150 磁力计通过BMI270的AUX接口连接
   - 根据实际硬件修改GPIO引脚配置

2. 校准过程：
   - 启动后保持设备静止10秒进行陀螺仪和加速度计校准
   - 磁力计校准需要将设备做8字形旋转运动

3. 算法选择：
   - get_quaternion_madgwick(): 高精度Madgwick 9轴融合
   - get_quaternion_data(): 简化的加速度计倾斜计算
   - get_absolute_orientation(): 倾斜补偿的磁力计偏航角

4. 参数调整：
   - beta值 (0.1-2.0): 控制滤波器收敛速度，越大收敛越快但噪声越大
   - 采样频率: 推荐100-200Hz
   - 校准时间: 可根据需要调整校准样本数量

5. 精度对比：
   - Madgwick 9轴融合: 最高精度，适合需要精确姿态的应用
   - 简化算法: 计算量小，适合低功耗或实时性要求高的场景
   - 倾斜补偿磁力计: 中等精度，适合需要准确偏航角的应用
*/

# RX8130 RTC任务调度器示例

本示例展示了`ic_rx8130_tools`的完整功能，包括RTC时间管理和智能任务调度系统。

## 功能特性

### 🕒 RTC基础功能
- RTC时间设置和读取
- 电池备份初始化
- 中断引脚配置

### 📋 任务调度系统
- **延时任务**：指定延迟时间后执行
- **定时任务**：在特定时间点执行
- **重复任务**：按固定间隔重复执行
- **任务说明**：为每个任务添加可选的描述信息

### 🛠️ 任务管理
- 任务查询和状态监控
- 任务取消和批量操作
- 时间同步和任务重计算
- 详细的调试信息输出

## 硬件连接

```
ESP32 Pin    RX8130 Pin    描述
--------     ----------    ----
GPIO8        SDA           I2C数据线
GPIO9        SCL           I2C时钟线
GPIO5        INT           中断输出引脚（可选）
3.3V         VCC           电源正极
GND          GND           电源负极
```

## 使用方法

### 1. 完整功能演示

```cpp
#include "ic_rx8130_task_scheduler_example.h"

void app_main()
{
    // 运行完整的功能演示
    ic_rx8130_example_main();
}
```

### 2. 快速测试

```cpp
#include "ic_rx8130_task_scheduler_example.h"

void app_main()
{
    // 运行简化的快速测试
    ic_rx8130_quick_test();
}
```

### 3. 自定义使用

```cpp
#include "ic_rx8130.h"
#include "i2c_bus_tools.h"

void app_main()
{
    // 初始化I2C总线
    i2c_bus_tools i2c_bus;
    i2c_bus.init(I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9, 400000);
    
    // 初始化RTC
    ic_rx8130_tools rtc;
    rtc.init(i2c_bus.get_bus_handle(), 0x32, GPIO_NUM_5);
    rtc.initBat();
    
    // 设置当前时间
    struct tm current_time = {0};
    current_time.tm_year = 2025 - 1900;
    current_time.tm_mon = 8;  // September
    current_time.tm_mday = 2;
    current_time.tm_hour = 14;
    current_time.tm_min = 30;
    current_time.tm_sec = 0;
    rtc.setTime(&current_time);
    
    // 创建任务
    rtc_task_handle task1 = rtc.scheduleTask(10, [](){
        printf("Hello from scheduled task!\n");
    }, "Greeting task");
    
    rtc_task_handle task2 = rtc.scheduleRepeatingTask(30, [](){
        printf("Repeating task executed!\n");
    }, "Periodic task");
    
    // 查询任务状态
    rtc.printTasksStatus();
}
```

## 示例执行流程

### 阶段1：初始化 (0-5秒)
- 初始化I2C总线和RTC
- 设置初始时间
- 创建示例任务
- 显示任务信息

### 阶段2：任务执行监控 (5-60秒)
- 系统初始化任务执行 (5秒)
- 传感器读取任务开始重复执行 (每10秒)
- 心跳任务开始重复执行 (每15秒)
- 状态报告任务开始重复执行 (每30秒)
- 数据备份任务执行 (60秒)

### 阶段3：任务管理演示 (60-80秒)
- 打印详细任务状态
- 演示任务取消功能
- 显示剩余活跃任务

### 阶段4：时间同步演示 (80-90秒)
- 调整RTC时间 (快进5分钟)
- 展示任务时间自动重计算
- 验证任务调度正确性

### 阶段5：持续监控 (90-210秒)
- 监控任务执行情况
- 定期打印系统状态
- 记录任务执行统计

### 阶段6：清理 (210秒+)
- 取消所有剩余任务
- 显示最终统计信息

## 预期输出示例

```
I (000) RX8130_EXAMPLE: 🚀 === RX8130 Task Scheduler Example Started ===
I (010) RX8130_EXAMPLE: ✅ I2C bus initialized successfully
I (020) RX8130_EXAMPLE: ✅ RX8130 initialized successfully with interrupt on GPIO5
I (030) RX8130_EXAMPLE: ✅ Battery backup initialized
I (040) RX8130_EXAMPLE: 🕒 RTC time set to: 2025-09-02 14:30:00
I (050) RX8130_EXAMPLE: 🔧 Creating example tasks...
I (060) RX8130_EXAMPLE: ✅ Created init task (ID: 1) - will execute in 5 seconds
I (070) RX8130_EXAMPLE: ✅ Created repeating sensor task (ID: 2) - every 10 seconds
I (080) RX8130_EXAMPLE: ✅ Created repeating heartbeat task (ID: 3) - every 15 seconds
I (090) RX8130_EXAMPLE: ✅ Created scheduled backup task (ID: 4) - at 14:31
I (100) RX8130_EXAMPLE: ✅ Created repeating report task (ID: 5) - every 30 seconds
I (110) RX8130_EXAMPLE: 📝 Total tasks created: 5

I (5000) RX8130_EXAMPLE: 🚀 [Task] System initialization completed!
I (10000) RX8130_EXAMPLE: 📊 [Task] Sensor reading #1 - Temperature: 25.1°C
I (15000) RX8130_EXAMPLE: 💓 [Task] Heartbeat #1 - System is alive!
I (20000) RX8130_EXAMPLE: 📊 [Task] Sensor reading #2 - Temperature: 25.2°C
I (30000) RX8130_EXAMPLE: 📋 [Task] Status Report:
I (30010) RX8130_EXAMPLE:      - Sensor readings: 3
I (30020) RX8130_EXAMPLE:      - Heartbeats sent: 2
I (30030) RX8130_EXAMPLE:      - Backups created: 0
I (30040) RX8130_EXAMPLE:      - Active tasks: 4
...
```

## 调试功能

### 任务状态打印
```cpp
rtc.printTasksStatus();
```

输出示例：
```
=== RTC Tasks Status ===
Current time: Mon Sep  2 14:30:45 2025
Active tasks count: 4

Task Details:
Task ID: 2
  Description: Sensor data reading
  Target time: Mon Sep  2 14:30:50 2025
  Created time: Mon Sep  2 14:30:00 2025
  Type: Repeating
  Interval: 10 seconds
  Time to execution: 5 seconds
  ---
...
========================
```

### 任务信息查询
```cpp
rtc_task_info info;
if (rtc.getTaskInfo(task_handle, info)) {
    printf("Task: %s\n", info.description.c_str());
    printf("Type: %s\n", info.is_repeating ? "Repeating" : "One-time");
}
```

## 注意事项

1. **GPIO配置**：确保中断引脚未被其他外设占用
2. **I2C地址**：RX8130默认地址为0x32
3. **时间格式**：使用标准的`struct tm`格式
4. **内存管理**：任务系统会自动清理已完成的一次性任务
5. **中断处理**：任务回调在独立线程中执行，避免阻塞ISR

## 扩展功能

### 自定义任务类型
```cpp
// 条件任务：只在特定条件下执行
rtc_task_handle conditional_task = rtc.scheduleTask(5, [](){
    if (sensor_value > threshold) {
        execute_emergency_protocol();
    }
}, "Emergency check");

// 链式任务：一个任务完成后启动另一个
rtc_task_handle chain_task = rtc.scheduleTask(10, [](){
    process_data();
    rtc.scheduleTask(5, send_results, "Send results");
}, "Data processing");
```

### 动态任务管理
```cpp
// 根据系统负载调整任务频率
void adjust_task_frequency() {
    auto handles = rtc.getActiveTaskHandles();
    for (auto handle : handles) {
        rtc_task_info info;
        if (rtc.getTaskInfo(handle, info) && info.description == "CPU monitoring") {
            rtc.cancelTask(handle);
            // 重新创建具有新频率的任务
            uint32_t new_interval = calculate_optimal_interval();
            rtc.scheduleRepeatingTask(new_interval, cpu_monitor_task, "CPU monitoring");
        }
    }
}
```

## 故障排除

1. **任务不执行**：检查RTC时间设置是否正确
2. **中断不触发**：验证GPIO连接和配置
3. **时间漂移**：检查晶振和电池备份
4. **内存泄漏**：确保合理使用任务数量

## 相关文档

- [RX8130数据手册](https://download.epsondevice.com/td/pdf/app/RX8130CE_en.pdf)
- [ESP-IDF I2C驱动文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
- [FreeRTOS任务管理](https://www.freertos.org/taskandcr.html)

/*
 * SPDX-FileCopyrightText: 2025 ESP-IDF Community
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ic_rx8130_usage_guide.md
 * @brief RX8130使用快速指南
 */

# RX8130 RTC任务调度器 - 快速使用指南

## 🚀 快速开始

### 1. 在main.cpp中使用

```cpp
#include "ic_rx8130_task_scheduler_example.h"

extern "C" void app_main()
{
    // 选择以下任一方式：
    
    // 方式1：完整功能演示（推荐第一次使用）
    ic_rx8130_example_main();
    
    // 方式2：快速测试（验证硬件连接）
    // ic_rx8130_quick_test();
}
```

### 2. 基本使用模式

```cpp
#include "ic_rx8130.h"
#include "i2c_bus_tools.h"

// 全局对象
i2c_bus_tools i2c_bus;
ic_rx8130_tools rtc;

void setup_rtc() {
    // 初始化I2C
    i2c_bus.init(I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9, 400000);
    
    // 初始化RTC（GPIO5为中断引脚）
    rtc.init(i2c_bus.get_bus_handle(), 0x32, GPIO_NUM_5);
    rtc.initBat();
    
    // 设置时间
    struct tm time = {0};
    time.tm_year = 2025 - 1900;
    time.tm_mon = 8;  // September
    time.tm_mday = 2;
    time.tm_hour = 14;
    time.tm_min = 30;
    rtc.setTime(&time);
}

void create_tasks() {
    // 延时任务
    rtc.scheduleTask(10, [](){
        printf("10秒后执行\n");
    }, "延时任务");
    
    // 重复任务
    rtc.scheduleRepeatingTask(30, [](){
        printf("每30秒执行一次\n");
    }, "心跳任务");
    
    // 指定时间任务
    struct tm target_time;
    rtc.getTime(&target_time);
    target_time.tm_min += 2;  // 2分钟后
    rtc.scheduleTaskAt(&target_time, [](){
        printf("指定时间执行\n");
    }, "定时任务");
}
```

## 📋 常用API

### 任务创建
```cpp
// 延时执行
rtc_task_handle h1 = rtc.scheduleTask(seconds, callback, "说明");

// 指定时间执行
rtc_task_handle h2 = rtc.scheduleTaskAt(&time, callback, "说明");

// 重复执行
rtc_task_handle h3 = rtc.scheduleRepeatingTask(interval, callback, "说明");

// 指定时间开始重复
rtc_task_handle h4 = rtc.scheduleRepeatingTaskAt(&start_time, interval, callback, "说明");
```

### 任务管理
```cpp
// 取消任务
rtc.cancelTask(handle);

// 取消所有任务
rtc.cancelAllTasks();

// 获取活跃任务数
size_t count = rtc.getActiveTaskCount();

// 打印任务状态
rtc.printTasksStatus();
```

### 任务查询
```cpp
// 获取任务信息
rtc_task_info info;
if (rtc.getTaskInfo(handle, info)) {
    printf("任务：%s\n", info.description.c_str());
    printf("类型：%s\n", info.is_repeating ? "重复" : "一次性");
}

// 获取所有任务
auto all_tasks = rtc.getAllTasksInfo();
for (const auto& task : all_tasks) {
    printf("任务ID：%u，说明：%s\n", task.handle.task_id, task.description.c_str());
}
```

## ⚡ 实用示例

### 传感器定时读取
```cpp
rtc.scheduleRepeatingTask(60, [](){
    float temperature = read_temperature();
    float humidity = read_humidity();
    printf("温度：%.1f°C，湿度：%.1f%%\n", temperature, humidity);
    
    // 如果超过阈值，立即报警
    if (temperature > 35.0) {
        rtc.scheduleTask(0, [](){
            send_alarm("温度过高");
        }, "温度报警");
    }
}, "传感器读取");
```

### 数据定时备份
```cpp
rtc.scheduleRepeatingTask(3600, [](){  // 每小时
    backup_data_to_flash();
    printf("数据已备份\n");
}, "数据备份");
```

### 网络心跳
```cpp
rtc.scheduleRepeatingTask(30, [](){
    if (wifi_is_connected()) {
        send_heartbeat();
    } else {
        reconnect_wifi();
    }
}, "网络心跳");
```

### 定时关机
```cpp
// 设置1小时后关机
struct tm shutdown_time;
rtc.getTime(&shutdown_time);
shutdown_time.tm_hour += 1;

rtc.scheduleTaskAt(&shutdown_time, [](){
    printf("系统即将关机...\n");
    system_shutdown();
}, "定时关机");
```

## 🔧 硬件连接

```
ESP32     RX8130
------    ------
GPIO8  -> SDA
GPIO9  -> SCL  
GPIO5  -> INT (可选)
3.3V   -> VCC
GND    -> GND
```

## ⚠️ 注意事项

1. **时间格式**：使用标准tm结构，年份需要减1900
2. **内存管理**：任务完成后会自动清理，无需手动释放
3. **中断安全**：任务回调在独立线程执行，可以安全使用printf等函数
4. **时间同步**：调用setTime()会自动重新计算所有任务时间

## 🐛 常见问题

**Q: 任务不执行？**
A: 检查RTC时间是否正确设置，中断引脚是否正确连接

**Q: 如何调试任务？**
A: 使用 `rtc.printTasksStatus()` 查看详细任务状态

**Q: 如何处理大量任务？**
A: 定期调用 `rtc.getActiveTaskCount()` 监控，必要时取消不需要的任务

**Q: 时间跳变怎么办？**
A: 调用 `rtc.setTime()` 会自动处理任务时间重计算

/*
 * SPDX-FileCopyrightText: 2025 ESP-IDF Community
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ic_rx8130_task_scheduler_example.cpp
 * @brief RX8130 RTC任务调度器完整功能示例
 * 
 * 本示例展示了ic_rx8130_tools的所有功能：
 * - RTC时间设置和读取
 * - 中断引脚配置
 * - 任务调度系统（延时任务、定时任务、重复任务）
 * - 任务管理（查询、取消、状态打印）
 * - 时间同步和任务重计算
 */

#pragma once

/**
 * @brief RX8130 RTC任务调度器示例函数声明
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 完整的RX8130功能演示
 * 
 * 展示RTC的所有功能，包括：
 * - 时间设置和读取
 * - 任务调度系统
 * - 任务管理和查询
 * - 时间同步机制
 * 
 * 运行时间约3.5分钟，包含详细的日志输出
 */
void ic_rx8130_example_main(void);

/**
 * @brief 快速功能测试
 * 
 * 简化版本的功能测试，适合快速验证硬件连接
 * 和基本功能。运行时间约10秒。
 */
void ic_rx8130_quick_test(void);

#ifdef __cplusplus
}
#endif

#include "ic_rx8130.h"
#include "i2c_bus_tools.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <time.h>

static const char* TAG = "RX8130_EXAMPLE";

// 全局RTC对象
ic_rx8130_tools rtc;

// 示例任务计数器
static int sensor_reading_count = 0;
static int heartbeat_count = 0;
static int backup_count = 0;

/**
 * @brief 传感器读取任务示例
 */
void sensor_reading_task()
{
    sensor_reading_count++;
    ESP_LOGI(TAG, "[Task] Sensor reading #%d - Temperature: 25.%d°C", 
             sensor_reading_count, sensor_reading_count % 10);
}

/**
 * @brief 心跳任务示例
 */
void heartbeat_task()
{
    heartbeat_count++;
    ESP_LOGI(TAG, "[Task] Heartbeat #%d - System is alive!", heartbeat_count);
}

/**
 * @brief 数据备份任务示例
 */
void data_backup_task()
{
    backup_count++;
    ESP_LOGI(TAG, "[Task] Data backup #%d - Saving to flash...", backup_count);
}

/**
 * @brief 一次性系统初始化任务
 */
void system_init_task()
{
    ESP_LOGI(TAG, "[Task] System initialization completed!");
    ESP_LOGI(TAG, "     - WiFi connected");
    ESP_LOGI(TAG, "     - Sensors calibrated");
    ESP_LOGI(TAG, "     - Memory initialized");
}

/**
 * @brief 定时报告任务
 */
void status_report_task()
{
    ESP_LOGI(TAG, "[Task] Status Report:");
    ESP_LOGI(TAG, "     - Sensor readings: %d", sensor_reading_count);
    ESP_LOGI(TAG, "     - Heartbeats sent: %d", heartbeat_count);
    ESP_LOGI(TAG, "     - Backups created: %d", backup_count);
    ESP_LOGI(TAG, "     - Active tasks: %zu", rtc.getActiveTaskCount());
}

/**
 * @brief 设置初始时间
 */
void setup_initial_time()
{
    struct tm initial_time = {0};
    initial_time.tm_year = 2025 - 1900;  // 年份从1900开始计算
    initial_time.tm_mon = 8;             // 月份从0开始，8表示9月
    initial_time.tm_mday = 2;            // 日
    initial_time.tm_hour = 14;           // 时
    initial_time.tm_min = 30;            // 分
    initial_time.tm_sec = 0;             // 秒
    initial_time.tm_wday = 1;            // 星期一
    
    rtc.setTime(&initial_time);
    
    struct tm current_time;
    rtc.getTime(&current_time);
    ESP_LOGI(TAG, "RTC time set to: %04d-%02d-%02d %02d:%02d:%02d",
             current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
             current_time.tm_hour, current_time.tm_min, current_time.tm_sec);
}

/**
 * @brief 创建示例任务
 */
void create_example_tasks()
{
    ESP_LOGI(TAG, "Creating example tasks...");
    
    // 1. 延时任务 - 5秒后执行系统初始化
    rtc_task_handle init_task = rtc.scheduleTask(5, system_init_task, "System initialization");
    ESP_LOGI(TAG, "Created init task (ID: %u) - will execute in 5 seconds", init_task.task_id);
    
    // 2. 重复任务 - 每10秒读取传感器数据
    rtc_task_handle sensor_task = rtc.scheduleRepeatingTask(10, sensor_reading_task, "Sensor data reading");
    ESP_LOGI(TAG, "Created repeating sensor task (ID: %u) - every 10 seconds", sensor_task.task_id);
    
    // 3. 重复任务 - 每15秒发送心跳
    rtc_task_handle heartbeat_task_handle = rtc.scheduleRepeatingTask(15, heartbeat_task, "Network heartbeat");
    ESP_LOGI(TAG, "Created repeating heartbeat task (ID: %u) - every 15 seconds", heartbeat_task_handle.task_id);
    
    // 4. 指定时间的任务 - 1分钟后执行数据备份
    struct tm backup_time;
    rtc.getTime(&backup_time);
    backup_time.tm_min += 1;  // 1分钟后
    mktime(&backup_time);     // 规范化时间
    
    rtc_task_handle backup_task = rtc.scheduleTaskAt(&backup_time, data_backup_task, "Data backup");
    ESP_LOGI(TAG, "Created scheduled backup task (ID: %u) - at %02d:%02d", 
             backup_task.task_id, backup_time.tm_hour, backup_time.tm_min);
    
    // 5. 重复任务 - 每30秒生成状态报告
    rtc_task_handle report_task = rtc.scheduleRepeatingTask(30, status_report_task, "System status report");
    ESP_LOGI(TAG, "Created repeating report task (ID: %u) - every 30 seconds", report_task.task_id);
    
    ESP_LOGI(TAG, "Total tasks created: %zu", rtc.getActiveTaskCount());
}

/**
 * @brief 演示任务查询功能
 */
void demonstrate_task_query()
{
    ESP_LOGI(TAG, "\n=== Task Query Demonstration ===");
    
    // 获取所有任务信息
    auto all_tasks = rtc.getAllTasksInfo();
    ESP_LOGI(TAG, "Found %zu active tasks:", all_tasks.size());
    
    for (const auto& task_info : all_tasks) {
        time_t time_to_exec = task_info.target_time - rtc.getCurrentTime();
        ESP_LOGI(TAG, "  Task ID: %u", task_info.handle.task_id);
        ESP_LOGI(TAG, "     Description: %s", task_info.description.c_str());
        ESP_LOGI(TAG, "     Type: %s", task_info.is_repeating ? "Repeating" : "One-time");
        if (task_info.is_repeating) {
            ESP_LOGI(TAG, "     Interval: %u seconds", task_info.interval_seconds);
        }
        ESP_LOGI(TAG, "     Time to execution: %ld seconds", time_to_exec > 0 ? time_to_exec : 0);
    }
    
    // 获取所有任务句柄
    auto handles = rtc.getActiveTaskHandles();
    ESP_LOGI(TAG, "Active task handles count: %zu", handles.size());
}

/**
 * @brief 演示任务管理功能
 */
void demonstrate_task_management()
{
    ESP_LOGI(TAG, "\n=== Task Management Demonstration ===");
    
    // 等待一段时间让一些任务执行
    ESP_LOGI(TAG, "Waiting 20 seconds to let some tasks execute...");
    vTaskDelay(pdMS_TO_TICKS(20000));
    
    // 打印当前任务状态
    rtc.printTasksStatus();
    
    // 演示取消任务
    auto handles = rtc.getActiveTaskHandles();
    if (handles.size() > 2) {
        ESP_LOGI(TAG, "Canceling task ID: %u", handles[1].task_id);
        if (rtc.cancelTask(handles[1])) {
            ESP_LOGI(TAG, "Task canceled successfully");
        } else {
            ESP_LOGI(TAG, "Failed to cancel task");
        }
        
        ESP_LOGI(TAG, "Active tasks after cancellation: %zu", rtc.getActiveTaskCount());
    }
}

/**
 * @brief 演示时间同步功能
 */
void demonstrate_time_sync()
{
    ESP_LOGI(TAG, "\n=== Time Synchronization Demonstration ===");
    
    ESP_LOGI(TAG, "Tasks before time change:");
    rtc.printTasksStatus();
    
    // 模拟时间调整（快进5分钟）
    struct tm new_time;
    rtc.getTime(&new_time);
    new_time.tm_min += 5;  // 快进5分钟
    mktime(&new_time);     // 规范化时间
    
    ESP_LOGI(TAG, "Adjusting time forward by 5 minutes...");
    rtc.setTime(&new_time);
    
    struct tm current_time;
    rtc.getTime(&current_time);
    ESP_LOGI(TAG, "New RTC time: %04d-%02d-%02d %02d:%02d:%02d",
             current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
             current_time.tm_hour, current_time.tm_min, current_time.tm_sec);
    
    ESP_LOGI(TAG, "Tasks after time synchronization:");
    rtc.printTasksStatus();
}

/**
 * @brief 主示例函数
 */
void ic_rx8130_example_main()
{
    ESP_LOGI(TAG, "\n=== RX8130 Task Scheduler Example Started ===");
    
    // 1. 初始化I2C总线
    i2c_bus_tools i2c_bus;
    if (!i2c_bus.init(I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9, 400000)) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    
    // 2. 初始化RX8130（使用GPIO5作为中断引脚）
    if (!rtc.init(i2c_bus.get_bus_handle(), 0x32, GPIO_NUM_5)) {
        ESP_LOGE(TAG, "Failed to initialize RX8130");
        return;
    }
    ESP_LOGI(TAG, "RX8130 initialized successfully with interrupt on GPIO5");
    
    // 3. 初始化电池备份
    rtc.initBat();
    ESP_LOGI(TAG, "Battery backup initialized");
    
    // 4. 设置初始时间
    setup_initial_time();
    
    // 5. 创建示例任务
    create_example_tasks();
    
    // 6. 演示任务查询功能
    demonstrate_task_query();
    
    // 7. 演示任务管理功能
    demonstrate_task_management();
    
    // 8. 演示时间同步功能
    demonstrate_time_sync();
    
    // 9. 持续监控任务执行
    ESP_LOGI(TAG, "\n=== Continuous Task Monitoring ===");
    ESP_LOGI(TAG, "Monitoring task execution for 2 minutes...");
    
    for (int i = 0; i < 12; i++) {  // 12 * 10秒 = 2分钟
        vTaskDelay(pdMS_TO_TICKS(10000));  // 等待10秒
        
        ESP_LOGI(TAG, "[%d0s] Active tasks: %zu", (i + 1), rtc.getActiveTaskCount());
        
        // 每30秒打印一次详细状态
        if ((i + 1) % 3 == 0) {
            rtc.printTasksStatus();
        }
    }
    
    // 10. 清理所有任务
    ESP_LOGI(TAG, "\n=== Cleanup ===");
    ESP_LOGI(TAG, "Canceling all remaining tasks...");
    rtc.cancelAllTasks();
    ESP_LOGI(TAG, "All tasks canceled. Active tasks: %zu", rtc.getActiveTaskCount());
    
    ESP_LOGI(TAG, "\n=== RX8130 Task Scheduler Example Completed ===");
    ESP_LOGI(TAG, "Final Statistics:");
    ESP_LOGI(TAG, "   - Sensor readings: %d", sensor_reading_count);
    ESP_LOGI(TAG, "   - Heartbeats sent: %d", heartbeat_count);
    ESP_LOGI(TAG, "   - Backups created: %d", backup_count);
}

/**
 * @brief 简化的示例入口（用于快速测试）
 */
void ic_rx8130_quick_test()
{
    ESP_LOGI(TAG, "Quick Test Started");
    
    // 初始化I2C和RTC
    i2c_bus_tools i2c_bus;
    i2c_bus.init(I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9, 400000);
    rtc.init(i2c_bus.get_bus_handle(), 0x32, GPIO_NUM_5);
    rtc.initBat();
    
    // 设置时间
    setup_initial_time();
    
    // 创建简单任务
    rtc.scheduleTask(3, []() { ESP_LOGI(TAG, "Quick task 1 executed!"); }, "Quick test 1");
    rtc.scheduleTask(6, []() { ESP_LOGI(TAG, "Quick task 2 executed!"); }, "Quick test 2");
    rtc.scheduleRepeatingTask(10, []() { ESP_LOGI(TAG, "Repeating task executed!"); }, "Repeating test");
    
    // 监控10秒
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "[%ds] Active tasks: %zu", i + 1, rtc.getActiveTaskCount());
    }
    
    rtc.cancelAllTasks();
    ESP_LOGI(TAG, "Quick test completed");
}

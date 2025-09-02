/*
 * SPDX-FileCopyrightText: 2025 ESP-IDF Community
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <driver/i2c_master.h>
#include "i2c_bus.h"
#include <driver/gpio.h>
#include <time.h>
#include <functional>
#include <vector>
#include <memory>
#include <string>
// https://download.epsondevice.com/td/pdf/app/RX8130CE_en.pdf
// https://github.com/alexreinert/piVCCU/blob/master/kernel/rtc-rx8130.c

// 任务类型定义
typedef std::function<void()> task_callback_t;

// 任务句柄类型
struct rtc_task_handle {
    uint32_t task_id;
    rtc_task_handle(uint32_t id = 0) : task_id(id) {}
    bool operator==(const rtc_task_handle& other) const { return task_id == other.task_id; }
    bool operator!=(const rtc_task_handle& other) const { return task_id != other.task_id; }
    bool isValid() const { return task_id != 0; }
};

// 任务信息结构体（用于查询任务状态）
struct rtc_task_info {
    rtc_task_handle handle;     // 任务句柄
    time_t target_time;         // 目标执行时间
    uint32_t interval_seconds;  // 重复间隔（0表示一次性）
    bool is_active;            // 是否激活
    time_t created_time;       // 创建时间
    bool is_repeating;         // 是否重复任务
    std::string description;   // 任务说明
};

// 任务结构体
struct rtc_task {
    uint32_t task_id;           // 任务ID
    time_t target_time;         // 目标执行时间（Unix时间戳）
    uint32_t interval_seconds;  // 重复间隔（0表示一次性任务）
    task_callback_t callback;   // 回调函数
    bool is_active;            // 任务是否激活
    time_t created_time;       // 任务创建时间（用于排序）
    std::string description;   // 任务说明（可选）
    
    rtc_task(uint32_t id, time_t target, uint32_t interval, task_callback_t cb, const std::string& desc = "") 
        : task_id(id), target_time(target), interval_seconds(interval), 
          callback(cb), is_active(true), created_time(time(NULL)), description(desc) {}
};

class ic_rx8130_tools {
public:
    ic_rx8130_tools()
    {
        _i2c_device_handle = NULL;
        _i2c_bus_device_handle = NULL;
        _int_pin = GPIO_NUM_NC;
        _interrupt_enabled = false;
        _interrupt_callback = nullptr;
        _next_task_id = 1;
        _last_rtc_time = 0;
    }

    bool init(i2c_master_bus_handle_t busHandle, uint8_t addr = 0x32, gpio_num_t int_pin = GPIO_NUM_NC);
    bool init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *i2c_device, gpio_num_t int_pin = GPIO_NUM_NC);
    bool init(i2c_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device, gpio_num_t int_pin = GPIO_NUM_NC);
    void initBat();
    void setTime(struct tm* time);         // 设置时间（会重新计算所有任务）
    void getTime(struct tm* time);
    void clearIrqFlags();
    void disableIrq();
    void setAlarmIrq(struct tm* time);
    void setTimerIrq(uint16_t seconds);
    
    // 用户任务系统API
    rtc_task_handle scheduleTask(uint32_t delay_seconds, task_callback_t callback, const std::string& description = "");  // 延时执行任务
    rtc_task_handle scheduleTaskAt(struct tm* target_time, task_callback_t callback, const std::string& description = ""); // 指定时间执行任务
    rtc_task_handle scheduleRepeatingTask(uint32_t interval_seconds, task_callback_t callback, const std::string& description = ""); // 重复执行任务
    rtc_task_handle scheduleRepeatingTaskAt(struct tm* start_time, uint32_t interval_seconds, task_callback_t callback, const std::string& description = ""); // 指定时间开始的重复任务
    
    bool cancelTask(rtc_task_handle handle);     // 取消任务
    void cancelAllTasks();                       // 取消所有任务
    size_t getActiveTaskCount();                 // 获取活跃任务数量
    
    // 任务查询API
    bool getTaskInfo(rtc_task_handle handle, rtc_task_info& info);  // 获取任务信息
    std::vector<rtc_task_info> getAllTasksInfo();                  // 获取所有任务信息
    std::vector<rtc_task_handle> getActiveTaskHandles();           // 获取所有活跃任务句柄
    void printTasksStatus();                                       // 打印任务状态（调试用）
    
    // 手动触发任务检查（用于RTC引脚连接不同的情况）
    void manualTriggerCheck(uint32_t tolerance_seconds = 1);       // 手动触发任务检查，默认允许1秒误差

protected:
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);
    void readRegister(uint8_t reg, uint8_t* buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t* buf, uint8_t len);
    
    // 中断相关保护方法
    bool setupInterrupt(gpio_num_t int_pin);
    
    // 私有回调系统API
    void setInterruptCallback(task_callback_t callback);
    void clearInterruptCallback();
    void enableInterrupt();
    void disableInterrupt();
    
    // 任务系统私有方法
    void updateNextAlarm();                // 更新下一个闹钟时间
    void processTriggeredTasks();          // 处理触发的任务
    void processTriggeredTasksWithTolerance(uint32_t tolerance_seconds); // 处理触发的任务（带容错）
    void recalculateAllTasks(time_t time_offset); // 重新计算所有任务时间
    time_t getCurrentTime();               // 获取当前RTC时间

public:
    // 让ISR函数可以访问私有成员
    task_callback_t _interrupt_callback;
    
    // 静态回调任务函数（需要公开访问）
    static void callback_task(void* param);

private:
    i2c_master_dev_handle_t _i2c_device_handle;
    i2c_bus_device_handle_t _i2c_bus_device_handle;
    gpio_num_t _int_pin;
    bool _interrupt_enabled;
    
    // 任务系统成员
    std::vector<std::shared_ptr<rtc_task>> _tasks;  // 任务列表
    uint32_t _next_task_id;                         // 下一个任务ID
    time_t _last_rtc_time;                          // 上次记录的RTC时间
};
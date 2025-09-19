/*
 * SPDX-FileCopyrightText: 2025 ESP-IDF Community
 *
 * SPDX-License-Identifier: MIT
 */
#include "ic_rx8130.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <algorithm>
#include <memory>

// RX-8130 Register definitions
#define RX8130_REG_SEC   0x10
#define RX8130_REG_MIN   0x11
#define RX8130_REG_HOUR  0x12
#define RX8130_REG_WDAY  0x13
#define RX8130_REG_MDAY  0x14
#define RX8130_REG_MONTH 0x15
#define RX8130_REG_YEAR  0x16

#define RX8130_REG_ALMIN   0x17
#define RX8130_REG_ALHOUR  0x18
#define RX8130_REG_ALWDAY  0x19
#define RX8130_REG_TCOUNT0 0x1A
#define RX8130_REG_TCOUNT1 0x1B
#define RX8130_REG_EXT     0x1C
#define RX8130_REG_FLAG    0x1D
#define RX8130_REG_CTRL0   0x1E
#define RX8130_REG_CTRL1   0x1F

#define RX8130_REG_END 0x23

// Extension Register (1Ch) bit positions
#define RX8130_BIT_EXT_TSEL (7 << 0)
#define RX8130_BIT_EXT_WADA (1 << 3)
#define RX8130_BIT_EXT_TE   (1 << 4)
#define RX8130_BIT_EXT_USEL (1 << 5)
#define RX8130_BIT_EXT_FSEL (3 << 6)

// Flag Register (1Dh) bit positions
#define RX8130_BIT_FLAG_VLF (1 << 1)
#define RX8130_BIT_FLAG_AF  (1 << 3)
#define RX8130_BIT_FLAG_TF  (1 << 4)
#define RX8130_BIT_FLAG_UF  (1 << 5)

// Control 0 Register (1Еh) bit positions
#define RX8130_BIT_CTRL_TSTP (1 << 2)
#define RX8130_BIT_CTRL_AIE  (1 << 3)
#define RX8130_BIT_CTRL_TIE  (1 << 4)
#define RX8130_BIT_CTRL_UIE  (1 << 5)
#define RX8130_BIT_CTRL_STOP (1 << 6)
#define RX8130_BIT_CTRL_TEST (1 << 7)

#define setbit(x, y)     x |= (0x01 << y)
#define clrbit(x, y)     x &= ~(0x01 << y)
#define reversebit(x, y) x ^= (0x01 << y)
#define getbit(x, y)     ((x) >> (y)&0x01)

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

// ISR处理函数（全局静态函数）
static void IRAM_ATTR rx8130_gpio_isr_handler(void* arg)
{
    // 这是ISR处理函数，这里先简单处理
    // 后续会添加任务系统的处理逻辑
    ic_rx8130_tools* rtc = (ic_rx8130_tools*)arg;
    if (rtc != NULL) {
        // 清除中断标志
        rtc->clearIrqFlags();
        
        // 如果设置了回调函数，创建任务来执行回调
        if (rtc->_interrupt_callback != nullptr) {
            // 创建一个临时任务来执行回调，避免在ISR中执行复杂操作
            xTaskCreate(ic_rx8130_tools::callback_task, "rtc_callback", 2048, (void*)rtc, 5, NULL);
        }
    }
}

bool ic_rx8130_tools::init(i2c_master_bus_handle_t busHandle, uint8_t addr, gpio_num_t int_pin)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &dev_cfg, &_i2c_device_handle));

    if (_i2c_device_handle == NULL) {
        return false;
    }

    // 配置中断引脚
    if (int_pin != GPIO_NUM_NC) {
        if (!setupInterrupt(int_pin)) {
            return false;
        }
    }

    return true;
}

bool ic_rx8130_tools::init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *i2c_device, gpio_num_t int_pin)
{
    if (i2c_device == NULL) {
        return false;
    }
    
    _i2c_device_handle = *i2c_device;
    
    if (_i2c_device_handle == NULL) {
        return false;
    }

    // 配置中断引脚
    if (int_pin != GPIO_NUM_NC) {
        if (!setupInterrupt(int_pin)) {
            return false;
        }
    }

    return true;
}

bool ic_rx8130_tools::init(i2c_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device, gpio_num_t int_pin)
{
    if (i2c_device == NULL) {
        return false;
    }
    
    _i2c_bus_device_handle = *i2c_device;
    
    if (_i2c_bus_device_handle == NULL) {
        return false;
    }

    // 配置中断引脚
    if (int_pin != GPIO_NUM_NC) {
        if (!setupInterrupt(int_pin)) {
            return false;
        }
    }

    return true;
}

void ic_rx8130_tools::initBat()
{
    auto data = readRegister8(0x1F);
    setbit(data, 4);
    setbit(data, 5);
    writeRegister8(0x1F, data);
    data = readRegister8(0x1F);
    printf("rtc bat init: 0x1F: %02X\n", data);
}

void ic_rx8130_tools::setTime(struct tm* time)
{
    // 记录当前时间和新时间，计算偏移量
    time_t old_time = getCurrentTime();
    
    uint8_t rbuf = 0;

    time->tm_year -= 100;

    // set STOP bit before changing clock/calendar
    rbuf = readRegister8(RX8130_REG_CTRL0);
    rbuf = rbuf | RX8130_BIT_CTRL_STOP;
    writeRegister8(RX8130_REG_CTRL0, rbuf);

    uint8_t date[7] = {dec2bcd(time->tm_sec),       dec2bcd(time->tm_min),  dec2bcd(time->tm_hour),
                       dec2bcd(time->tm_wday),      dec2bcd(time->tm_mday), dec2bcd(time->tm_mon),
                       dec2bcd(time->tm_year % 100)};

    writeRegister(RX8130_REG_SEC, date, 7);

    // clear STOP bit after changing clock/calendar
    rbuf = readRegister8(RX8130_REG_CTRL0);
    rbuf = rbuf & ~RX8130_BIT_CTRL_STOP;
    writeRegister8(RX8130_REG_CTRL0, rbuf);
    
    // 计算时间偏移并重新计算所有任务
    time_t new_time = getCurrentTime();
    time_t time_offset = new_time - old_time;
    
    if (time_offset != 0) {
        recalculateAllTasks(time_offset);
    }
}

void ic_rx8130_tools::getTime(struct tm* time)
{
    uint8_t date[7];
    readRegister(RX8130_REG_SEC, date, 7);

    time->tm_sec  = bcd2dec(date[RX8130_REG_SEC - 0x10] & 0x7f);
    time->tm_min  = bcd2dec(date[RX8130_REG_MIN - 0x10] & 0x7f);
    time->tm_hour = bcd2dec(date[RX8130_REG_HOUR - 0x10] & 0x3f);  // only 24-hour clock
    time->tm_mday = bcd2dec(date[RX8130_REG_MDAY - 0x10] & 0x3f);
    time->tm_mon  = bcd2dec(date[RX8130_REG_MONTH - 0x10] & 0x1f);
    time->tm_year = bcd2dec(date[RX8130_REG_YEAR - 0x10]);
    time->tm_wday = bcd2dec(date[RX8130_REG_WDAY - 0x10] & 0x7f);

    time->tm_year += 100;
}

void ic_rx8130_tools::clearIrqFlags()
{
    writeRegister8(RX8130_REG_FLAG, 0);
}

void ic_rx8130_tools::disableIrq()
{
    writeRegister8(RX8130_REG_CTRL0, 0);
}

void ic_rx8130_tools::setAlarmIrq(struct tm* time)
{
    uint8_t buf = 0;

    // Write 0 to AIE
    buf = readRegister8(RX8130_REG_CTRL0);
    clrbit(buf, 3);
    clrbit(buf, 5);
    writeRegister8(RX8130_REG_CTRL0, buf);

    buf = readRegister8(RX8130_REG_CTRL0);
    // debug_print_reg(0x1E, buf);

    // Hour AE, week AE day AE
    buf = 0x80;
    writeRegister8(RX8130_REG_ALWDAY, buf);
    buf = readRegister8(RX8130_REG_ALWDAY);
    // debug_print_reg(0x19, buf);

    buf = 0x80;
    writeRegister8(RX8130_REG_ALHOUR, buf);
    buf = readRegister8(RX8130_REG_ALHOUR);
    // debug_print_reg(0x18, buf);

    buf = 0x80;
    writeRegister8(RX8130_REG_ALMIN, buf);
    buf = readRegister8(RX8130_REG_ALMIN);
    // debug_print_reg(0x17, buf);

    // Write 1 to AIE
    buf = readRegister8(RX8130_REG_CTRL0);
    setbit(buf, 3);
    writeRegister8(RX8130_REG_CTRL0, buf);

    buf = readRegister8(RX8130_REG_CTRL0);
    // debug_print_reg(0x1E, buf);
}

// RX8130 寄存器地址
#define RX8130_REG_SEC                0x10
#define RX8130_REG_MIN                0x11
#define RX8130_REG_HOUR               0x12
#define RX8130_REG_WEEK               0x13
#define RX8130_REG_DAY                0x14
#define RX8130_REG_MONTH              0x15
#define RX8130_REG_YEAR               0x16
#define RX8130_REG_ALARM_MINUTE       0x17
#define RX8130_REG_ALARM_HOUR         0x18
#define RX8130_REG_ALARM_WEEKDAY      0x19
#define RX8130_REG_TIMER_COUNTER_LOW  0x1A
#define RX8130_REG_TIMER_COUNTER_HIGH 0x1B
#define RX8130_REG_EXTENSION          0x1C
#define RX8130_REG_FLAG               0x1D
#define RX8130_REG_CONTROL0           0x1E
#define RX8130_REG_CONTROL1           0x1F

void ic_rx8130_tools::setTimerIrq(uint16_t seconds)
{
    uint8_t flag_register = 0;
    uint8_t buffer[2]     = {0};
    buffer[0]             = seconds & 0xFF;         // 定时器低字节
    buffer[1]             = (seconds >> 8) & 0xFF;  // 定时器高字节

    // Step 1: Disable Timer
    flag_register = readRegister8(RX8130_REG_EXTENSION);

    flag_register &= ~(1 << 4);  // 禁用定时器 (TE = 0)
    writeRegister8(RX8130_REG_EXTENSION, flag_register);

    // Setp 2: Write Timer Counter Register (1Ah, 1Bh)
    writeRegister(RX8130_REG_TIMER_COUNTER_LOW, buffer, 2);

    // Step 3: Enable Timer
    flag_register = readRegister8(RX8130_REG_EXTENSION);

    setbit(flag_register, 4);  // 启用定时器 (TE = 1)
    clrbit(flag_register, 2);
    setbit(flag_register, 1);
    clrbit(flag_register, 0);
    writeRegister8(RX8130_REG_EXTENSION, flag_register);

    // Step 4: Enable Timer Interrupt
    flag_register = readRegister8(RX8130_REG_CONTROL0);
    flag_register |= (1 << 4);  // 启用定时器中断 (TIE = 1)
    writeRegister8(RX8130_REG_CONTROL0, flag_register);
}

// 私有回调系统API实现（原来的公共API现在是私有的）
void ic_rx8130_tools::setInterruptCallback(task_callback_t callback)
{
    _interrupt_callback = callback;
}

void ic_rx8130_tools::clearInterruptCallback()
{
    _interrupt_callback = nullptr;
}

void ic_rx8130_tools::enableInterrupt()
{
    if (_int_pin != GPIO_NUM_NC) {
        gpio_intr_enable(_int_pin);
        _interrupt_enabled = true;
    }
}

void ic_rx8130_tools::disableInterrupt()
{
    if (_int_pin != GPIO_NUM_NC) {
        gpio_intr_disable(_int_pin);
        _interrupt_enabled = false;
    }
}

uint8_t ic_rx8130_tools::readRegister8(uint8_t reg)
{
    uint8_t value;
    readRegister(reg, &value, 1);
    return value;
}

void ic_rx8130_tools::writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t buf[1] = {value};
    writeRegister(reg, buf, 1);
}

void ic_rx8130_tools::readRegister(uint8_t reg, uint8_t* buf, uint8_t len)
{
    if (_i2c_bus_device_handle) {
        i2c_bus_read_bytes(_i2c_bus_device_handle, reg, len, buf);
    } else {
        uint8_t w_buffer[1] = {0};
        w_buffer[0]         = reg;
        i2c_master_transmit_receive(_i2c_device_handle, w_buffer, 1, buf, len, portMAX_DELAY);
    }
}

void ic_rx8130_tools::writeRegister(uint8_t reg, uint8_t* buf, uint8_t len)
{
    if (_i2c_bus_device_handle) {
        i2c_bus_write_bytes(_i2c_bus_device_handle, reg, len, buf);
    } else {
        uint8_t w_buffer[1 + len];
        w_buffer[0] = reg;
        memcpy(w_buffer + 1, buf, len);
        i2c_master_transmit(_i2c_device_handle, w_buffer, 1 + len, portMAX_DELAY);
    }
}

bool ic_rx8130_tools::setupInterrupt(gpio_num_t int_pin)
{
    _int_pin = int_pin;
    
    // 配置GPIO为输入，启用内部上拉
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << int_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
        .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
#endif

    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return false;
    }
    
    // 安装GPIO ISR服务
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already installed, which is OK
        return false;
    }
    
    // 添加ISR处理函数
    ret = gpio_isr_handler_add(int_pin, rx8130_gpio_isr_handler, (void*)this);
    if (ret != ESP_OK) {
        return false;
    }
    
    _interrupt_enabled = true;
    return true;
}

// 静态回调任务函数
void ic_rx8130_tools::callback_task(void* param)
{
    ic_rx8130_tools* rtc = (ic_rx8130_tools*)param;
    if (rtc != NULL && rtc->_interrupt_callback != nullptr) {
        // 执行用户设置的回调函数
        rtc->_interrupt_callback();
    }
    
    // 任务执行完成后删除自己
    vTaskDelete(NULL);
}

// ============= 任务系统实现 =============

// 获取当前RTC时间（Unix时间戳）
time_t ic_rx8130_tools::getCurrentTime()
{
    struct tm current_tm;
    getTime(&current_tm);
    return mktime(&current_tm);
}

// 延时执行任务
rtc_task_handle ic_rx8130_tools::scheduleTask(uint32_t delay_seconds, task_callback_t callback, const std::string& description)
{
    time_t current_time = getCurrentTime();
    time_t target_time = current_time + delay_seconds;
    
    auto task = std::make_shared<rtc_task>(_next_task_id++, target_time, 0, callback, description);
    _tasks.push_back(task);
    
    updateNextAlarm();
    return rtc_task_handle(task->task_id);
}

// 指定时间执行任务
rtc_task_handle ic_rx8130_tools::scheduleTaskAt(struct tm* target_time, task_callback_t callback, const std::string& description)
{
    time_t target_timestamp = mktime(target_time);
    
    auto task = std::make_shared<rtc_task>(_next_task_id++, target_timestamp, 0, callback, description);
    _tasks.push_back(task);
    
    updateNextAlarm();
    return rtc_task_handle(task->task_id);
}

// 重复执行任务
rtc_task_handle ic_rx8130_tools::scheduleRepeatingTask(uint32_t interval_seconds, task_callback_t callback, const std::string& description)
{
    time_t current_time = getCurrentTime();
    time_t target_time = current_time + interval_seconds;
    
    auto task = std::make_shared<rtc_task>(_next_task_id++, target_time, interval_seconds, callback, description);
    _tasks.push_back(task);
    
    updateNextAlarm();
    return rtc_task_handle(task->task_id);
}

// 指定时间开始的重复任务
rtc_task_handle ic_rx8130_tools::scheduleRepeatingTaskAt(struct tm* start_time, uint32_t interval_seconds, task_callback_t callback, const std::string& description)
{
    time_t start_timestamp = mktime(start_time);
    
    auto task = std::make_shared<rtc_task>(_next_task_id++, start_timestamp, interval_seconds, callback, description);
    _tasks.push_back(task);
    
    updateNextAlarm();
    return rtc_task_handle(task->task_id);
}

// 取消任务
bool ic_rx8130_tools::cancelTask(rtc_task_handle handle)
{
    for (auto& task : _tasks) {
        if (task->task_id == handle.task_id && task->is_active) {
            task->is_active = false;
            updateNextAlarm();
            return true;
        }
    }
    return false;
}

// 取消所有任务
void ic_rx8130_tools::cancelAllTasks()
{
    for (auto& task : _tasks) {
        task->is_active = false;
    }
    _tasks.clear();
    clearIrqFlags();
    disableIrq();
}

// 获取活跃任务数量
size_t ic_rx8130_tools::getActiveTaskCount()
{
    size_t count = 0;
    for (const auto& task : _tasks) {
        if (task->is_active) {
            count++;
        }
    }
    return count;
}

// 更新下一个闹钟时间
void ic_rx8130_tools::updateNextAlarm()
{
    time_t current_time = getCurrentTime();
    time_t next_alarm_time = 0;
    
    // 找到最近的任务时间
    for (const auto& task : _tasks) {
        if (task->is_active && task->target_time > current_time) {
            if (next_alarm_time == 0 || task->target_time < next_alarm_time) {
                next_alarm_time = task->target_time;
            }
        }
    }
    
    if (next_alarm_time > 0) {
        // 计算需要等待的秒数
        uint32_t seconds_to_wait = (uint32_t)(next_alarm_time - current_time);
        
        // 设置定时器中断
        setTimerIrq(seconds_to_wait);
        
        // 设置处理函数
        setInterruptCallback([this]() {
            this->processTriggeredTasks();
        });
    } else {
        // 没有待执行任务，清除中断
        clearInterruptCallback();
        disableIrq();
    }
}

// 处理触发的任务
void ic_rx8130_tools::processTriggeredTasks()
{
    processTriggeredTasksWithTolerance(0); // 精确匹配，无容错
}

// 处理触发的任务（带容错）
void ic_rx8130_tools::processTriggeredTasksWithTolerance(uint32_t tolerance_seconds)
{
    time_t current_time = getCurrentTime();
    std::vector<std::shared_ptr<rtc_task>> triggered_tasks;
    
    // 收集所有应该执行的任务（考虑容错范围）
    for (auto& task : _tasks) {
        if (task->is_active) {
            // 计算任务时间与当前时间的差值
            time_t time_diff = (task->target_time > current_time) ? 
                              (task->target_time - current_time) : 
                              (current_time - task->target_time);
            
            // 如果任务时间已过或在容错范围内，则触发
            if (task->target_time <= current_time || time_diff <= tolerance_seconds) {
                triggered_tasks.push_back(task);
            }
        }
    }
    
    // 按创建时间排序（先创建的先执行）
    std::sort(triggered_tasks.begin(), triggered_tasks.end(), 
              [](const std::shared_ptr<rtc_task>& a, const std::shared_ptr<rtc_task>& b) {
                  return a->created_time < b->created_time;
              });
    
    // 依次执行任务
    for (auto& task : triggered_tasks) {
        if (task->callback) {
            printf("Executing task ID: %lu, Description: %s\n", (unsigned long)task->task_id, task->description.c_str());
            task->callback();
        }
        
        // 处理重复任务
        if (task->interval_seconds > 0) {
            task->target_time = current_time + task->interval_seconds;
        } else {
            // 一次性任务，标记为非活跃
            task->is_active = false;
        }
    }
    
    // 清理非活跃任务
    _tasks.erase(
        std::remove_if(_tasks.begin(), _tasks.end(),
                      [](const std::shared_ptr<rtc_task>& task) {
                          return !task->is_active;
                      }),
        _tasks.end());
    
    // 更新下一个闹钟
    updateNextAlarm();
}

// 重新计算所有任务时间（当RTC时间被修改时调用）
void ic_rx8130_tools::recalculateAllTasks(time_t time_offset)
{
    for (auto& task : _tasks) {
        if (task->is_active) {
            task->target_time += time_offset;
        }
    }
    updateNextAlarm();
}

// ============= 任务查询API实现 =============

// 获取任务信息
bool ic_rx8130_tools::getTaskInfo(rtc_task_handle handle, rtc_task_info& info)
{
    for (const auto& task : _tasks) {
        if (task->task_id == handle.task_id && task->is_active) {
            info.handle = handle;
            info.target_time = task->target_time;
            info.interval_seconds = task->interval_seconds;
            info.is_active = task->is_active;
            info.created_time = task->created_time;
            info.is_repeating = (task->interval_seconds > 0);
            info.description = task->description;
            return true;
        }
    }
    return false;
}

// 获取所有任务信息
std::vector<rtc_task_info> ic_rx8130_tools::getAllTasksInfo()
{
    std::vector<rtc_task_info> info_list;
    
    for (const auto& task : _tasks) {
        if (task->is_active) {
            rtc_task_info info;
            info.handle = rtc_task_handle(task->task_id);
            info.target_time = task->target_time;
            info.interval_seconds = task->interval_seconds;
            info.is_active = task->is_active;
            info.created_time = task->created_time;
            info.is_repeating = (task->interval_seconds > 0);
            info.description = task->description;
            info_list.push_back(info);
        }
    }
    
    return info_list;
}

// 获取所有活跃任务句柄
std::vector<rtc_task_handle> ic_rx8130_tools::getActiveTaskHandles()
{
    std::vector<rtc_task_handle> handles;
    
    for (const auto& task : _tasks) {
        if (task->is_active) {
            handles.push_back(rtc_task_handle(task->task_id));
        }
    }
    
    return handles;
}

// 打印任务状态（调试用）
void ic_rx8130_tools::printTasksStatus()
{
    time_t current_time = getCurrentTime();
    printf("\n=== RTC Tasks Status ===\n");
    printf("Current time: %s", ctime(&current_time));
    printf("Active tasks count: %zu\n", getActiveTaskCount());
    
    if (_tasks.empty()) {
        printf("No tasks scheduled.\n");
        return;
    }
    
    printf("\nTask Details:\n");
    for (const auto& task : _tasks) {
        if (task->is_active) {
            printf("Task ID: %lu\n", (unsigned long)task->task_id);
            if (!task->description.empty()) {
                printf("  Description: %s\n", task->description.c_str());
            }
            printf("  Target time: %s", ctime(&task->target_time));
            printf("  Created time: %s", ctime(&task->created_time));
            printf("  Type: %s\n", (task->interval_seconds > 0) ? "Repeating" : "One-time");
            if (task->interval_seconds > 0) {
                printf("  Interval: %lu seconds\n", (unsigned long)task->interval_seconds);
            }
            
            time_t time_diff = task->target_time - current_time;
            if (time_diff > 0) {
                printf("  Time to execution: %lld seconds\n", (long long)time_diff);
            } else {
                printf("  Status: Ready to execute\n");
            }
            printf("  ---\n");
        }
    }
    printf("========================\n\n");
}

// 手动触发任务检查（用于RTC引脚连接不同的情况）
void ic_rx8130_tools::manualTriggerCheck(uint32_t tolerance_seconds)
{
    time_t current_time = getCurrentTime();
    printf("Manual trigger check at: %s", ctime(&current_time));
    printf("Tolerance: %lu seconds\n", (unsigned long)tolerance_seconds);
    
    // 检查是否有任务需要执行
    bool has_tasks_to_execute = false;
    for (const auto& task : _tasks) {
        if (task->is_active) {
            time_t time_diff = (task->target_time > current_time) ? 
                              (task->target_time - current_time) : 
                              (current_time - task->target_time);
            
            if (task->target_time <= current_time || time_diff <= tolerance_seconds) {
                has_tasks_to_execute = true;
                break;
            }
        }
    }
    
    if (has_tasks_to_execute) {
        printf("Found tasks within tolerance range, executing...\n");
        processTriggeredTasksWithTolerance(tolerance_seconds);
    } else {
        printf("No tasks found within tolerance range.\n");
        
        // 显示下一个最近的任务时间
        time_t next_task_time = 0;
        std::string next_task_desc;
        for (const auto& task : _tasks) {
            if (task->is_active && task->target_time > current_time) {
                if (next_task_time == 0 || task->target_time < next_task_time) {
                    next_task_time = task->target_time;
                    next_task_desc = task->description;
                }
            }
        }
        
        if (next_task_time > 0) {
            time_t time_to_next = next_task_time - current_time;
            printf("Next task: \"%s\" in %lld seconds at %s", 
                   next_task_desc.c_str(), (long long)time_to_next, ctime(&next_task_time));
        } else {
            printf("No scheduled tasks.\n");
        }
    }
}
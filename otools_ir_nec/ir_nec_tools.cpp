/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "ir_nec_tools.h"
#include "ir_nec_encoder.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "freertos/queue.h"

static const char *IR_TAG = "ir_nec_tools";

/*
    // 方法1: 使用GPIO直接控制EN引脚
    corep4.otool_tools.ir_nec.init(Pin_P4_IR_TX, Pin_P4_IR_RX, Pin_P4_IR_EN);

    // 方法2: 使用回调函数控制EN引脚（示例：其他芯片控制）
    corep4.otool_tools.ir_nec.init(Pin_P4_IR_TX, Pin_P4_IR_RX, [](bool enable){
        if (enable) {
            ESP_LOGI("IR_EN", "Enabling IR module via callback");
            corep4.pyb.pinMode(Pin_P4_IR_EN, OUTPUT);
            corep4.pyb.digitalWrite(Pin_P4_IR_EN, HIGH); // 直接控制PYB的EN引脚
        } else {
            ESP_LOGI("IR_EN", "Disabling IR module via callback");
            corep4.pyb.pinMode(Pin_P4_IR_EN, OUTPUT);
            corep4.pyb.digitalWrite(Pin_P4_IR_EN, LOW); // 直接控制PYB的EN引脚
        }
    });
*/


ir_nec_tools::~ir_nec_tools() {
    deinit();
}

esp_err_t ir_nec_tools::init() {
    // 使用默认配置调用重载版本
    return init(IR_NEC_DEFAULT_TX_GPIO, IR_NEC_DEFAULT_RX_GPIO, IR_NEC_DEFAULT_EN_GPIO);
}

esp_err_t ir_nec_tools::init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, gpio_num_t en_gpio_pin) {
    if (initialized) return ESP_OK;

    // 检查TX和RX都为-1的情况，这是无效配置
    if (tx_gpio_pin == (gpio_num_t)-1 && rx_gpio_pin == (gpio_num_t)-1) {
        ESP_LOGE(IR_TAG, "Configuration failed: both TX and RX GPIO are -1, at least one must be valid");
        return ESP_ERR_INVALID_ARG;
    }

    // 应用传入的配置
    resolution_hz = IR_NEC_DEFAULT_RESOLUTION_HZ;
    tx_gpio = tx_gpio_pin;
    rx_gpio = rx_gpio_pin;
    en_gpio = en_gpio_pin;

    // 配置TX GPIO引脚
    if (tx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring TX GPIO: %d", (int)tx_gpio);
        gpio_reset_pin(tx_gpio);
        gpio_set_direction(tx_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(tx_gpio, 0);  // 初始为低电平
    }

    // 配置RX GPIO引脚
    if (rx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring RX GPIO: %d", (int)rx_gpio);
        gpio_reset_pin(rx_gpio);
        gpio_set_direction(rx_gpio, GPIO_MODE_INPUT);
    }

    // 配置使能引脚（直接GPIO控制）
    if (en_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring EN GPIO: %d", (int)en_gpio);
        gpio_reset_pin(en_gpio);
        gpio_set_direction(en_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio, 1);  // 使能IR模块
    }

    // 初始化TX通道
    if (tx_gpio != (gpio_num_t)-1) {
        esp_err_t ret = init_tx_channel();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    // 初始化RX通道
    if (rx_gpio != (gpio_num_t)-1) {
        esp_err_t ret = init_rx_channel();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t ir_nec_tools::init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, ir_nec_enable_callback_t enable_cb) {
    if (initialized) return ESP_OK;

    // 检查TX和RX都为-1的情况，这是无效配置
    if (tx_gpio_pin == (gpio_num_t)-1 && rx_gpio_pin == (gpio_num_t)-1) {
        ESP_LOGE(IR_TAG, "Configuration failed: both TX and RX GPIO are -1, at least one must be valid");
        return ESP_ERR_INVALID_ARG;
    }

    // 应用传入的配置
    resolution_hz = IR_NEC_DEFAULT_RESOLUTION_HZ;
    tx_gpio = tx_gpio_pin;
    rx_gpio = rx_gpio_pin;
    en_gpio = (gpio_num_t)-1; // 使用回调函数时，不使用GPIO
    enable_callback = enable_cb;

    // 配置TX GPIO引脚
    if (tx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring TX GPIO: %d", (int)tx_gpio);
        gpio_reset_pin(tx_gpio);
        gpio_set_direction(tx_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(tx_gpio, 0);  // 初始为低电平
    }

    // 配置RX GPIO引脚
    if (rx_gpio != (gpio_num_t)-1) {
        ESP_LOGI(IR_TAG, "Configuring RX GPIO: %d", (int)rx_gpio);
        gpio_reset_pin(rx_gpio);
        gpio_set_direction(rx_gpio, GPIO_MODE_INPUT);
    }

    // 通过回调函数使能IR模块
    if (enable_callback) {
        ESP_LOGI(IR_TAG, "Enabling IR module via callback");
        enable_callback(true);
    }

    // 初始化TX通道
    if (tx_gpio != (gpio_num_t)-1) {
        esp_err_t ret = init_tx_channel();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    // 初始化RX通道
    if (rx_gpio != (gpio_num_t)-1) {
        esp_err_t ret = init_rx_channel();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t ir_nec_tools::init_tx_channel() {
    ESP_LOGI(IR_TAG, "create RMT TX channel gpio=%d res=%u", (int)tx_gpio, (unsigned)resolution_hz);
    rmt_tx_channel_config_t tx_channel_cfg = {
        .gpio_num = tx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        .flags = {
            .invert_out = true, // NEC 典型需要反相
        }
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_channel_cfg, &tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "new tx channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = 38000,
        .duty_cycle = 0.5f,
        .flags = {}
    };
    ret = rmt_apply_carrier(tx_channel, &carrier_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "apply carrier failed: %s", esp_err_to_name(ret));
        rmt_del_channel(tx_channel);
        return ret;
    }

    ir_nec_encoder_config_t nec_encoder_cfg = { .resolution = resolution_hz };
    ret = rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "new nec encoder failed: %s", esp_err_to_name(ret));
        rmt_del_channel(tx_channel);
        return ret;
    }

    ret = rmt_enable(tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "enable tx failed: %s", esp_err_to_name(ret));
        rmt_del_encoder(nec_encoder);
        rmt_del_channel(tx_channel);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ir_nec_tools::init_rx_channel() {
    ESP_LOGI(IR_TAG, "create RMT RX channel gpio=%d res=%u", (int)rx_gpio, (unsigned)resolution_hz);
    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 64,
    };

    esp_err_t ret = rmt_new_rx_channel(&rx_channel_cfg, &rx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "new rx channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 创建接收队列
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if (!receive_queue) {
        ESP_LOGE(IR_TAG, "create receive queue failed");
        rmt_del_channel(rx_channel);
        return ESP_ERR_NO_MEM;
    }

    // 注册RX回调
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_done_callback,
    };
    ret = rmt_rx_register_event_callbacks(rx_channel, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "register rx callback failed: %s", esp_err_to_name(ret));
        vQueueDelete(receive_queue);
        rmt_del_channel(rx_channel);
        return ret;
    }

    ret = rmt_enable(rx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "enable rx failed: %s", esp_err_to_name(ret));
        vQueueDelete(receive_queue);
        rmt_del_channel(rx_channel);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ir_nec_tools::deinit() {
    if (!initialized) return ESP_OK;
    
    // 停止接收任务
    stop_receive();
    
    // 禁用IR模块
    if (en_gpio != (gpio_num_t)-1) {
        gpio_set_level(en_gpio, 0);  // 禁用IR模块
    } else if (enable_callback) {
        enable_callback(false);  // 通过回调函数禁用
    }
    
    if (nec_encoder) {
        rmt_del_encoder(nec_encoder);
        nec_encoder = nullptr;
    }
    if (tx_channel) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = nullptr;
    }
    if (rx_channel) {
        rmt_disable(rx_channel);
        rmt_del_channel(rx_channel);
        rx_channel = nullptr;
    }
    if (receive_queue) {
        vQueueDelete(receive_queue);
        receive_queue = nullptr;
    }
    
    initialized = false;
    return ESP_OK;
}

esp_err_t ir_nec_tools::send(uint16_t address, uint16_t command) {
    if (!initialized) {
        esp_err_t err = init();
        if (err != ESP_OK) return err;
    }
    
    if (tx_channel == nullptr) {
        ESP_LOGE(IR_TAG, "TX channel not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ir_nec_scan_code_t scan_code = { .address = address, .command = command };
    rmt_transmit_config_t transmit_config = { 
        .loop_count = 0,
        .flags = {}
    };
    return rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config);
}

void ir_nec_tools::set_receive_callback(ir_nec_receive_callback_t callback) {
    receive_callback = callback;
}

void ir_nec_tools::set_enable_callback(ir_nec_enable_callback_t callback) {
    enable_callback = callback;
}

esp_err_t ir_nec_tools::start_receive() {
    if (!initialized) {
        esp_err_t err = init();
        if (err != ESP_OK) return err;
    }
    
    if (rx_channel == nullptr) {
        ESP_LOGE(IR_TAG, "RX channel not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (rx_task_handle != nullptr) {
        ESP_LOGW(IR_TAG, "RX task already running");
        return ESP_OK;
    }
    
    // 重置退出标志
    rx_task_should_exit = false;
    
    // 创建接收任务，增加栈大小
    BaseType_t ret = xTaskCreate(rx_task, "ir_rx_task", 4096, this, 5, &rx_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(IR_TAG, "create rx task failed");
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t ir_nec_tools::stop_receive() {
    if (rx_task_handle) {
        // 设置退出标志
        rx_task_should_exit = true;
        
        // 等待任务自然退出
        int timeout_count = 0;
        while (rx_task_handle != nullptr && timeout_count < 100) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout_count++;
        }
        
        // 如果任务仍然存在，强制删除
        if (rx_task_handle != nullptr) {
            ESP_LOGW(IR_TAG, "Force deleting RX task");
            vTaskDelete(rx_task_handle);
            rx_task_handle = nullptr;
        }
    }
    return ESP_OK;
}

bool ir_nec_tools::rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    ir_nec_tools *instance = static_cast<ir_nec_tools*>(user_data);
    
    // 发送接收到的数据到队列
    xQueueSendFromISR(instance->receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void ir_nec_tools::rx_task(void *pvParameters) {
    ir_nec_tools *instance = static_cast<ir_nec_tools*>(pvParameters);
    
    // 动态分配接收缓冲区以减少栈使用
    rmt_symbol_word_t *raw_symbols = (rmt_symbol_word_t*)malloc(64 * sizeof(rmt_symbol_word_t));
    if (!raw_symbols) {
        ESP_LOGE(IR_TAG, "Failed to allocate memory for raw_symbols");
        vTaskDelete(NULL);
        return;
    }
    
    rmt_rx_done_event_data_t rx_data;
    
    // 配置接收参数
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // 最短560us，1250ns < 560us
        .signal_range_max_ns = 12000000, // 最长9000us，12000000ns > 9000us
    };
    
    // 开始第一次接收
    esp_err_t ret = rmt_receive(instance->rx_channel, raw_symbols, 64 * sizeof(rmt_symbol_word_t), &receive_config);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "start receive failed: %s", esp_err_to_name(ret));
        free(raw_symbols);
        vTaskDelete(NULL);
        return;
    }
    
    while (!instance->rx_task_should_exit) {
        // 等待接收完成，使用较短的超时以便能及时响应退出信号
        if (xQueueReceive(instance->receive_queue, &rx_data, pdMS_TO_TICKS(100)) == pdPASS) {
            // 解析接收到的数据
            if (instance->parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols)) {
                // 解析成功，调用回调函数
                if (instance->receive_callback) {
                    instance->receive_callback(instance->last_address, instance->last_command, false);
                }
            }
            
            // 重新开始接收
            if (!instance->rx_task_should_exit) {
                ret = rmt_receive(instance->rx_channel, raw_symbols, 64 * sizeof(rmt_symbol_word_t), &receive_config);
                if (ret != ESP_OK) {
                    ESP_LOGE(IR_TAG, "restart receive failed: %s", esp_err_to_name(ret));
                    break;
                }
            }
        }
    }
    
    free(raw_symbols);
    instance->rx_task_handle = nullptr;
    vTaskDelete(NULL);
}

bool ir_nec_tools::parse_nec_frame(rmt_symbol_word_t *symbols, size_t symbol_num) {
    switch (symbol_num) {
    case 34: // NEC 正常帧
        {
            rmt_symbol_word_t *cur = symbols;
            uint16_t address = 0;
            uint16_t command = 0;
            
            // 检查前导码
            bool valid_leading_code = check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                                      check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
            if (!valid_leading_code) {
                return false;
            }
            cur++;
            
            // 解析16位地址
            for (int i = 0; i < 16; i++) {
                if (parse_logic1(cur)) {
                    address |= 1 << i;
                } else if (parse_logic0(cur)) {
                    address &= ~(1 << i);
                } else {
                    return false;
                }
                cur++;
            }
            
            // 解析16位命令
            for (int i = 0; i < 16; i++) {
                if (parse_logic1(cur)) {
                    command |= 1 << i;
                } else if (parse_logic0(cur)) {
                    command &= ~(1 << i);
                } else {
                    return false;
                }
                cur++;
            }
            
            // 保存地址和命令
            last_address = address;
            last_command = command;
            return true;
        }
    case 2: // NEC 重复帧
        if (parse_repeat_frame(symbols)) {
            if (receive_callback) {
                receive_callback(last_address, last_command, true);
            }
            return true;
        }
        break;
    default:
        ESP_LOGW(IR_TAG, "Unknown NEC frame with %d symbols", symbol_num);
        break;
    }
    return false;
}

bool ir_nec_tools::check_in_range(uint32_t signal_duration, uint32_t spec_duration) {
    return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

bool ir_nec_tools::parse_logic0(rmt_symbol_word_t *symbol) {
    return check_in_range(symbol->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           check_in_range(symbol->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

bool ir_nec_tools::parse_logic1(rmt_symbol_word_t *symbol) {
    return check_in_range(symbol->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           check_in_range(symbol->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

bool ir_nec_tools::parse_repeat_frame(rmt_symbol_word_t *symbol) {
    return check_in_range(symbol->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           check_in_range(symbol->duration1, NEC_REPEAT_CODE_DURATION_1);
}

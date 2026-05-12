/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#include "ir_tools.h"

#include <cstdlib>
#include <utility>

#include "driver/gpio.h"
#include "esp_check.h"
#include "freertos/queue.h"

static const char *IR_TAG = "ir_tools";

static void log_default_frame(const ir_decode_result_t &frame) {
    const char *repeat_suffix = frame.is_repeat ? " (repeat)" : "";
    switch (frame.format) {
        case ir_format_t::NEC:
            ESP_LOGI("IR_RX", "[NEC] addr=0x%04X cmd=0x%04X%s",
                     static_cast<unsigned>(frame.address & 0xFFFFu),
                     static_cast<unsigned>(frame.command & 0xFFFFu),
                     repeat_suffix);
            break;
        case ir_format_t::RC5: {
            const uint32_t toggle = (frame.raw_value >> 11) & 0x1u;
            ESP_LOGI("IR_RX", "[RC5] addr=0x%02X cmd=0x%02X toggle=%u%s",
                     static_cast<unsigned>(frame.address & 0x1Fu),
                     static_cast<unsigned>(frame.command & 0x3Fu),
                     static_cast<unsigned>(toggle),
                     repeat_suffix);
            break;
        }
        default:
            ESP_LOGI("IR_RX", "[%s] addr=0x%08X cmd=0x%08X%s",
                     ir_format_name(frame.format),
                     static_cast<unsigned>(frame.address),
                     static_cast<unsigned>(frame.command),
                     repeat_suffix);
            break;
    }
}

ir_tools::~ir_tools() {
    deinit();
    ir_decoder_destroy(decoder_ctx_);
    decoder_ctx_ = nullptr;
    ir_encoder_destroy(encoder_ctx_);
    encoder_ctx_ = nullptr;
}

esp_err_t ir_tools::init() {
    return init(IR_DEFAULT_TX_GPIO, IR_DEFAULT_RX_GPIO, IR_DEFAULT_EN_GPIO);
}

esp_err_t ir_tools::init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, gpio_num_t en_gpio_pin) {
    if (initialized_) {
        return ESP_OK;
    }

    if (tx_gpio_pin == IR_DEFAULT_TX_GPIO && rx_gpio_pin == IR_DEFAULT_RX_GPIO) {
        ESP_LOGE(IR_TAG, "Either TX or RX pin must be valid");
        return ESP_ERR_INVALID_ARG;
    }

    resolution_hz_ = IR_DEFAULT_RESOLUTION_HZ;
    tx_gpio_ = tx_gpio_pin;
    rx_gpio_ = rx_gpio_pin;
    en_gpio_ = en_gpio_pin;
    enable_callback_ = nullptr;

    if (tx_gpio_ != IR_DEFAULT_TX_GPIO) {
        gpio_reset_pin(tx_gpio_);
        gpio_set_direction(tx_gpio_, GPIO_MODE_OUTPUT);
        gpio_set_level(tx_gpio_, 0);
    }

    if (rx_gpio_ != IR_DEFAULT_RX_GPIO) {
        gpio_reset_pin(rx_gpio_);
        gpio_set_direction(rx_gpio_, GPIO_MODE_INPUT);
    }

    if (en_gpio_ != IR_DEFAULT_EN_GPIO) {
        gpio_reset_pin(en_gpio_);
        gpio_set_direction(en_gpio_, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio_, 1);
    }

    if (tx_gpio_ != IR_DEFAULT_TX_GPIO) {
        ESP_RETURN_ON_ERROR(init_tx_channel(), IR_TAG, "Failed to init TX channel");
        ESP_RETURN_ON_ERROR(ensure_encoder_context(), IR_TAG, "Failed to init encoder context");
    }

    if (rx_gpio_ != IR_DEFAULT_RX_GPIO) {
        ESP_RETURN_ON_ERROR(init_rx_channel(), IR_TAG, "Failed to init RX channel");
    }
    ESP_RETURN_ON_ERROR(ensure_decoder_context(), IR_TAG, "Failed to init decoder context");
    initialized_ = true;
    return ESP_OK;
}

esp_err_t ir_tools::init(gpio_num_t tx_gpio_pin, gpio_num_t rx_gpio_pin, ir_enable_callback_t enable_cb) {
    if (initialized_) {
        return ESP_OK;
    }

    if (tx_gpio_pin == IR_DEFAULT_TX_GPIO && rx_gpio_pin == IR_DEFAULT_RX_GPIO) {
        ESP_LOGE(IR_TAG, "Either TX or RX pin must be valid");
        return ESP_ERR_INVALID_ARG;
    }

    resolution_hz_ = IR_DEFAULT_RESOLUTION_HZ;
    tx_gpio_ = tx_gpio_pin;
    rx_gpio_ = rx_gpio_pin;
    en_gpio_ = IR_DEFAULT_EN_GPIO;
    enable_callback_ = enable_cb;

    if (tx_gpio_ != IR_DEFAULT_TX_GPIO) {
        gpio_reset_pin(tx_gpio_);
        gpio_set_direction(tx_gpio_, GPIO_MODE_OUTPUT);
        gpio_set_level(tx_gpio_, 0);
    }

    if (rx_gpio_ != IR_DEFAULT_RX_GPIO) {
        gpio_reset_pin(rx_gpio_);
        gpio_set_direction(rx_gpio_, GPIO_MODE_INPUT);
    }

    if (enable_callback_) {
        enable_callback_(true);
    }

    if (tx_gpio_ != IR_DEFAULT_TX_GPIO) {
        ESP_RETURN_ON_ERROR(init_tx_channel(), IR_TAG, "Failed to init TX channel");
        ESP_RETURN_ON_ERROR(ensure_encoder_context(), IR_TAG, "Failed to init encoder context");
    }

    if (rx_gpio_ != IR_DEFAULT_RX_GPIO) {
        ESP_RETURN_ON_ERROR(init_rx_channel(), IR_TAG, "Failed to init RX channel");
    }
    ESP_RETURN_ON_ERROR(ensure_decoder_context(), IR_TAG, "Failed to init decoder context");
    initialized_ = true;
    return ESP_OK;
}

esp_err_t ir_tools::deinit() {
    stop_receive();

    if (en_gpio_ != IR_DEFAULT_EN_GPIO) {
        gpio_set_level(en_gpio_, 0);
    } else if (enable_callback_) {
        enable_callback_(false);
    }

    if (tx_channel_) {
        rmt_disable(tx_channel_);
        rmt_del_channel(tx_channel_);
        tx_channel_ = nullptr;
    }
    if (rx_channel_) {
        rmt_disable(rx_channel_);
        rmt_del_channel(rx_channel_);
        rx_channel_ = nullptr;
    }
    if (receive_queue_) {
        vQueueDelete(receive_queue_);
        receive_queue_ = nullptr;
    }

    if (encoder_ctx_) {
        ir_encoder_destroy(encoder_ctx_);
        encoder_ctx_ = nullptr;
    }

    initialized_ = false;
    return ESP_OK;
}

esp_err_t ir_tools::start_receive() {
    if (!initialized_) {
        ESP_RETURN_ON_ERROR(init(), IR_TAG, "Failed to initialize before RX");
    }
    if (rx_channel_ == nullptr) {
        ESP_LOGE(IR_TAG, "RX channel not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (rx_task_handle_ != nullptr) {
        ESP_LOGW(IR_TAG, "RX task already running");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ensure_decoder_context(), IR_TAG, "Failed to ensure decoder context");
    rx_task_should_exit_ = false;
    BaseType_t ret = xTaskCreate(rx_task, "ir_rx_task", 4096, this, 5, &rx_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(IR_TAG, "Failed to create RX task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ir_tools::stop_receive() {
    if (rx_task_handle_) {
        rx_task_should_exit_ = true;
        int guard = 0;
        while (rx_task_handle_ && guard < 100) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ++guard;
        }
        if (rx_task_handle_) {
            ESP_LOGW(IR_TAG, "Force deleting RX task");
            vTaskDelete(rx_task_handle_);
            rx_task_handle_ = nullptr;
        }
    }
    return ESP_OK;
}

esp_err_t ir_tools::register_decoder(IrProtocolDecoderPtr decoder) {
    if (!decoder) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(ensure_decoder_context(), IR_TAG, "Decoder context not ready");
    return ir_decoder_register_decoder(decoder_ctx_, std::move(decoder));
}

void ir_tools::subscribe(ir_format_t format, ir_decode_callback_t callback) {
    if (ensure_decoder_context() != ESP_OK) {
        ESP_LOGE(IR_TAG, "Decoder context not available");
        return;
    }
    if (!callback) {
        callback = log_default_frame;
    }
    
    size_t old_count = ir_decoder_get_subscribed_count(decoder_ctx_);
    ir_decoder_subscribe(decoder_ctx_, format, std::move(callback));
    size_t new_count = ir_decoder_get_subscribed_count(decoder_ctx_);
    
    // 如果RX通道已初始化，根据订阅数量重新配置载波
    if (rx_channel_) {
        reconfigure_rx_carrier();
        
        // 如果订阅数跨越阈值(1→2或2→1)，需要重启RX任务以应用新的signal_range_min_ns
        bool need_restart = (old_count <= 1 && new_count > 1) || (old_count > 1 && new_count <= 1);
        if (need_restart && rx_task_handle_) {
            ESP_LOGI(IR_TAG, "Restarting RX task due to protocol count change: %zu→%zu", old_count, new_count);
            stop_receive();
            start_receive();
        }
    }
}

void ir_tools::unsubscribe(ir_format_t format) {
    if (decoder_ctx_) {
        size_t old_count = ir_decoder_get_subscribed_count(decoder_ctx_);
        ir_decoder_unsubscribe(decoder_ctx_, format);
        size_t new_count = ir_decoder_get_subscribed_count(decoder_ctx_);
        
        // 如果RX通道已初始化，根据订阅数量重新配置载波
        if (rx_channel_) {
            reconfigure_rx_carrier();
            
            // 如果订阅数跨越阈值(2→1或1→0)，需要重启RX任务以应用新的signal_range_min_ns
            bool need_restart = (old_count <= 1 && new_count > 1) || (old_count > 1 && new_count <= 1);
            if (need_restart && rx_task_handle_) {
                ESP_LOGI(IR_TAG, "Restarting RX task due to protocol count change: %zu→%zu", old_count, new_count);
                stop_receive();
                start_receive();
            }
        }
    }
}

void ir_tools::set_default_callback(ir_decode_callback_t callback) {
    if (ensure_decoder_context() != ESP_OK) {
        ESP_LOGE(IR_TAG, "Decoder context not available");
        return;
    }
    ir_decoder_set_default_callback(decoder_ctx_, std::move(callback));
}

esp_err_t ir_tools::send(const ir_send_request_t &request) {
    if (!initialized_) {
        ESP_RETURN_ON_ERROR(init(), IR_TAG, "Failed to init before send");
    }
    if (tx_channel_ == nullptr) {
        ESP_LOGE(IR_TAG, "TX channel not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(ensure_encoder_context(), IR_TAG, "Encoder context not available");
    
    // 根据协议动态设置载波频率
    if (request.format == ir_format_t::RC5) {
        // RC5不使用载波调制（参考esp32-rmt-ir）
        esp_err_t ret = rmt_apply_carrier(tx_channel_, nullptr);
        if (ret != ESP_OK) {
            ESP_LOGW(IR_TAG, "Failed to disable carrier for RC5: %s", esp_err_to_name(ret));
        }
    } else {
        // 其他协议使用载波
        uint32_t carrier_freq = 38000; // 默认38kHz
        if (request.format == ir_format_t::NEC) {
            carrier_freq = 38000;
        }
        
        rmt_carrier_config_t carrier_cfg = {
            .frequency_hz = carrier_freq,
            .duty_cycle = 0.5f,
            .flags = {}
        };
        esp_err_t ret = rmt_apply_carrier(tx_channel_, &carrier_cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(IR_TAG, "Failed to apply %ukHz carrier: %s",
                     static_cast<unsigned>(carrier_freq / 1000),
                     esp_err_to_name(ret));
        }
    }
    
    return ir_encoder_send(encoder_ctx_, tx_channel_, request);
}

esp_err_t ir_tools::init_tx_channel() {
    ESP_LOGI(IR_TAG, "create RMT TX channel gpio=%d res=%u", static_cast<int>(tx_gpio_), static_cast<unsigned>(resolution_hz_));
    rmt_tx_channel_config_t tx_channel_cfg = {};
    tx_channel_cfg.gpio_num          = tx_gpio_;
    tx_channel_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_channel_cfg.resolution_hz     = resolution_hz_;
    tx_channel_cfg.mem_block_symbols = 64;
    tx_channel_cfg.trans_queue_depth = 2;
    tx_channel_cfg.flags.invert_out  = true;

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel_), IR_TAG, "new tx channel failed");

    rmt_carrier_config_t carrier_cfg = {};
    carrier_cfg.frequency_hz = 38000;
    carrier_cfg.duty_cycle   = 0.5f;
    esp_err_t ret = rmt_apply_carrier(tx_channel_, &carrier_cfg);
    if (ret != ESP_OK) {
        rmt_del_channel(tx_channel_);
        tx_channel_ = nullptr;
        ESP_LOGE(IR_TAG, "apply carrier failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rmt_enable(tx_channel_);
    if (ret != ESP_OK) {
        rmt_del_channel(tx_channel_);
        tx_channel_ = nullptr;
        ESP_LOGE(IR_TAG, "enable tx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t ir_tools::init_rx_channel() {
    ESP_LOGI(IR_TAG, "create RMT RX channel gpio=%d res=%u", static_cast<int>(rx_gpio_), static_cast<unsigned>(resolution_hz_));
    size_t subscribed_count = ir_decoder_get_subscribed_count(decoder_ctx_);
    // Each ESP32-P4 RMT block stores 64 symbols; cap total blocks to avoid exhausting group slots
    uint32_t rx_mem_symbols = (subscribed_count <= 1) ? 128 : 512;
    rmt_rx_channel_config_t rx_channel_cfg = {};
    rx_channel_cfg.gpio_num          = rx_gpio_;
    rx_channel_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rx_channel_cfg.resolution_hz     = resolution_hz_;
    rx_channel_cfg.mem_block_symbols = rx_mem_symbols;

    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel_), IR_TAG,
                        "new rx channel failed (mem_symbols=%u)", static_cast<unsigned>(rx_mem_symbols));
    
    // 根据订阅的协议数量配置载波解调
    configure_rx_carrier_for_channel();

    receive_queue_ = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if (!receive_queue_) {
        ESP_LOGE(IR_TAG, "create receive queue failed");
        rmt_del_channel(rx_channel_);
        rx_channel_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_done_callback,
    };
    esp_err_t ret = rmt_rx_register_event_callbacks(rx_channel_, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "register rx callback failed: %s", esp_err_to_name(ret));
        vQueueDelete(receive_queue_);
        receive_queue_ = nullptr;
        rmt_del_channel(rx_channel_);
        rx_channel_ = nullptr;
        return ret;
    }

    ret = rmt_enable(rx_channel_);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "enable rx failed: %s", esp_err_to_name(ret));
        vQueueDelete(receive_queue_);
        receive_queue_ = nullptr;
        rmt_del_channel(rx_channel_);
        rx_channel_ = nullptr;
        return ret;
    }

    return ESP_OK;
}

esp_err_t ir_tools::configure_rx_carrier_for_channel() {
    if (!rx_channel_) {
        return ESP_ERR_INVALID_STATE;
    }
    
    size_t subscribed_count = ir_decoder_get_subscribed_count(decoder_ctx_);
    
    if (subscribed_count <= 1) {
        // 单协议: 根据订阅的协议配置对应载波频率
        ir_format_t format = ir_decoder_get_first_subscribed_format(decoder_ctx_);
        
        if (format == ir_format_t::RC5) {
            // RC5不使用载波解调（参考esp32-rmt-ir）
            ESP_LOGI(IR_TAG, "RX carrier disabled for RC5 (baseband mode)");
            return rmt_apply_carrier(rx_channel_, nullptr);
        } else {
            // 其他协议使用载波解调
            uint32_t carrier_freq = 38000; // 默认38kHz
            if (format == ir_format_t::NEC) {
                carrier_freq = 38000;
            }
            
            rmt_carrier_config_t carrier_cfg = {};
            carrier_cfg.frequency_hz = carrier_freq;
            carrier_cfg.duty_cycle   = 0.33f;
            carrier_cfg.flags.polarity_active_low = false;
            ESP_LOGI(IR_TAG, "RX carrier enabled: %ukHz for %s",
                     static_cast<unsigned>(carrier_freq / 1000),
                     ir_format_name(format));
            return rmt_apply_carrier(rx_channel_, &carrier_cfg);
        }
    } else {
        // 多协议: 禁用载波解调，直接读取基带波形
        ESP_LOGI(IR_TAG, "RX carrier disabled (baseband mode for %zu protocols)", subscribed_count);
        return rmt_apply_carrier(rx_channel_, nullptr);
    }
}

esp_err_t ir_tools::reconfigure_rx_carrier() {
    if (!rx_channel_) {
        return ESP_OK;  // RX未初始化，无需配置
    }
    
    // 禁用通道进行重新配置
    esp_err_t ret = rmt_disable(rx_channel_);
    if (ret != ESP_OK) {
        ESP_LOGW(IR_TAG, "Failed to disable RX for reconfiguration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 重新配置载波
    ret = configure_rx_carrier_for_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "Failed to reconfigure RX carrier: %s", esp_err_to_name(ret));
    }
    
    // 重新使能通道
    esp_err_t enable_ret = rmt_enable(rx_channel_);
    if (enable_ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "Failed to re-enable RX: %s", esp_err_to_name(enable_ret));
        return enable_ret;
    }
    
    return ret;
}

bool ir_tools::rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    auto *instance = static_cast<ir_tools *>(user_data);
    xQueueSendFromISR(instance->receive_queue_, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void ir_tools::rx_task(void *pvParameters) {
    auto *instance = static_cast<ir_tools *>(pvParameters);
    // 根据订阅数量动态设置接收参数
    size_t subscribed_count = ir_decoder_get_subscribed_count(instance->decoder_ctx_);
    // 基带模式需要更大的缓冲区以捕获完整载波波形，即使在单协议下硬件也可能输出未完全解调的波形
    const size_t symbol_buffer_len = (subscribed_count <= 1) ? 512 : 2048;

    rmt_symbol_word_t *raw_symbols = static_cast<rmt_symbol_word_t *>(malloc(symbol_buffer_len * sizeof(rmt_symbol_word_t)));
    if (!raw_symbols) {
        ESP_LOGE(IR_TAG, "Failed to allocate memory for raw symbols (len=%zu)", symbol_buffer_len);
        vTaskDelete(nullptr);
        return;
    }

    // 硬件限制: signal_range_min_ns 最大约3187ns
    // 单协议(载波解调): 使用1250ns捕获解调后的精确脉冲
    // 多协议(基带模式): 使用3000ns尽量过滤单个载波周期(36kHz≈27.8us, 但硬件限制无法完全过滤)
    uint32_t min_ns = (subscribed_count <= 1) ? 1250 : 3000;
    
    rmt_receive_config_t receive_config = {};
    receive_config.signal_range_min_ns = min_ns;
    receive_config.signal_range_max_ns = 30000000; // allow RC5 (~25ms) to be captured fully
    
    ESP_LOGI(IR_TAG, "RX task started with min_ns=%u (subscribed=%zu, buffer=%zu symbols)", 
             static_cast<unsigned>(min_ns), subscribed_count, symbol_buffer_len);

    rmt_rx_done_event_data_t rx_data;
    esp_err_t ret = rmt_receive(instance->rx_channel_, raw_symbols, symbol_buffer_len * sizeof(rmt_symbol_word_t), &receive_config);
    if (ret != ESP_OK) {
        ESP_LOGE(IR_TAG, "start receive failed: %s", esp_err_to_name(ret));
        free(raw_symbols);
        vTaskDelete(nullptr);
        return;
    }

    while (!instance->rx_task_should_exit_) {
        if (xQueueReceive(instance->receive_queue_, &rx_data, pdMS_TO_TICKS(100)) == pdPASS) {
            instance->handle_received_symbols(rx_data.received_symbols, rx_data.num_symbols);
            if (!instance->rx_task_should_exit_) {
                ret = rmt_receive(instance->rx_channel_, raw_symbols, symbol_buffer_len * sizeof(rmt_symbol_word_t), &receive_config);
                if (ret != ESP_OK) {
                    ESP_LOGE(IR_TAG, "restart receive failed: %s", esp_err_to_name(ret));
                    break;
                }
            }
        }
    }

    free(raw_symbols);
    instance->rx_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

bool ir_tools::handle_received_symbols(const rmt_symbol_word_t *symbols, size_t symbol_num) {
    if (ensure_decoder_context() != ESP_OK) {
        ESP_LOGE(IR_TAG, "Decoder context not available");
        return false;
    }
    return ir_decoder_handle_symbols(decoder_ctx_, symbols, symbol_num);
}

esp_err_t ir_tools::ensure_decoder_context() {
    if (decoder_ctx_) {
        return ESP_OK;
    }
    decoder_ctx_ = ir_decoder_create();
    if (!decoder_ctx_) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ir_tools::ensure_encoder_context() {
    if (!encoder_ctx_) {
        encoder_ctx_ = ir_encoder_create(resolution_hz_);
        if (!encoder_ctx_) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        ir_encoder_set_resolution(encoder_ctx_, resolution_hz_);
    }
    return ESP_OK;
}

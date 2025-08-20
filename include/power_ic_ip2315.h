/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __POWER_IC_IP2315_H__
#define __POWER_IC_IP2315_H__

#include <stdint.h>
#include "esp_err.h"
#include "i2c_bus.h"

#ifndef I2C_IP2315_ADDR
#define I2C_IP2315_ADDR 0x75
#endif

/**
 * @brief IP2315 电源管理芯片（充电管理）I2C 封装类
 * 
 * 说明：寄存器地址与位定义需参考数据手册进行校准。本类先提供通用读写接口与
 *       一些典型高层操作的占位实现，便于后续按手册补齐。
 */
class power_ic_ip2315 {
public:
	// 充电输入电压档位（决定写哪个 ISET 寄存器）
	enum charge_input_profile_t {
		CHG_IN_5V,
		CHG_IN_7V,
		CHG_IN_9V,
		CHG_IN_12V,
	};

	// ISET 量化步进选择：输入侧 25mA/LSB 或电池侧 50mA/LSB
	// 后缀明确标注 _25ma / _50ma 以便理解
	enum iset_scale_t {
		ISET_SCALE_IN_25ma = 0,  // 输入路径量化：25 mA 每 LSB (Ichg = ISET * 0.025A)
		ISET_SCALE_BAT_50ma      // 电池侧量化：50 mA 每 LSB (Ichg = ISET * 0.05A)
	};

public:
	/**
	 * @brief 构造函数
	 * @param addr I2C 地址（默认 0x75）
	 * @param bus_freq_hz I2C 频率（默认 400k）
	 */
	explicit power_ic_ip2315(uint8_t addr = I2C_IP2315_ADDR, uint32_t bus_freq_hz = 400000);
	~power_ic_ip2315();

	/**
	 * @brief 初始化：基于外部 I2C Master Bus 创建设备句柄
	 * @param bus I2C 主总线句柄
	 */
	esp_err_t init(i2c_master_bus_handle_t bus);
    esp_err_t init(i2c_master_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device);

	/**
	 * @brief 去初始化：释放设备句柄
	 */
	esp_err_t deinit();

	// 低层寄存器操作（单字节寄存器 + 简单的16位读取辅助）
	esp_err_t read_reg(uint8_t reg, uint8_t *val) const;
	esp_err_t write_reg(uint8_t reg, uint8_t val) const;
	esp_err_t update_bits(uint8_t reg, uint8_t mask, uint8_t value) const;
	esp_err_t read_u16_be(uint8_t reg_msb, uint16_t &out) const;

	// 高层功能（需按手册完善映射）
	esp_err_t enable_charging(bool enable);
	// 指定档位与量化：
	// scale=ISET_SCALE_IN_25ma 表示按输入侧 25 mA/LSB；
	// scale=ISET_SCALE_BAT_50ma 表示按电池侧 50 mA/LSB。
	esp_err_t set_charge_current_ma(uint16_t ma, charge_input_profile_t profile, iset_scale_t scale = ISET_SCALE_IN_25ma);

	/**
	 * @brief 5V 输入时选择是否“恒定 BAT 电流”通路
	 * @param use_constant_bat true: 置 0x20[1]=0（选择恒定 BAT 电流）；false: 置 0x20[1]=1（非恒定 BAT 电流）
	 */
	esp_err_t set_5v_bat_current_mode(bool use_constant_bat);
	esp_err_t get_status(uint8_t &status) const;
	esp_err_t get_fault(uint8_t &fault) const;
	// 预留/不确定功能已移除，避免误用
	esp_err_t dump_registers(uint8_t start = 0x00, uint8_t end = 0x3F) const;

	/**
	 * @brief 设置是否通过 ICHGSET 引脚设定充电电流
	 * true: 使能引脚设定（0x1E[5:3] = 101）
	 * false: 禁用引脚，改为寄存器设定（0x1E[5:3] = 000 且 0x74[6] = 1）
	 */
	esp_err_t set_ichgset_pin_mode(bool enable_pin);

	// 基本属性
	i2c_bus_device_handle_t device() const;
	uint8_t address() const;
	bool is_initialized() const;

private:
	esp_err_t read_reg_safely(uint8_t reg, uint8_t &val) const;
	esp_err_t ensure_register_current_control();
	esp_err_t configure_ichgset_pin_mode(bool enable_pin);

private:
	i2c_master_bus_handle_t _bus;
	i2c_bus_device_handle_t _dev;
	uint8_t _addr;
	uint32_t _bus_freq;
	bool _initialized;
	bool _owns_dev;
};

#endif // __POWER_IC_IP2315_H__


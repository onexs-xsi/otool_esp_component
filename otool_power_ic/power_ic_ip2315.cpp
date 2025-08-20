/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

// IP2315 power IC helper (class-form) – generic I2C wrapper and basic ops
// Register map portions implemented per provided datasheet excerpts.

#include "power_ic_ip2315.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ip2315";

// Some static analyzers require CONFIG_LOG_MAXIMUM_LEVEL to be defined when expanding ESP_LOGx.
// In ESP-IDF builds, it's provided by sdkconfig.h. Add a safe fallback for static checks.
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif

#ifndef ESP_LOG_LEVEL
#define ESP_LOG_LEVEL(level, tag, fmt, ...) ((void)0)
#endif

#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#endif

// --- Optional guessed/default register addresses (placeholders) ---
// Please replace with the actual register map from the datasheet.
// Keeping them in one place helps later maintenance.
enum : uint8_t {
	IP2315_REG_SYS_CTL1      = 0x01, // SYS_CTL1: bit0 charger enable (0: disable, 1: enable)
	IP2315_REG_CHG_MODE0     = 0x20, // bit1 used at 5V to select constant BAT current path (0 selects constant BAT current)
	IP2315_REG_ICHSET_CTL    = 0x1E, // ICHSET_CTL: [5:3] select ICHGSET pin or register control
	IP2315_REG_MISC2         = 0x74, // [6] used with 0x1E[5:3]=000 to enable register control per doc
	// Current set by input profile (ISET at 5V/7V/9V/12V); bits [6:0], but [6] reserved => use [5:0]
	IP2315_REG_CHG_ISET_5V   = 0x26, // ISET in 25mA steps (input path), 50mA steps at BAT side
	IP2315_REG_CHG_ISET_7V   = 0x28,
	IP2315_REG_CHG_ISET_9V   = 0x29,
	IP2315_REG_CHG_ISET_12V  = 0x2A,
	// Charge status register per datasheet snapshot: bit6=Chg_end (0: charging, 1: full)
	IP2315_REG_CHG_STAT    = 0xC7,
};

power_ic_ip2315::power_ic_ip2315(uint8_t addr, uint32_t bus_freq_hz)
	: _bus(nullptr), _dev(nullptr), _addr(addr), _bus_freq(bus_freq_hz), _initialized(false), _owns_dev(false) {}

power_ic_ip2315::~power_ic_ip2315()
{
	// Ensure resources are released
	(void)deinit();
}

esp_err_t power_ic_ip2315::init(i2c_master_bus_handle_t bus)
{
	if (_initialized) {
		ESP_LOGW(TAG, "IP2315 already initialized @0x%02X", _addr);
		return ESP_OK;
	}
	if (bus == nullptr) {
		ESP_LOGE(TAG, "Invalid I2C bus handle");
		return ESP_ERR_INVALID_ARG;
	}

	_bus = bus;
	// Create a device handle bound to the provided I2C master bus
	_dev = i2c_bus_device_create(_bus, _addr, _bus_freq);
	if (_dev == nullptr) {
		ESP_LOGE(TAG, "Failed to create I2C device for IP2315 @0x%02X", _addr);
		return ESP_FAIL;
	}
	_owns_dev = true;

	_initialized = true;
	ESP_LOGI(TAG, "IP2315 init done (addr=0x%02X, freq=%lu)", _addr, (unsigned long)_bus_freq);
	return ESP_OK;
}

esp_err_t power_ic_ip2315::init(i2c_master_bus_handle_t bus, i2c_bus_device_handle_t *i2c_device)
{
	if (_initialized) {
		ESP_LOGW(TAG, "IP2315 already initialized @0x%02X", _addr);
		return ESP_OK;
	}
	if (bus == nullptr || i2c_device == nullptr || *i2c_device == nullptr) {
		ESP_LOGE(TAG, "Invalid I2C bus/device handle");
		return ESP_ERR_INVALID_ARG;
	}
	_bus = bus;
	_dev = *i2c_device; // 直接使用外部传入的设备句柄
	_owns_dev = false;  // 不接管设备生命周期
	_initialized = true;
	ESP_LOGI(TAG, "IP2315 init with external dev handle (addr=0x%02X)", _addr);
	return ESP_OK;
}

esp_err_t power_ic_ip2315::deinit()
{
	if (_dev && _owns_dev) {
		i2c_bus_device_delete(&_dev);
	}
	_dev = nullptr;
	_bus = nullptr;
	_initialized = false;
	_owns_dev = false;
	return ESP_OK;
}

// --- Low-level helpers ---

esp_err_t power_ic_ip2315::read_reg(uint8_t reg, uint8_t *val) const
{
	if (!_initialized || _dev == nullptr || val == nullptr) return ESP_ERR_INVALID_STATE;
	return i2c_bus_read_byte(_dev, reg, val);
}

esp_err_t power_ic_ip2315::write_reg(uint8_t reg, uint8_t val) const
{
	if (!_initialized || _dev == nullptr) return ESP_ERR_INVALID_STATE;
	return i2c_bus_write_byte(_dev, reg, val);
}

esp_err_t power_ic_ip2315::update_bits(uint8_t reg, uint8_t mask, uint8_t value) const
{
	if (!_initialized || _dev == nullptr) return ESP_ERR_INVALID_STATE;
	uint8_t v = 0;
	esp_err_t ret = i2c_bus_read_byte(_dev, reg, &v);
	if (ret != ESP_OK) return ret;
	v = (uint8_t)((v & ~mask) | (value & mask));
	return i2c_bus_write_byte(_dev, reg, v);
}

esp_err_t power_ic_ip2315::read_u16_be(uint8_t reg_msb, uint16_t &out) const
{
	if (!_initialized || _dev == nullptr) return ESP_ERR_INVALID_STATE;
	uint8_t hi = 0, lo = 0;
	esp_err_t ret = i2c_bus_read_byte(_dev, reg_msb, &hi);
	if (ret != ESP_OK) return ret;
	ret = i2c_bus_read_byte(_dev, (uint8_t)(reg_msb + 1), &lo);
	if (ret != ESP_OK) return ret;
	out = (uint16_t)((hi << 8) | lo);
	return ESP_OK;
}

esp_err_t power_ic_ip2315::read_reg_safely(uint8_t reg, uint8_t &val) const
{
	uint8_t tmp = 0;
	esp_err_t ret = ESP_OK;
	if (!_initialized || _dev == nullptr) return ESP_ERR_INVALID_STATE;
	ret = i2c_bus_read_byte(_dev, reg, &tmp);
	if (ret == ESP_OK) {
		val = tmp;
	}
	return ret;
}

// --- High-level operations (placeholders; wire up with real regs/bits later) ---

esp_err_t power_ic_ip2315::enable_charging(bool enable)
{
	// Per datasheet: SYS_CTL1 (0x01) bit0 controls charger enable: 0=disable, 1=enable
	constexpr uint8_t CHG_EN_MASK = 0x01; // bit0
	uint8_t set = enable ? CHG_EN_MASK : 0x00;
	ESP_LOGI(TAG, "%s charging", enable ? "Enable" : "Disable");
	return update_bits(IP2315_REG_SYS_CTL1, CHG_EN_MASK, set);
}

static inline uint8_t clamp6(uint32_t v) { return (uint8_t)((v > 0x3F) ? 0x3F : v); }

esp_err_t power_ic_ip2315::set_charge_current_ma(uint16_t ma, charge_input_profile_t profile, iset_scale_t scale)
{
	// Ensure using register control
	esp_err_t ret = ensure_register_current_control();
	if (ret != ESP_OK) return ret;

	uint8_t reg = IP2315_REG_CHG_ISET_5V;
	switch (profile) {
		case CHG_IN_5V:  reg = IP2315_REG_CHG_ISET_5V; break;
		case CHG_IN_7V:  reg = IP2315_REG_CHG_ISET_7V; break;
		case CHG_IN_9V:  reg = IP2315_REG_CHG_ISET_9V; break;
		case CHG_IN_12V: reg = IP2315_REG_CHG_ISET_12V; break;
		default:         reg = IP2315_REG_CHG_ISET_5V; break;
	}

	// Quantization per datasheet
	// input path: Ichg = ISET * 0.025A => LSB = 25mA
	// BAT side:   Ichg = ISET * 0.05A  => LSB = 50mA
	uint32_t step_ma = (scale == ISET_SCALE_BAT_50ma) ? 50u : 25u;
	uint32_t code = (ma + step_ma - 1) / step_ma; // round-up
	uint8_t iset6 = clamp6(code);

	ESP_LOGI(TAG, "Set ISET(%s,%s-scale): %u mA -> code=%u (0x%02X)",
			 (profile==CHG_IN_5V?"5V":profile==CHG_IN_7V?"7V":profile==CHG_IN_9V?"9V":"12V"),
			 (scale==ISET_SCALE_BAT_50ma)?"BAT":"IN",
			 (unsigned)ma, (unsigned)iset6, (unsigned)iset6);

	// Only bits [5:0] are ISET; bit7 reserved; bit6 noted reserved too—write only lower 6 bits
	return write_reg(reg, (uint8_t)(iset6 & 0x3F));
}

esp_err_t power_ic_ip2315::dump_registers(uint8_t start, uint8_t end) const
{
	if (!_initialized || _dev == nullptr) return ESP_ERR_INVALID_STATE;
	if (start > end) return ESP_ERR_INVALID_ARG;
	for (uint8_t r = start; r <= end; ++r) {
		uint8_t v = 0;
		esp_err_t ret = i2c_bus_read_byte(_dev, r, &v);
		if (ret == ESP_OK) {
			ESP_LOGI(TAG, "REG[0x%02X] = 0x%02X", r, v);
		} else {
			ESP_LOGW(TAG, "REG[0x%02X] read failed: %s", r, esp_err_to_name(ret));
		}
		// avoid watchdog in long loops
	vTaskDelay(1);
		if (r == 0xFF) break; // safety
	}
	return ESP_OK;
}

i2c_bus_device_handle_t power_ic_ip2315::device() const { return _dev; }
uint8_t power_ic_ip2315::address() const { return _addr; }
bool power_ic_ip2315::is_initialized() const { return _initialized; }

esp_err_t power_ic_ip2315::get_status(uint8_t &status) const
{
	// CHG_STAT @0xC7, bit6: Chg_end (0: charging, 1: full)
	uint8_t v = 0;
	esp_err_t ret = read_reg(IP2315_REG_CHG_STAT, &v);
	if (ret == ESP_OK) {
		status = v;
	}
	return ret;
}

esp_err_t power_ic_ip2315::set_5v_bat_current_mode(bool use_constant_bat)
{
	// For 5V input charging: 0x20[1]=0 selects constant BAT current
	const uint8_t mask = (uint8_t)(1u << 1);
	const uint8_t val  = use_constant_bat ? 0x00 : mask;
	ESP_LOGI(TAG, "5V BAT current mode: %s", use_constant_bat ? "constant BAT current" : "non-constant BAT current");
	return update_bits(IP2315_REG_CHG_MODE0, mask, val);
}

esp_err_t power_ic_ip2315::configure_ichgset_pin_mode(bool enable_pin)
{
	// 0x1E[5:3]:
	//  - 000: disable pin; use register to set charge current (requires 0x74[6]=1)
	//  - 101: enable pin; use ICHGSET pin to set charge current
	uint8_t v = 0;
	esp_err_t ret = read_reg(IP2315_REG_ICHSET_CTL, &v);
	if (ret != ESP_OK) return ret;
	v &= ~(uint8_t)(0x07 << 3);
	if (enable_pin) {
		v |= (uint8_t)(0x05 << 3); // 101b
	} else {
		v |= (uint8_t)(0x00 << 3); // 000b
	}
	ret = write_reg(IP2315_REG_ICHSET_CTL, v);
	if (ret != ESP_OK) return ret;
	if (!enable_pin) {
		// When disabling pin mode, doc says also set 0x74[6]=1 to select register control path
		// Keep other bits of 0x74 intact
		ret = update_bits(IP2315_REG_MISC2, (uint8_t)(1u << 6), (uint8_t)(1u << 6));
	}
	return ret;
}

esp_err_t power_ic_ip2315::set_ichgset_pin_mode(bool enable_pin)
{
	ESP_LOGI(TAG, "ICHGSET pin mode: %s", enable_pin ? "enable" : "disable");
	return configure_ichgset_pin_mode(enable_pin);
}

esp_err_t power_ic_ip2315::ensure_register_current_control()
{
	// Force to register current control (0x1E[5:3]=000 and 0x74[6]=1)
	esp_err_t ret = configure_ichgset_pin_mode(false);
	return ret;
}


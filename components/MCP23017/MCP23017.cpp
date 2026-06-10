#include "MCP23017.hpp"
#include "esp_log.h"

static const char *TAG = "MCP23017";

MCP23017::MCP23017(I2CBase &i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
    , _initialized(false) {}

uint8_t MCP23017::_pin_to_port_reg(uint8_t pin, Register reg_a) {
    Register reg = (pin < 8) ? reg_a : (Register)(reg_a + 1);
    return (uint8_t)reg;
}

esp_err_t MCP23017::init() {
    _initialized = true;
    ESP_LOGI(TAG, "Initialized (addr=0x%02X)", _addr);
    return ESP_OK;
}

esp_err_t MCP23017::pin_mode(uint8_t pin, bool output) {
    if (pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t reg = _pin_to_port_reg(pin, REG_IODIRA);
    uint8_t bit = pin & 0x07;

    uint8_t val;
    esp_err_t ret = _i2c.read_reg(_addr, reg, &val, 1);
    if (ret != ESP_OK) return ret;

    if (output) val &= ~(1 << bit);
    else val |= (1 << bit);

    return _i2c.write_reg(_addr, reg, &val, 1);
}

esp_err_t MCP23017::digital_write(uint8_t pin, bool level) {
    if (pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t reg = _pin_to_port_reg(pin, REG_GPIOA);
    uint8_t bit = pin & 0x07;

    uint8_t val;
    esp_err_t ret = _i2c.read_reg(_addr, reg, &val, 1);
    if (ret != ESP_OK) return ret;

    if (level) val |= (1 << bit);
    else val &= ~(1 << bit);

    return _i2c.write_reg(_addr, reg, &val, 1);
}

esp_err_t MCP23017::digital_read(uint8_t pin, bool &level) {
    if (pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t reg = _pin_to_port_reg(pin, REG_GPIOA);
    uint8_t bit = pin & 0x07;

    uint8_t val;
    esp_err_t ret = _i2c.read_reg(_addr, reg, &val, 1);
    if (ret != ESP_OK) return ret;

    level = (val >> bit) & 1;
    return ESP_OK;
}

esp_err_t MCP23017::port_write(uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return _i2c.write_reg(_addr, REG_GPIOA, buf, 2);
}

esp_err_t MCP23017::port_read(uint16_t &value) {
    uint8_t buf[2];
    esp_err_t ret = _i2c.read_reg(_addr, REG_GPIOA, buf, 2);
    if (ret == ESP_OK) {
        value = buf[0] | ((uint16_t)buf[1] << 8);
    }
    return ret;
}

esp_err_t MCP23017::port_set_mode(uint16_t direction) {
    uint8_t buf[2] = { (uint8_t)(direction & 0xFF), (uint8_t)(direction >> 8) };
    return _i2c.write_reg(_addr, REG_IODIRA, buf, 2);
}

esp_err_t MCP23017::set_pullup(uint8_t pin, bool enable) {
    if (pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t reg = _pin_to_port_reg(pin, REG_GPPUA);
    uint8_t bit = pin & 0x07;

    uint8_t val;
    esp_err_t ret = _i2c.read_reg(_addr, reg, &val, 1);
    if (ret != ESP_OK) return ret;

    if (enable) val |= (1 << bit);
    else val &= ~(1 << bit);

    return _i2c.write_reg(_addr, reg, &val, 1);
}

esp_err_t MCP23017::set_interrupt(uint8_t pin, bool enable) {
    if (pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t reg = _pin_to_port_reg(pin, REG_GPINTENA);
    uint8_t bit = pin & 0x07;

    uint8_t val;
    esp_err_t ret = _i2c.read_reg(_addr, reg, &val, 1);
    if (ret != ESP_OK) return ret;

    if (enable) val |= (1 << bit);
    else val &= ~(1 << bit);

    return _i2c.write_reg(_addr, reg, &val, 1);
}

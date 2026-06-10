#pragma once

#include <cstdint>
#include "I2CBase.hpp"
#include "esp_err.h"

#define MCP23017_DEFAULT_ADDR 0x20

class MCP23017 {
public:
    enum Register : uint8_t {
        REG_IODIRA   = 0x00,
        REG_IODIRB   = 0x01,
        REG_IPOLA    = 0x02,
        REG_IPOLB    = 0x03,
        REG_GPINTENA = 0x04,
        REG_GPINTENB = 0x05,
        REG_DEFVALA  = 0x06,
        REG_DEFVALB  = 0x07,
        REG_INTCONA  = 0x08,
        REG_INTCONB  = 0x09,
        REG_IOCON    = 0x0A,
        REG_IOCONB   = 0x0B,
        REG_GPPUA    = 0x0C,
        REG_GPPUB    = 0x0D,
        REG_INTFA    = 0x0E,
        REG_INTFB    = 0x0F,
        REG_INTCAPA  = 0x10,
        REG_INTCAPB  = 0x11,
        REG_GPIOA    = 0x12,
        REG_GPIOB    = 0x13,
        REG_OLATA    = 0x14,
        REG_OLATB    = 0x15,
    };

    MCP23017(I2CBase &i2c, uint8_t addr = MCP23017_DEFAULT_ADDR);

    esp_err_t init();

    esp_err_t pin_mode(uint8_t pin, bool output);
    esp_err_t digital_write(uint8_t pin, bool level);
    esp_err_t digital_read(uint8_t pin, bool &level);

    esp_err_t port_write(uint16_t value);
    esp_err_t port_read(uint16_t &value);
    esp_err_t port_set_mode(uint16_t direction);

    esp_err_t set_pullup(uint8_t pin, bool enable);
    esp_err_t set_interrupt(uint8_t pin, bool enable);

    bool is_initialized() const { return _initialized; }

private:
    static uint8_t _pin_to_port_reg(uint8_t pin, Register reg_a);

    I2CBase &_i2c;
    uint8_t _addr;
    bool _initialized;
};

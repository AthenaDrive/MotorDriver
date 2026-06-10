#pragma once

#include <cstdint>
#include "I2CBase.hpp"
#include "esp_err.h"

#define LM75A_DEFAULT_ADDR 0x48

class LM75A {
public:
    LM75A(I2CBase &i2c, uint8_t addr = LM75A_DEFAULT_ADDR);

    esp_err_t init();

    esp_err_t read_temperature(float &temp_c);

    esp_err_t set_shutdown(bool shutdown);
    esp_err_t set_os_mode(bool comparator_mode);
    esp_err_t set_os_polarity(bool active_high);
    esp_err_t set_fault_queue(uint8_t faults);

    bool is_initialized() const { return _initialized; }

private:
    static constexpr uint8_t REG_TEMP   = 0x00;
    static constexpr uint8_t REG_CONF   = 0x01;
    static constexpr uint8_t REG_THYST  = 0x02;
    static constexpr uint8_t REG_TOS    = 0x03;

    I2CBase &_i2c;
    uint8_t _addr;
    bool _initialized;
};

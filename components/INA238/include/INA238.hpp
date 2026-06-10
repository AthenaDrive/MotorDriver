#pragma once

#include <cstdint>
#include "I2CBase.hpp"
#include "esp_err.h"

#define INA238_DEFAULT_ADDR 0x40

class INA238 {
public:
    enum Register : uint8_t {
        REG_CONFIG       = 0x00,
        REG_ADC_CONFIG   = 0x01,
        REG_SHUNT_CAL    = 0x02,
        REG_VSHUNT       = 0x04,
        REG_VBUS         = 0x05,
        REG_DIETEMP      = 0x06,
        REG_CURRENT      = 0x07,
        REG_POWER        = 0x08,
        REG_DIAG_ALRT    = 0x0B,
        REG_SOVL         = 0x0C,
        REG_SUVL         = 0x0D,
        REG_BOVL         = 0x0E,
        REG_BUVL         = 0x0F,
        REG_TEMP_LIMIT   = 0x10,
        REG_PWR_LIMIT    = 0x11,
        REG_MANUF_ID     = 0x3E,
        REG_DEVICE_ID    = 0x3F,
    };

    enum ConfigBits : uint16_t {
        CONFIG_RST       = 0x8000,
        CONFIG_ADCRANGE  = 0x0010,
    };

    enum AdcConfigBits : uint16_t {
        MODE_CONT_ALL    = 0xF000,
        MODE_CONT_VBUS   = 0x9000,
        MODE_CONT_VSHUNT = 0xA000,
        VBUSCT_1052US   = 0x0A00,
        VSHCT_1052US    = 0x0280,
        VTCT_1052US     = 0x0028,
        AVG_1            = 0x0000,
        AVG_128          = 0x0004,
        AVG_256          = 0x0005,
        AVG_512          = 0x0006,
    };

    INA238(I2CBase &i2c, uint8_t addr = INA238_DEFAULT_ADDR);

    esp_err_t init();
    esp_err_t reset();

    esp_err_t set_adc_range(bool range_40mv);
    esp_err_t set_adc_config(uint16_t config);
    esp_err_t calibrate(float shunt_resistance, float max_current_a);

    esp_err_t read_shunt_voltage(float &voltage_mv);
    esp_err_t read_bus_voltage(float &voltage_v);
    esp_err_t read_temperature(float &temp_c);
    esp_err_t read_current(float &current_a);
    esp_err_t read_power(float &power_w);
    esp_err_t read_manufacturer_id(uint16_t &id);
    esp_err_t read_device_id(uint16_t &id);

    bool is_initialized() const { return _initialized; }

private:
    esp_err_t _read_reg16(Register reg, uint16_t &value);
    esp_err_t _write_reg16(Register reg, uint16_t value);
    esp_err_t _read_reg24(Register reg, uint32_t &value);

    I2CBase &_i2c;
    uint8_t _addr;
    float _current_lsb;
    float _power_lsb;
    bool _range_40mv;
    bool _initialized;
};

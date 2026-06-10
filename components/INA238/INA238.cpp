#include "INA238.hpp"
#include "esp_log.h"

static const char *TAG = "INA238";

INA238::INA238(I2CBase &i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
    , _current_lsb(0.0f)
    , _power_lsb(0.0f)
    , _range_40mv(false)
    , _initialized(false) {}

esp_err_t INA238::_read_reg16(Register reg, uint16_t &value) {
    uint8_t buf[2];
    esp_err_t ret = _i2c.read_reg(_addr, (uint8_t)reg, buf, 2);
    if (ret == ESP_OK) {
        value = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

esp_err_t INA238::_write_reg16(Register reg, uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return _i2c.write_reg(_addr, (uint8_t)reg, buf, 2);
}

esp_err_t INA238::_read_reg24(Register reg, uint32_t &value) {
    uint8_t buf[3];
    esp_err_t ret = _i2c.read_reg(_addr, (uint8_t)reg, buf, 3);
    if (ret == ESP_OK) {
        value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }
    return ret;
}

esp_err_t INA238::init() {
    uint16_t device_id = 0;
    esp_err_t ret = read_device_id(device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Device ID: 0x%04X", device_id);

    ret = reset();
    if (ret != ESP_OK) return ret;

    _initialized = true;
    return ESP_OK;
}

esp_err_t INA238::reset() {
    return _write_reg16(REG_CONFIG, CONFIG_RST);
}

esp_err_t INA238::set_adc_range(bool range_40mv) {
    uint16_t config;
    esp_err_t ret = _read_reg16(REG_CONFIG, config);
    if (ret != ESP_OK) return ret;

    _range_40mv = range_40mv;
    if (_range_40mv) {
        config |= CONFIG_ADCRANGE;
    } else {
        config &= ~CONFIG_ADCRANGE;
    }
    return _write_reg16(REG_CONFIG, config);
}

esp_err_t INA238::set_adc_config(uint16_t config) {
    return _write_reg16(REG_ADC_CONFIG, config);
}

esp_err_t INA238::calibrate(float shunt_resistance, float max_current_a) {
    _current_lsb = max_current_a / 32768.0f;
    _power_lsb = 20.0f * _current_lsb;

    uint16_t config;
    esp_err_t ret = _read_reg16(REG_CONFIG, config);
    if (ret != ESP_OK) return ret;

    _range_40mv = config & CONFIG_ADCRANGE;
    float shunt_cal;
    if (_range_40mv) {
        shunt_cal = 524288000.0f * _current_lsb * shunt_resistance;
    } else {
        shunt_cal = 13107200000.0f * _current_lsb * shunt_resistance;
    }

    uint16_t shunt_cal_val = (uint16_t)(shunt_cal + 0.5f);
    if (shunt_cal_val > 0x7FFF) shunt_cal_val = 0x7FFF;

    ret = _write_reg16(REG_SHUNT_CAL, shunt_cal_val);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibrated: Rshunt=%.6f Ohm, Imax=%.3f A, LSB=%.8f A/bit, SHUNT_CAL=0x%04X",
                 shunt_resistance, max_current_a, _current_lsb, shunt_cal_val);
    }
    return ret;
}

esp_err_t INA238::read_shunt_voltage(float &voltage_mv) {
    uint16_t raw;
    esp_err_t ret = _read_reg16(REG_VSHUNT, raw);
    if (ret != ESP_OK) return ret;

    int16_t signed_raw = (int16_t)raw;
    if (_range_40mv) {
        voltage_mv = (float)signed_raw * 1.25e-3f;
    } else {
        voltage_mv = (float)signed_raw * 5.0e-3f;
    }
    return ESP_OK;
}

esp_err_t INA238::read_bus_voltage(float &voltage_v) {
    uint16_t raw;
    esp_err_t ret = _read_reg16(REG_VBUS, raw);
    if (ret != ESP_OK) return ret;

    voltage_v = (float)raw * 3.125e-3f;
    return ESP_OK;
}

esp_err_t INA238::read_temperature(float &temp_c) {
    uint16_t raw;
    esp_err_t ret = _read_reg16(REG_DIETEMP, raw);
    if (ret != ESP_OK) return ret;

    int16_t signed_raw = (int16_t)raw;
    temp_c = (float)signed_raw * 7.8125e-3f;
    return ESP_OK;
}

esp_err_t INA238::read_current(float &current_a) {
    if (_current_lsb == 0.0f) {
        ESP_LOGE(TAG, "Device not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t raw;
    esp_err_t ret = _read_reg16(REG_CURRENT, raw);
    if (ret != ESP_OK) return ret;

    int16_t signed_raw = (int16_t)raw;
    current_a = (float)signed_raw * _current_lsb;
    return ESP_OK;
}

esp_err_t INA238::read_power(float &power_w) {
    if (_current_lsb == 0.0f) {
        ESP_LOGE(TAG, "Device not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t raw;
    esp_err_t ret = _read_reg24(REG_POWER, raw);
    if (ret != ESP_OK) return ret;

    if (raw & 0x800000) raw |= 0xFF000000;
    int32_t signed_raw = (int32_t)raw;
    power_w = (float)signed_raw * _power_lsb;
    return ESP_OK;
}

esp_err_t INA238::read_manufacturer_id(uint16_t &id) {
    return _read_reg16(REG_MANUF_ID, id);
}

esp_err_t INA238::read_device_id(uint16_t &id) {
    return _read_reg16(REG_DEVICE_ID, id);
}

#include "INA238.hpp"
#include "esp_log.h"

static const char *TAG = "INA238";

INA238::INA238(I2CBase &i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
    , _current_lsb(0.0f)
    , _power_lsb(0.0f)
    , _shunt_resistance(0.0f)
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
    uint16_t manuf_id = 0;
    esp_err_t ret = read_manufacturer_id(manuf_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read manufacturer ID: %s", esp_err_to_name(ret));
        return ret;
    }
    if (manuf_id != 0x5449) {   // 0x5449 = "TI" in ASCII
        ESP_LOGE(TAG, "Unexpected manufacturer ID 0x%04X (expected 0x5449)",
                 (unsigned)manuf_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t device_id = 0;
    ret = read_device_id(device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((device_id >> 4) != 0x238) {   // bits 15:4 = die ID, bits 3:0 = revision
        ESP_LOGE(TAG, "Unexpected device ID 0x%04X (expected die 0x238)",
                 (unsigned)device_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "INA238 found: die 0x%03X, revision %u",
             (unsigned)(device_id >> 4), (unsigned)(device_id & 0x0F));

    ret = reset();
    if (ret != ESP_OK) return ret;

    _initialized = true;
    return ESP_OK;
}

esp_err_t INA238::reset() {
    esp_err_t ret = _write_reg16(REG_CONFIG, CONFIG_RST);
    if (ret == ESP_OK) {
        // The chip is back to defaults: invalidate the cached calibration so
        // read_current()/read_power() fail fast instead of silently returning
        // values scaled with a SHUNT_CAL that no longer matches the chip.
        _current_lsb = 0.0f;
        _power_lsb = 0.0f;
        _shunt_resistance = 0.0f;
        _range_40mv = false;
    }
    return ret;
}

esp_err_t INA238::set_adc_range(bool range_40mv) {
    uint16_t config;
    esp_err_t ret = _read_reg16(REG_CONFIG, config);
    if (ret != ESP_OK) return ret;

    uint16_t new_config = config;
    if (range_40mv) {
        new_config |= CONFIG_ADCRANGE;
    } else {
        new_config &= ~CONFIG_ADCRANGE;
    }

    if (new_config != config) {
        ret = _write_reg16(REG_CONFIG, new_config);
        if (ret != ESP_OK) return ret;
    }
    _range_40mv = range_40mv;   // only after the chip accepted the change

    // SHUNT_CAL scaling depends on ADCRANGE (x4 for ADCRANGE = 1),
    // so refresh the calibration when one has already been programmed.
    if (_current_lsb > 0.0f) {
        ret = _program_shunt_cal(_current_lsb, _shunt_resistance, range_40mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADCRANGE changed but SHUNT_CAL could not be updated - call calibrate() again");
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t INA238::set_adc_config(uint16_t config) {
    return _write_reg16(REG_ADC_CONFIG, config);
}

esp_err_t INA238::calibrate(float shunt_resistance, float max_current_a) {
    if (shunt_resistance <= 0.0f || max_current_a <= 0.0f) {
        ESP_LOGE(TAG, "Invalid calibration parameters: Rshunt=%.6f Ohm, Imax=%.3f A",
                 shunt_resistance, max_current_a);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t config;
    esp_err_t ret = _read_reg16(REG_CONFIG, config);
    if (ret != ESP_OK) return ret;

    bool range_40mv = (config & CONFIG_ADCRANGE) != 0;
    float current_lsb = max_current_a / 32768.0f;   // datasheet eq. 2: Imax / 2^15

    ret = _program_shunt_cal(current_lsb, shunt_resistance, range_40mv);
    if (ret != ESP_OK) return ret;

    // Commit cached state only after the chip accepted the calibration.
    _range_40mv = range_40mv;
    _shunt_resistance = shunt_resistance;
    _current_lsb = current_lsb;
    _power_lsb = 0.2f * current_lsb;   // datasheet eq. 4: Power LSB = 0.2 * CURRENT_LSB
    return ESP_OK;
}

// Datasheet 8.1.2, equation 1:
//   SHUNT_CAL = 819.2e6 * CURRENT_LSB * Rshunt        (ADCRANGE = 0)
//   SHUNT_CAL = 819.2e6 * 4 * CURRENT_LSB * Rshunt    (ADCRANGE = 1)
esp_err_t INA238::_program_shunt_cal(float current_lsb, float shunt_resistance, bool range_40mv) {
    float shunt_cal = 819.2e6f * current_lsb * shunt_resistance;
    if (range_40mv) shunt_cal *= 4.0f;

    // SHUNT_CAL is 15 bits (bit 15 is reserved). Never silently clamp: a
    // clamped value breaks the CURRENT_LSB scaling and skews every reading.
    if (shunt_cal < 0.5f || shunt_cal > 32767.0f) {
        ESP_LOGE(TAG, "SHUNT_CAL %.1f out of range: Rshunt=%.6f Ohm, LSB=%.8f A/bit, ADCRANGE=%d",
                 shunt_cal, shunt_resistance, current_lsb, range_40mv ? 1 : 0);
        return ESP_ERR_INVALID_SIZE;
    }

    uint16_t cal_val = (uint16_t)(shunt_cal + 0.5f);
    esp_err_t ret = _write_reg16(REG_SHUNT_CAL, cal_val);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibrated: Rshunt=%.6f Ohm, LSB=%.8f A/bit, SHUNT_CAL=0x%04X, ADCRANGE=%d",
                 shunt_resistance, current_lsb, (unsigned)cal_val, range_40mv ? 1 : 0);
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

    // DIETEMP occupies bits 15:4 (bits 3:0 are reserved and always read 0),
    // two's complement, 125 m*C per LSB (datasheet 7.6.1.6).
    int16_t signed_raw = (int16_t)raw;
    temp_c = (float)(signed_raw >> 4) * 0.125f;
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

    // POWER is a 24-bit UNSIGNED register (datasheet 7.6.1.8) - no sign extension.
    power_w = (float)raw * _power_lsb;
    return ESP_OK;
}

esp_err_t INA238::read_manufacturer_id(uint16_t &id) {
    return _read_reg16(REG_MANUF_ID, id);
}

esp_err_t INA238::read_device_id(uint16_t &id) {
    return _read_reg16(REG_DEVICE_ID, id);
}

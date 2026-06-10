#include "LM75A.hpp"
#include "esp_log.h"

LM75A::LM75A(I2CBase &i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
    , _initialized(false) {}

esp_err_t LM75A::init() {
    _initialized = true;
    return ESP_OK;
}

esp_err_t LM75A::read_temperature(float &temp_c) {
    uint8_t buf[2];
    esp_err_t ret = _i2c.read_reg(_addr, REG_TEMP, buf, 2);
    if (ret != ESP_OK) return ret;

    int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
    raw >>= 5;

    if (raw & 0x0400) {
        raw |= 0xF800;
    }

    temp_c = (float)raw * 0.125f;
    return ESP_OK;
}

esp_err_t LM75A::set_shutdown(bool shutdown) {
    uint8_t conf;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CONF, &conf, 1);
    if (ret != ESP_OK) return ret;

    if (shutdown) conf |= 0x01;
    else conf &= ~0x01;

    return _i2c.write_reg(_addr, REG_CONF, &conf, 1);
}

esp_err_t LM75A::set_os_mode(bool comparator_mode) {
    uint8_t conf;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CONF, &conf, 1);
    if (ret != ESP_OK) return ret;

    if (comparator_mode) conf &= ~0x02;
    else conf |= 0x02;

    return _i2c.write_reg(_addr, REG_CONF, &conf, 1);
}

esp_err_t LM75A::set_os_polarity(bool active_high) {
    uint8_t conf;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CONF, &conf, 1);
    if (ret != ESP_OK) return ret;

    if (active_high) conf |= 0x04;
    else conf &= ~0x04;

    return _i2c.write_reg(_addr, REG_CONF, &conf, 1);
}

esp_err_t LM75A::set_fault_queue(uint8_t faults) {
    uint8_t conf;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CONF, &conf, 1);
    if (ret != ESP_OK) return ret;

    conf &= ~0x18;
    switch (faults) {
        case 1:  conf |= 0x00; break;
        case 2:  conf |= 0x08; break;
        case 4:  conf |= 0x10; break;
        case 6:  conf |= 0x18; break;
        default: return ESP_ERR_INVALID_ARG;
    }

    return _i2c.write_reg(_addr, REG_CONF, &conf, 1);
}

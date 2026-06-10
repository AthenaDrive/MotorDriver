#include "AS5047P.hpp"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "AS5047P";

AS5047P::AS5047P(SPIBase &spi, uint8_t cs_gpio, int clock_speed_hz)
    : _spi(spi)
    , _dev(nullptr)
    , _cs_gpio(cs_gpio)
    , _clock_speed_hz(clock_speed_hz)
    , _initialized(false)
    , _pipeline_active(false) {}

uint16_t AS5047P::_apply_parity(uint16_t word) {
    uint16_t w = word & 0x7FFF;
    w ^= w >> 8;
    w ^= w >> 4;
    w ^= w >> 2;
    w ^= w >> 1;
    if (w & 1) {
        word |= ENCODER_PAR_BIT;
    } else {
        word &= ~ENCODER_PAR_BIT;
    }
    return word;
}

float AS5047P::_raw_to_degrees(uint16_t raw) {
    return (float)(raw & ENCODER_DATA_MASK) * 360.0f / 16384.0f;
}

esp_err_t AS5047P::init() {
    if (_initialized) {
        return ESP_OK;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode = 1;
    dev_cfg.clock_speed_hz = _clock_speed_hz;
    dev_cfg.spics_io_num = _cs_gpio;
    dev_cfg.queue_size = 1;
    dev_cfg.cs_ena_pretrans = 2;
    dev_cfg.cs_ena_posttrans = 2;

    esp_err_t ret = _spi.add_device(&dev_cfg, &_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    _pipeline_active = false;
    ESP_LOGI(TAG, "AS5047P initialized (CS:%d)", _cs_gpio);
    return ESP_OK;
}

esp_err_t AS5047P::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_dev) {
        SPIBase::remove_device(_dev);
        _dev = nullptr;
    }
    _initialized = false;
    _pipeline_active = false;
    return ESP_OK;
}

esp_err_t AS5047P::_transfer(uint16_t tx, uint16_t &rx) {
    if (!_initialized || !_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t tx_buf[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
    uint8_t rx_buf[2] = {0};

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.rxlength = 16;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t ret = _spi.polling_transmit(_dev, &trans);
    if (ret == ESP_OK) {
        rx = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    }
    return ret;
}

esp_err_t AS5047P::read_register(uint16_t address, uint16_t &data) {
    uint16_t cmd = _apply_parity(ENCODER_RW_READ | (address & ENCODER_ADDR_MASK));
    uint16_t rx;
    esp_err_t ret = _transfer(cmd, rx);
    if (ret != ESP_OK) return ret;

    uint16_t nop = _apply_parity(NOP);
    ret = _transfer(nop, rx);
    if (ret != ESP_OK) return ret;

    data = rx & ENCODER_DATA_MASK;
    return ESP_OK;
}

esp_err_t AS5047P::write_register(uint16_t address, uint16_t data) {
    uint16_t cmd = _apply_parity(ENCODER_RW_WRITE | (address & ENCODER_ADDR_MASK));
    uint16_t rx;
    esp_err_t ret = _transfer(cmd, rx);
    if (ret != ESP_OK) return ret;

    uint16_t data_frame = _apply_parity(data & ENCODER_DATA_MASK);
    ret = _transfer(data_frame, rx);
    return ret;
}

esp_err_t AS5047P::read_angle_raw(uint16_t &raw, bool with_daec) {
    return read_register(with_daec ? ANGLECOM : ANGLEUNC, raw);
}

esp_err_t AS5047P::read_angle(float &degrees, bool with_daec) {
    uint16_t raw;
    esp_err_t ret = read_angle_raw(raw, with_daec);
    if (ret != ESP_OK) return ret;
    degrees = _raw_to_degrees(raw);
    return ESP_OK;
}

esp_err_t AS5047P::pipeline_start() {
    uint16_t cmd = _apply_parity(ENCODER_RW_READ | (ANGLECOM & ENCODER_ADDR_MASK));
    uint16_t rx;
    esp_err_t ret = _transfer(cmd, rx);
    if (ret == ESP_OK) {
        _pipeline_active = true;
    }
    return ret;
}

esp_err_t AS5047P::pipeline_read_angle(float &degrees, bool with_daec) {
    if (!_pipeline_active) {
        esp_err_t ret = pipeline_start();
        if (ret != ESP_OK) return ret;
        degrees = 0.0f;
        return ESP_OK;
    }
    uint16_t reg = with_daec ? ANGLECOM : ANGLEUNC;
    uint16_t cmd = _apply_parity(ENCODER_RW_READ | (reg & ENCODER_ADDR_MASK));
    uint16_t rx;
    esp_err_t ret = _transfer(cmd, rx);
    if (ret != ESP_OK) return ret;
    degrees = _raw_to_degrees(rx);
    return ESP_OK;
}

#include "AS5047P.hpp"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>

static const char *TAG = "AS5047P";

AS5047P::AS5047P(SPIBase &spi, uint8_t cs_gpio, int clock_speed_hz)
    : _spi(spi)
    , _dev(nullptr)
    , _cs_gpio(cs_gpio)
    , _clock_speed_hz(clock_speed_hz)
    , _initialized(false)
    , _pipeline_active(false)
    , _transfer_pending(false)
    , _first_pipeline_frame(false)
    , _pending_trans{} {}

IRAM_ATTR uint16_t AS5047P::_apply_parity(uint16_t word) {
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

IRAM_ATTR float AS5047P::_raw_to_radians(uint16_t raw)
{
    // Keep this single precision. M_PI is a double, and the resulting
    // __extendsfdf2/__muldf3/__truncdfsf2 calls made the encoder read one of
    // the most expensive stages in the FOC loop.
    constexpr float RAD_PER_COUNT = (2.0f * 3.14159265358979323846f) / 16384.0f;
    return (float)(raw & ENCODER_DATA_MASK) * RAD_PER_COUNT;
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
    _transfer_pending = false;
    _first_pipeline_frame = false;

    // Log the actual clock the driver picked. If the encoder stage is slow,
    // this separates wire time from IDF transaction overhead.
    int actual_freq_khz = 0;
    if (spi_device_get_actual_freq(_dev, &actual_freq_khz) == ESP_OK) {
        ESP_LOGI(TAG, "AS5047P initialized (CS:%d, SPI %d kHz)", _cs_gpio, actual_freq_khz);
    } else {
        ESP_LOGI(TAG, "AS5047P initialized (CS:%d)", _cs_gpio);
    }
    return ESP_OK;
}

esp_err_t AS5047P::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    // A split transaction must be finished before the device is removed,
    // otherwise the SPI bus lock is left held.
    if (_transfer_pending && _dev) {
        _spi.polling_end(_dev);
        _transfer_pending = false;
    }
    if (_dev) {
        SPIBase::remove_device(_dev);
        _dev = nullptr;
    }
    _initialized = false;
    _pipeline_active = false;
    return ESP_OK;
}

IRAM_ATTR esp_err_t AS5047P::_transfer(uint16_t tx, uint16_t &rx) {
    esp_err_t ret = _transfer_start(tx);
    if (ret != ESP_OK) {
        return ret;
    }
    return _transfer_finish(rx);
}

IRAM_ATTR esp_err_t AS5047P::_transfer_start(uint16_t tx) {
    if (!_initialized || !_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_transfer_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    // Keep the buffers alive until polling_end(); the transaction stores
    // pointers to them, so they cannot be local to this function.
    _tx_buf[0] = static_cast<uint8_t>(tx >> 8);
    _tx_buf[1] = static_cast<uint8_t>(tx & 0xFF);
    _rx_buf[0] = 0;
    _rx_buf[1] = 0;

    _pending_trans = {};
    _pending_trans.length = 16;
    _pending_trans.rxlength = 16;
    _pending_trans.tx_buffer = _tx_buf;
    _pending_trans.rx_buffer = _rx_buf;

    esp_err_t ret = _spi.polling_start(_dev, &_pending_trans);
    if (ret == ESP_OK) {
        _transfer_pending = true;
    }
    return ret;
}

IRAM_ATTR esp_err_t AS5047P::_transfer_finish(uint16_t &rx) {
    if (!_transfer_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = _spi.polling_end(_dev);
    _transfer_pending = false;
    if (ret == ESP_OK) {
        rx = (static_cast<uint16_t>(_rx_buf[0]) << 8) | _rx_buf[1];
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
        _first_pipeline_frame = false;
    }
    return ret;
}

IRAM_ATTR esp_err_t AS5047P::pipeline_start_read(bool with_daec) {
    uint16_t reg = with_daec ? ANGLECOM : ANGLEUNC;
    uint16_t cmd = _apply_parity(ENCODER_RW_READ | (reg & ENCODER_ADDR_MASK));

    esp_err_t ret = _transfer_start(cmd);
    if (ret == ESP_OK) {
        if (!_pipeline_active) {
            // The response to the first command is not a valid angle yet.
            _pipeline_active = true;
            _first_pipeline_frame = true;
        } else {
            _first_pipeline_frame = false;
        }
    }
    return ret;
}

IRAM_ATTR esp_err_t AS5047P::pipeline_finish_read(float &radians) {
    uint16_t rx;
    esp_err_t ret = _transfer_finish(rx);
    if (ret != ESP_OK) {
        return ret;
    }

    if (_first_pipeline_frame) {
        _first_pipeline_frame = false;
        radians = 0.0f;
        return ESP_OK;
    }

    radians = _raw_to_radians(rx);
    return ESP_OK;
}

IRAM_ATTR esp_err_t AS5047P::pipeline_read_angle(float &radians, bool with_daec) {
    esp_err_t ret = pipeline_start_read(with_daec);
    if (ret != ESP_OK) {
        return ret;
    }
    return pipeline_finish_read(radians);
}

#include "DRV8323.hpp"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "DRV8323";

DRV8323::DRV8323(SPIBase &spi, uint8_t cs_gpio, uint8_t spi_mode, int clock_speed_hz)
    : _spi(spi)
    , _dev(nullptr)
    , _cs_gpio(cs_gpio)
    , _spi_mode(spi_mode)
    , _clock_speed_hz(clock_speed_hz)
    , _initialized(false) {}

esp_err_t DRV8323::init() {
    if (_initialized) {
        return ESP_OK;
    }

    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << _cs_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_conf));
    gpio_set_level((gpio_num_t)_cs_gpio, 1);

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode = _spi_mode;
    dev_cfg.clock_speed_hz = _clock_speed_hz;
    dev_cfg.spics_io_num = _cs_gpio;
    dev_cfg.queue_size = 1;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;

    esp_err_t ret = _spi.add_device(&dev_cfg, &_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG, "DRV8323 initialized (CS:%d)", _cs_gpio);
    return ESP_OK;
}

esp_err_t DRV8323::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_dev) {
        SPIBase::remove_device(_dev);
        _dev = nullptr;
    }
    _initialized = false;
    return ESP_OK;
}

esp_err_t DRV8323::_spi_transfer(uint16_t tx, uint16_t &rx) {
    if (!_initialized || !_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t tx_buf[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
    uint8_t rx_buf[2] = {0, 0};

    spi_transaction_t trans{};
    trans.length = 16;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;
    
    esp_err_t ret = _spi.transmit(_dev, &trans);
    rx = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return ret;
}

esp_err_t DRV8323::write_register(uint8_t reg, uint16_t value) {
    uint16_t cmd = (reg << 11) | (value & 0x07FF);
    uint16_t rx;
    esp_err_t ret = _spi_transfer(cmd, rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t DRV8323::read_register(uint8_t reg, uint16_t &value) {
    uint16_t cmd = (1 << 15) | (reg << 11);
    uint16_t rx;
    esp_err_t ret = _spi_transfer(cmd, rx);
    if (ret != ESP_OK) {
        return ret;
    }
    value = rx & 0x07FF;
    return ESP_OK;
}

esp_err_t DRV8323::set_pwm_mode(uint16_t mode) {
    uint16_t val = 0;
    esp_err_t ret = read_register(REG_DRIVER_CONTROL, val);
    if (ret != ESP_OK) return ret;
    val = (val & ~0x0700) | mode;
    return write_register(REG_DRIVER_CONTROL, val);
}

esp_err_t DRV8323::set_3x_pwm_mode() {
    return set_pwm_mode(PWM_MODE_3X);
}

esp_err_t DRV8323::read_fault_status(uint16_t &status) {
    return read_register(REG_FAULT_STATUS_1, status);
}

esp_err_t DRV8323::read_vgs_status(uint16_t &status) {
    return read_register(REG_VGS_STATUS_2, status);
}

bool DRV8323::has_fault(uint16_t status, FaultBits bit) const {
    return (status & (uint16_t)bit) != 0;
}

#include <cstdlib>
#include <cstring>
#include "I2CBase.hpp"
#include "esp_log.h"

static const char *TAG = "I2CBase";

I2CBase::BusEntry I2CBase::_buses[I2C_BASE_MAX_BUSES] = {};

I2CBase::I2CBase(gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz)
    : _bus_entry(nullptr)
    , _sda(sda)
    , _scl(scl)
    , _freq_hz(freq_hz)
    , _initialized(false) {}

I2CBase::~I2CBase() {
    if (_initialized) {
        deinit();
    }
}

int I2CBase::_find_bus(gpio_num_t sda, gpio_num_t scl) {
    for (int i = 0; i < I2C_BASE_MAX_BUSES; i++) {
        if (_buses[i].used && _buses[i].sda == sda && _buses[i].scl == scl) {
            return i;
        }
    }
    return -1;
}

int I2CBase::_find_free_slot() {
    for (int i = 0; i < I2C_BASE_MAX_BUSES; i++) {
        if (!_buses[i].used) {
            return i;
        }
    }
    return -1;
}

bool I2CBase::is_initialized() const {
    return _initialized;
}

esp_err_t I2CBase::init() {
    if (_initialized) {
        return ESP_OK;
    }

    int idx = _find_bus(_sda, _scl);
    if (idx >= 0) {
        _bus_entry = &_buses[idx];
        _bus_entry->ref_count++;
        _initialized = true;
        ESP_LOGD(TAG, "Reusing I2C bus (SDA:%d, SCL:%d)", _sda, _scl);
        return ESP_OK;
    }

    int slot = _find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No free I2C bus slots (max %d)", I2C_BASE_MAX_BUSES);
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = _sda,
        .scl_io_num = _scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &_buses[slot].handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _buses[slot].sda = _sda;
    _buses[slot].scl = _scl;
    _buses[slot].ref_count = 1;
    _buses[slot].used = true;
    _bus_entry = &_buses[slot];
    _initialized = true;

    ESP_LOGI(TAG, "I2C bus created (SDA:%d, SCL:%d, %lu Hz)",
             _sda, _scl, (unsigned long)_freq_hz);
    return ESP_OK;
}

esp_err_t I2CBase::deinit() {
    if (!_initialized || !_bus_entry) {
        return ESP_ERR_INVALID_STATE;
    }

    _bus_entry->ref_count--;
    if (_bus_entry->ref_count <= 0) {
        for (int i = 0; i < I2C_BASE_DEV_CACHE_SIZE; i++) {
            if (_bus_entry->dev_cache[i].handle) {
                i2c_master_bus_rm_device(_bus_entry->dev_cache[i].handle);
                _bus_entry->dev_cache[i].handle = nullptr;
            }
        }
        esp_err_t ret = i2c_del_master_bus(_bus_entry->handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_del_master_bus failed: %s", esp_err_to_name(ret));
            return ret;
        }
        _bus_entry->used = false;
        _bus_entry->handle = NULL;
        ESP_LOGI(TAG, "I2C bus deleted (SDA:%d, SCL:%d)", _sda, _scl);
    }

    _bus_entry = nullptr;
    _initialized = false;
    return ESP_OK;
}

esp_err_t I2CBase::probe(uint8_t dev_addr, int timeout_ms) {
    if (!_initialized || !_bus_entry) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_probe(_bus_entry->handle, dev_addr, timeout_ms);
}

esp_err_t I2CBase::_get_cached_dev(uint8_t dev_addr, i2c_master_dev_handle_t &dev_handle) {
    for (int i = 0; i < I2C_BASE_DEV_CACHE_SIZE; i++) {
        if (_bus_entry->dev_cache[i].handle && _bus_entry->dev_cache[i].addr == dev_addr) {
            dev_handle = _bus_entry->dev_cache[i].handle;
            return ESP_OK;
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = _freq_hz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    esp_err_t ret = i2c_master_bus_add_device(_bus_entry->handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) return ret;

    for (int i = 0; i < I2C_BASE_DEV_CACHE_SIZE; i++) {
        if (!_bus_entry->dev_cache[i].handle) {
            _bus_entry->dev_cache[i].addr = dev_addr;
            _bus_entry->dev_cache[i].handle = dev_handle;
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Device cache full, not caching 0x%02X", dev_addr);
    return ESP_OK;
}

esp_err_t I2CBase::_transmit(uint8_t dev_addr, const uint8_t *write_buf,
                             size_t write_len, uint8_t *read_buf,
                             size_t read_len, int timeout_ms) {
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = _get_cached_dev(dev_addr, dev_handle);
    if (ret != ESP_OK) return ret;

    if (write_buf && read_buf) {
        ret = i2c_master_transmit_receive(dev_handle, write_buf, write_len,
                                          read_buf, read_len, timeout_ms);
    } else if (write_buf) {
        ret = i2c_master_transmit(dev_handle, write_buf, write_len, timeout_ms);
    } else if (read_buf) {
        ret = i2c_master_receive(dev_handle, read_buf, read_len, timeout_ms);
    } else {
        ret = ESP_ERR_INVALID_ARG;
    }
    return ret;
}

esp_err_t I2CBase::read(uint8_t dev_addr, uint8_t *data, size_t len, int timeout_ms) {
    return _transmit(dev_addr, NULL, 0, data, len, timeout_ms);
}

esp_err_t I2CBase::write(uint8_t dev_addr, const uint8_t *data, size_t len, int timeout_ms) {
    return _transmit(dev_addr, data, len, NULL, 0, timeout_ms);
}

esp_err_t I2CBase::read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data,
                            size_t len, int timeout_ms) {
    return _transmit(dev_addr, &reg, 1, data, len, timeout_ms);
}

esp_err_t I2CBase::write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data,
                             size_t len, int timeout_ms) {
    uint8_t *buf = (uint8_t *)malloc(1 + len);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    esp_err_t ret = _transmit(dev_addr, buf, 1 + len, NULL, 0, timeout_ms);
    free(buf);
    return ret;
}

esp_err_t I2CBase::add_device(uint8_t dev_addr, i2c_master_dev_handle_t *out_handle) {
    if (!_initialized || !_bus_entry) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = _freq_hz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    return i2c_master_bus_add_device(_bus_entry->handle, &dev_cfg, out_handle);
}

esp_err_t I2CBase::remove_device(i2c_master_dev_handle_t handle) {
    return i2c_master_bus_rm_device(handle);
}

i2c_master_bus_handle_t I2CBase::bus_handle() const {
    return _bus_entry ? _bus_entry->handle : NULL;
}

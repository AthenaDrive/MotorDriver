#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_BASE_MAX_BUSES      4
#define I2C_BASE_DEFAULT_FREQ   400000
#define I2C_BASE_DEV_CACHE_SIZE 4

class I2CBase {
public:
    I2CBase(gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz = I2C_BASE_DEFAULT_FREQ);
    ~I2CBase();

    I2CBase(const I2CBase &) = delete;
    I2CBase &operator=(const I2CBase &) = delete;

    esp_err_t init();
    esp_err_t deinit();
    bool is_initialized() const;

    esp_err_t probe(uint8_t dev_addr, int timeout_ms = 100);

    esp_err_t read(uint8_t dev_addr, uint8_t *data, size_t len, int timeout_ms = -1);
    esp_err_t write(uint8_t dev_addr, const uint8_t *data, size_t len, int timeout_ms = -1);
    esp_err_t read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len, int timeout_ms = -1);
    esp_err_t write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len, int timeout_ms = -1);

    esp_err_t add_device(uint8_t dev_addr, i2c_master_dev_handle_t *out_handle);
    static esp_err_t remove_device(i2c_master_dev_handle_t handle);

    i2c_master_bus_handle_t bus_handle() const;

private:
    struct CachedDev {
        uint8_t addr;
        i2c_master_dev_handle_t handle;
    };

    struct BusEntry {
        gpio_num_t sda;
        gpio_num_t scl;
        i2c_master_bus_handle_t handle;
        int ref_count;
        bool used;
        CachedDev dev_cache[I2C_BASE_DEV_CACHE_SIZE];
    };

    static BusEntry _buses[I2C_BASE_MAX_BUSES];
    static int _find_bus(gpio_num_t sda, gpio_num_t scl);
    static int _find_free_slot();

    esp_err_t _get_cached_dev(uint8_t dev_addr, i2c_master_dev_handle_t &dev_handle);
    esp_err_t _transmit(uint8_t dev_addr, const uint8_t *write_buf, size_t write_len,
                        uint8_t *read_buf, size_t read_len, int timeout_ms);

    BusEntry *_bus_entry;
    gpio_num_t _sda;
    gpio_num_t _scl;
    uint32_t _freq_hz;
    bool _initialized;
};

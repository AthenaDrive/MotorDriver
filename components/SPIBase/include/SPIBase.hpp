#pragma once

#include <cstdint>
#include <cstddef>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#define SPI_BASE_MAX_BUSES    4

class SPIBase {
public:
    SPIBase(spi_host_device_t host, int clk, int pico, int poci,
            int dma_chan = SPI_DMA_CH_AUTO, int max_speed_hz = 1000000);
    ~SPIBase();

    SPIBase(const SPIBase &) = delete;
    SPIBase &operator=(const SPIBase &) = delete;

    esp_err_t init();
    esp_err_t deinit();
    bool is_initialized() const;

    esp_err_t add_device(const spi_device_interface_config_t *dev_config,
                         spi_device_handle_t *out_handle);
    static esp_err_t remove_device(spi_device_handle_t handle);

    esp_err_t transmit(spi_device_handle_t dev, spi_transaction_t *trans);
    esp_err_t polling_transmit(spi_device_handle_t dev, spi_transaction_t *trans);

    esp_err_t write_read(spi_device_handle_t dev,
                         const uint8_t *write_buf, size_t write_len,
                         uint8_t *read_buf, size_t read_len);

    spi_host_device_t host() const;

private:
    struct BusEntry {
        spi_host_device_t host;
        int clk;
        int pico;
        int poci;
        int ref_count;
        bool used;
    };

    static BusEntry _buses[SPI_BASE_MAX_BUSES];
    static int _find_bus(spi_host_device_t host, int clk, int pico, int poci);
    static int _find_free_slot();

    BusEntry *_bus_entry;
    spi_host_device_t _host;
    int _clk, _pico, _poci;
    int _dma_chan;
    int _max_speed_hz;
    bool _initialized;
};

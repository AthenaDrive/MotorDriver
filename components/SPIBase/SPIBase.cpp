#include <cstdlib>
#include <cstring>
#include "SPIBase.hpp"
#include "esp_log.h"

static const char *TAG = "SPIBase";

SPIBase::BusEntry SPIBase::_buses[SPI_BASE_MAX_BUSES] = {};

SPIBase::SPIBase(spi_host_device_t host, int clk, int pico, int poci,
                 int dma_chan, int max_speed_hz)
    : _bus_entry(nullptr)
    , _host(host)
    , _clk(clk)
    , _pico(pico)
    , _poci(poci)
    , _dma_chan(dma_chan)
    , _max_speed_hz(max_speed_hz)
    , _initialized(false) {}

SPIBase::~SPIBase() {
    if (_initialized) {
        deinit();
    }
}

int SPIBase::_find_bus(spi_host_device_t host, int clk, int pico, int poci) {
    for (int i = 0; i < SPI_BASE_MAX_BUSES; i++) {
        if (_buses[i].used &&
            _buses[i].host == host &&
            _buses[i].clk == clk &&
            _buses[i].pico == pico &&
            _buses[i].poci == poci) {
            return i;
        }
    }
    return -1;
}

int SPIBase::_find_free_slot() {
    for (int i = 0; i < SPI_BASE_MAX_BUSES; i++) {
        if (!_buses[i].used) {
            return i;
        }
    }
    return -1;
}

bool SPIBase::is_initialized() const {
    return _initialized;
}

esp_err_t SPIBase::init() {
    if (_initialized) {
        return ESP_OK;
    }

    int idx = _find_bus(_host, _clk, _pico, _poci);
    if (idx >= 0) {
        _bus_entry = &_buses[idx];
        _bus_entry->ref_count++;
        _initialized = true;
        ESP_LOGD(TAG, "Reusing SPI bus (host:%d, CLK:%d, PICO:%d, POCI:%d)",
                 _host, _clk, _pico, _poci);
        return ESP_OK;
    }

    int slot = _find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No free SPI bus slots (max %d)", SPI_BASE_MAX_BUSES);
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = _pico;
    bus_config.miso_io_num = _poci;
    bus_config.sclk_io_num = _clk;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.data4_io_num = -1;
    bus_config.data5_io_num = -1;
    bus_config.data6_io_num = -1;
    bus_config.data7_io_num = -1;
    bus_config.max_transfer_sz = 4096;
    bus_config.flags = 0;
    bus_config.intr_flags = 0;

    esp_err_t ret = spi_bus_initialize(_host, &bus_config, (spi_dma_chan_t)_dma_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _buses[slot].host = _host;
    _buses[slot].clk = _clk;
    _buses[slot].pico = _pico;
    _buses[slot].poci = _poci;
    _buses[slot].ref_count = 1;
    _buses[slot].used = true;
    _bus_entry = &_buses[slot];
    _initialized = true;

    ESP_LOGI(TAG, "SPI bus created (host:%d, CLK:%d, PICO:%d, POCI:%d, %d Hz)",
             _host, _clk, _pico, _poci, _max_speed_hz);
    return ESP_OK;
}

esp_err_t SPIBase::deinit() {
    if (!_initialized || !_bus_entry) {
        return ESP_ERR_INVALID_STATE;
    }

    _bus_entry->ref_count--;
    if (_bus_entry->ref_count <= 0) {
        esp_err_t ret = spi_bus_free(_host);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_free failed: %s", esp_err_to_name(ret));
            return ret;
        }
        _bus_entry->used = false;
        ESP_LOGI(TAG, "SPI bus freed (host:%d)", _host);
    }

    _bus_entry = nullptr;
    _initialized = false;
    return ESP_OK;
}

esp_err_t SPIBase::add_device(const spi_device_interface_config_t *dev_config,
                              spi_device_handle_t *out_handle) {
    if (!_initialized || !_bus_entry) {
        return ESP_ERR_INVALID_STATE;
    }
    return spi_bus_add_device(_host, dev_config, out_handle);
}

esp_err_t SPIBase::remove_device(spi_device_handle_t handle) {
    return spi_bus_remove_device(handle);
}

esp_err_t SPIBase::transmit(spi_device_handle_t dev, spi_transaction_t *trans) {
    return spi_device_transmit(dev, trans);
}

esp_err_t SPIBase::polling_transmit(spi_device_handle_t dev, spi_transaction_t *trans) {
    return spi_device_polling_transmit(dev, trans);
}

esp_err_t SPIBase::write_read(spi_device_handle_t dev,
                              const uint8_t *write_buf, size_t write_len,
                              uint8_t *read_buf, size_t read_len) {
    spi_transaction_t trans = {};
    trans.length = write_len * 8;
    trans.rxlength = read_len * 8;
    trans.user = NULL;
    trans.tx_buffer = write_buf;
    trans.rx_buffer = read_buf;
    return spi_device_transmit(dev, &trans);
}

spi_host_device_t SPIBase::host() const {
    return _bus_entry ? _bus_entry->host : (spi_host_device_t)-1;
}

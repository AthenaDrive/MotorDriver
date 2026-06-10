#pragma once

#include <cstdint>
#include <cstring>
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_netif.h"

class W5500 {
public:
    W5500(uint8_t cs_gpio, int int_gpio, uint8_t eth_idx = 0);
    ~W5500();

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t set_static_ip(const char *ip, const char *netmask = "255.255.255.0",
                            const char *gw = "192.168.1.1");

    esp_eth_handle_t eth_handle() const { return _eth_handle; }
    esp_netif_t *netif() const { return _netif; }
    bool is_initialized() const { return _initialized; }

private:
    uint8_t _cs_gpio;
    int _int_gpio;
    uint8_t _eth_idx;
    bool _initialized;

    esp_eth_handle_t _eth_handle;
    esp_eth_mac_t *_mac;
    esp_eth_phy_t *_phy;
    esp_netif_t *_netif;
    spi_device_interface_config_t _devcfg;
    esp_eth_netif_glue_handle_t _glue;
};

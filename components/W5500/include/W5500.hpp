#pragma once

#include <cstdint>
#include <cstring>
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_netif.h"

class W5500 {
public:
    // dhcp_server: create the netif with DHCP-server support. Only the port
    // facing the PC needs it; it is started explicitly once the final IP is set.
    W5500(uint8_t cs_gpio, int int_gpio, uint8_t eth_idx = 0, bool dhcp_server = false);
    ~W5500();

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t set_static_ip(const char *ip, const char *netmask = "255.255.255.0",
                            const char *gw = "192.168.1.1");

    // Serve leases to the device on this port. Pool is derived from this
    // interface's IP; the router option is suppressed because there is no
    // gateway on a point-to-point link.
    esp_err_t start_dhcp_server();
    esp_err_t stop_dhcp_server();

    // Read W5500 PHYCFGR (0x002E) through the ESP-IDF PHY register API.
    esp_err_t read_phycfgr(uint8_t *value) const;
    void log_phy_status(const char *when) const;

    esp_eth_handle_t eth_handle() const { return _eth_handle; }
    esp_netif_t *netif() const { return _netif; }
    bool is_initialized() const { return _initialized; }

private:
    uint8_t _cs_gpio;
    int _int_gpio;
    uint8_t _eth_idx;
    bool _dhcp_server;
    bool _initialized;

    esp_eth_handle_t _eth_handle;
    esp_eth_mac_t *_mac;
    esp_eth_phy_t *_phy;
    esp_netif_t *_netif;
    spi_device_interface_config_t _devcfg;
    esp_eth_netif_glue_handle_t _glue;
};

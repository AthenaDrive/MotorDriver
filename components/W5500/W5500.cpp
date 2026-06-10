#include "W5500.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_eth_mac_w5500.h"
#include "esp_eth_phy_w5500.h"
#include "esp_netif_defaults.h"
#include "lwip/inet.h"

static const char *TAG = "W5500";

W5500::W5500(uint8_t cs_gpio, int int_gpio, uint8_t eth_idx)
    : _cs_gpio(cs_gpio)
    , _int_gpio(int_gpio)
    , _eth_idx(eth_idx)
    , _initialized(false)
    , _eth_handle(nullptr)
    , _mac(nullptr)
    , _phy(nullptr)
    , _netif(nullptr)
    , _devcfg{}
    , _glue(nullptr) {}

W5500::~W5500() {
    deinit();
}

static void cleanup(esp_eth_mac_t *mac, esp_eth_phy_t *phy,
                    esp_eth_handle_t eth, esp_eth_netif_glue_handle_t glue,
                    esp_netif_t *netif) {
    if (glue) { esp_eth_del_netif_glue(glue); }
    if (eth) { esp_eth_driver_uninstall(eth); }
    if (phy) { phy->del(phy); }
    if (mac) { mac->del(mac); }
    if (netif) { esp_netif_destroy(netif); }
}

static void derive_mac(uint8_t *out_mac, const uint8_t *base_mac, uint8_t idx) {
    memcpy(out_mac, base_mac, 6);
    out_mac[5] += idx;
    if (out_mac[5] < base_mac[5]) {  // wrapped, carry into byte 4
        out_mac[4]++;
    }
}

esp_err_t W5500::init() {
    if (_initialized) {
        return ESP_OK;
    }

    _glue = nullptr;

    esp_eth_config_t eth_cfg = {};
    eth_w5500_config_t w5500_cfg = {};

    _devcfg = {};
    _devcfg.mode = 0;
    _devcfg.clock_speed_hz = 30 * 1000 * 1000;
    _devcfg.spics_io_num = _cs_gpio;
    _devcfg.cs_ena_posttrans = 2;
    _devcfg.queue_size = 20;

    w5500_cfg = ETH_W5500_DEFAULT_CONFIG(SPI3_HOST, &_devcfg);
    w5500_cfg.int_gpio_num = _int_gpio;

    {
        eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
        _mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    }
    if (!_mac) {
        ESP_LOGE(TAG, "esp_eth_mac_new_w5500 failed");
        return ESP_FAIL;
    }

    {
        eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
        phy_cfg.phy_addr = 0;       // W5500 internal PHY address is always 0
        _phy = esp_eth_phy_new_w5500(&phy_cfg);
    }
    if (!_phy) {
        cleanup(_mac, nullptr, nullptr, nullptr, nullptr);
        _mac = nullptr;
        return ESP_FAIL;
    }

    eth_cfg = ETH_DEFAULT_CONFIG(_mac, _phy);
    esp_err_t ret = esp_eth_driver_install(&eth_cfg, &_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(ret));
        cleanup(_mac, _phy, nullptr, nullptr, nullptr);
        _mac = nullptr; _phy = nullptr;
        return ESP_FAIL;
    }

    uint8_t mac_addr[6] = {};
    if (_eth_idx == 0) {
        ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_ETH));
    } else {
        uint8_t base_mac[6];
        ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_ETH));
        derive_mac(mac_addr, base_mac, _eth_idx);
    }
    esp_eth_ioctl(_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "W5500[%d] MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             _eth_idx, mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    _glue = esp_eth_new_netif_glue(_eth_handle);
    if (!_glue) {
        ESP_LOGE(TAG, "esp_eth_new_netif_glue failed");
        cleanup(_mac, _phy, _eth_handle, nullptr, nullptr);
        _mac = nullptr; _phy = nullptr; _eth_handle = nullptr;
        return ESP_FAIL;
    }

    {
        // Use unique if_key/if_desc per instance to avoid "duplicate key" error
        esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
        char key_buf[12], desc_buf[12];
        snprintf(key_buf, sizeof(key_buf), "ETH_%d", _eth_idx);
        snprintf(desc_buf, sizeof(desc_buf), "eth%d", _eth_idx);
        base_cfg.if_key = key_buf;
        base_cfg.if_desc = desc_buf;

        esp_netif_config_t cfg = {
            .base = &base_cfg,
            .driver = NULL,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
        };
        _netif = esp_netif_new(&cfg);
    }
    if (!_netif) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        cleanup(_mac, _phy, _eth_handle, _glue, nullptr);
        _mac = nullptr; _phy = nullptr; _eth_handle = nullptr; _glue = nullptr;
        return ESP_FAIL;
    }

    ret = esp_netif_attach(_netif, _glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_attach failed: %s", esp_err_to_name(ret));
        cleanup(_mac, _phy, _eth_handle, _glue, _netif);
        _mac = nullptr; _phy = nullptr; _eth_handle = nullptr; _glue = nullptr; _netif = nullptr;
        return ESP_FAIL;
    }

    ret = esp_eth_start(_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(ret));
        cleanup(_mac, _phy, _eth_handle, _glue, _netif);
        _mac = nullptr; _phy = nullptr; _eth_handle = nullptr; _glue = nullptr; _netif = nullptr;
        return ESP_FAIL;
    }

    _initialized = true;
    ESP_LOGI(TAG, "W5500[%d] ready (CS:%d, INT:%d)", _eth_idx, _cs_gpio, _int_gpio);
    return ESP_OK;
}

esp_err_t W5500::deinit() {
    if (!_initialized) {
        return ESP_OK;
    }

    if (_eth_handle) {
        esp_eth_stop(_eth_handle);
    }

    if (_glue) {
        esp_eth_del_netif_glue(_glue);
        _glue = nullptr;
    }

    if (_eth_handle) {
        esp_eth_driver_uninstall(_eth_handle);
        _eth_handle = nullptr;
    }

    if (_phy) {
        _phy->del(_phy);
        _phy = nullptr;
    }

    if (_mac) {
        _mac->del(_mac);
        _mac = nullptr;
    }

    if (_netif) {
        esp_netif_destroy(_netif);
        _netif = nullptr;
    }

    _initialized = false;
    return ESP_OK;
}

esp_err_t W5500::set_static_ip(const char *ip, const char *netmask, const char *gw) {
    if (!_netif) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_dhcpc_stop(_netif);

    esp_netif_ip_info_t ip_info = {};
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(ip, &ip_info.ip));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(netmask, &ip_info.netmask));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(gw, &ip_info.gw));
    return esp_netif_set_ip_info(_netif, &ip_info);
}

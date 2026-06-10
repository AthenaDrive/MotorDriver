#pragma once

#include <cstdint>
#include "esp_err.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"

class SdCard {
public:
    SdCard(spi_host_device_t host, int cs_gpio, const char *mount_point = "/sdcard");
    esp_err_t init();
    esp_err_t deinit();
    bool is_mounted() const;
    sdmmc_card_t *card() const;

private:
    spi_host_device_t _host;
    int _cs_gpio;
    const char *_mount_point;
    sdmmc_card_t *_card;
    bool _mounted;
};

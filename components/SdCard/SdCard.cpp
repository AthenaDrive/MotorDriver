#include "SdCard.hpp"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"

static const char *TAG = "SdCard";

SdCard::SdCard(spi_host_device_t host, int cs_gpio, const char *mount_point)
    : _host(host)
    , _cs_gpio(cs_gpio)
    , _mount_point(mount_point)
    , _card(nullptr)
    , _mounted(false) {}

esp_err_t SdCard::init() {
    if (_mounted) {
        return ESP_OK;
    }

    esp_err_t ret = sdspi_host_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "sdspi_host_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_conf = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_conf.host_id = _host;
    slot_conf.gpio_cs = (gpio_num_t)_cs_gpio;
    slot_conf.gpio_cd = SDSPI_SLOT_NO_CD;
    slot_conf.gpio_wp = SDSPI_SLOT_NO_WP;
    slot_conf.gpio_int = GPIO_NUM_NC;

    sdspi_dev_handle_t sd_handle;
    ret = sdspi_host_init_device(&slot_conf, &sd_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sdspi_host_init_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = sd_handle;

    esp_vfs_fat_mount_config_t mount_conf = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 0,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    ret = esp_vfs_fat_sdspi_mount(_mount_point, &host, &slot_conf, &mount_conf, &_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_fat_sdspi_mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s", _mount_point);
    return ESP_OK;
}

esp_err_t SdCard::deinit() {
    if (!_mounted) {
        return ESP_OK;
    }
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(_mount_point, _card);
    sdspi_host_deinit();
    _mounted = false;
    return ret;
}

bool SdCard::is_mounted() const {
    return _mounted;
}

sdmmc_card_t *SdCard::card() const {
    return _card;
}

#include "LSM6DSO.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSO";

LSM6DSO::LSM6DSO(I2CBase &i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
    , _accel_scale(ACCEL_SENSITIVITY[ACCEL_FS_2G])
    , _gyro_scale(GYRO_SENSITIVITY[GYRO_FS_250])
    , _initialized(false) {}

esp_err_t LSM6DSO::init() {
    uint8_t id = 0;
    esp_err_t ret = who_am_i(id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    if (id != LSM6DSO_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)", id, LSM6DSO_WHO_AM_I_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);

    uint8_t ctrl3 = CTRL3_SW_RESET;
    ret = _i2c.write_reg(_addr, REG_CTRL3_C, &ctrl3, 1);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    ctrl3 = CTRL3_BDU | CTRL3_IF_INC;
    ret = _i2c.write_reg(_addr, REG_CTRL3_C, &ctrl3, 1);
    if (ret != ESP_OK) return ret;

    set_accel_odr(0x04);
    set_accel_fs(ACCEL_FS_2G);
    set_gyro_odr(0x04);
    set_gyro_fs(GYRO_FS_250);

    _initialized = true;
    ESP_LOGI(TAG, "Initialized");
    return ESP_OK;
}

esp_err_t LSM6DSO::who_am_i(uint8_t &id) {
    return _i2c.read_reg(_addr, REG_WHO_AM_I, &id, 1);
}

esp_err_t LSM6DSO::set_accel_odr(uint8_t odr) {
    uint8_t ctrl1;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CTRL1_XL, &ctrl1, 1);
    if (ret != ESP_OK) return ret;

    ctrl1 = (ctrl1 & 0x0F) | (odr << 4);
    return _i2c.write_reg(_addr, REG_CTRL1_XL, &ctrl1, 1);
}

esp_err_t LSM6DSO::set_accel_fs(AccelFs fs) {
    if (fs > ACCEL_FS_8G) return ESP_ERR_INVALID_ARG;

    uint8_t ctrl1;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CTRL1_XL, &ctrl1, 1);
    if (ret != ESP_OK) return ret;

    ctrl1 = (ctrl1 & 0xF3) | (fs << 2);
    _accel_scale = ACCEL_SENSITIVITY[fs];
    return _i2c.write_reg(_addr, REG_CTRL1_XL, &ctrl1, 1);
}

esp_err_t LSM6DSO::read_accel(float &x, float &y, float &z) {
    uint8_t buf[6];
    esp_err_t ret = _i2c.read_reg(_addr, REG_OUTX_L_A, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)(buf[1] << 8) | buf[0];
    int16_t raw_y = (int16_t)(buf[3] << 8) | buf[2];
    int16_t raw_z = (int16_t)(buf[5] << 8) | buf[4];

    x = (float)raw_x * _accel_scale;
    y = (float)raw_y * _accel_scale;
    z = (float)raw_z * _accel_scale;
    return ESP_OK;
}

esp_err_t LSM6DSO::set_gyro_odr(uint8_t odr) {
    uint8_t ctrl2;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CTRL2_G, &ctrl2, 1);
    if (ret != ESP_OK) return ret;

    ctrl2 = (ctrl2 & 0x0F) | (odr << 4);
    return _i2c.write_reg(_addr, REG_CTRL2_G, &ctrl2, 1);
}

esp_err_t LSM6DSO::set_gyro_fs(GyroFs fs) {
    if (fs > GYRO_FS_125) return ESP_ERR_INVALID_ARG;

    uint8_t ctrl2;
    esp_err_t ret = _i2c.read_reg(_addr, REG_CTRL2_G, &ctrl2, 1);
    if (ret != ESP_OK) return ret;

    if (fs == GYRO_FS_125) {
        ctrl2 = (ctrl2 & 0xFC) | 0x02;
    } else {
        ctrl2 = (ctrl2 & 0xF3) | (fs << 2);
        ctrl2 &= ~0x02;
    }
    _gyro_scale = GYRO_SENSITIVITY[fs];
    return _i2c.write_reg(_addr, REG_CTRL2_G, &ctrl2, 1);
}

esp_err_t LSM6DSO::read_gyro(float &x, float &y, float &z) {
    uint8_t buf[6];
    esp_err_t ret = _i2c.read_reg(_addr, REG_OUTX_L_G, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)(buf[1] << 8) | buf[0];
    int16_t raw_y = (int16_t)(buf[3] << 8) | buf[2];
    int16_t raw_z = (int16_t)(buf[5] << 8) | buf[4];

    x = (float)raw_x * _gyro_scale;
    y = (float)raw_y * _gyro_scale;
    z = (float)raw_z * _gyro_scale;
    return ESP_OK;
}

esp_err_t LSM6DSO::read_temperature(float &temp_c) {
    uint8_t buf[2];
    esp_err_t ret = _i2c.read_reg(_addr, REG_OUT_TEMP_L, buf, 2);
    if (ret != ESP_OK) return ret;

    int16_t raw = (int16_t)(buf[1] << 8) | buf[0];
    temp_c = (float)raw / 256.0f + 25.0f;
    return ESP_OK;
}

esp_err_t LSM6DSO::set_int1_ctrl(uint8_t mask) {
    return _i2c.write_reg(_addr, REG_INT1_CTRL, &mask, 1);
}

esp_err_t LSM6DSO::set_int2_ctrl(uint8_t mask) {
    return _i2c.write_reg(_addr, REG_INT2_CTRL, &mask, 1);
}

#pragma once

#include <cstdint>
#include "I2CBase.hpp"
#include "esp_err.h"

#define LSM6DSO_DEFAULT_ADDR  0x6A
#define LSM6DSO_WHO_AM_I_VAL  0x6C

class LSM6DSO {
public:
    enum Register : uint8_t {
        REG_FIFO_CTRL1  = 0x06,
        REG_FIFO_CTRL2  = 0x07,
        REG_INT1_CTRL   = 0x0D,
        REG_INT2_CTRL   = 0x0E,
        REG_WHO_AM_I    = 0x0F,
        REG_CTRL1_XL    = 0x10,
        REG_CTRL2_G     = 0x11,
        REG_CTRL3_C     = 0x12,
        REG_CTRL4_C     = 0x13,
        REG_CTRL5_C     = 0x14,
        REG_CTRL6_C     = 0x15,
        REG_CTRL7_G     = 0x16,
        REG_CTRL8_XL    = 0x17,
        REG_CTRL9_XL    = 0x18,
        REG_CTRL10_C    = 0x19,
        REG_OUT_TEMP_L  = 0x20,
        REG_OUT_TEMP_H  = 0x21,
        REG_OUTX_L_G    = 0x22,
        REG_OUTX_H_G    = 0x23,
        REG_OUTY_L_G    = 0x24,
        REG_OUTY_H_G    = 0x25,
        REG_OUTZ_L_G    = 0x26,
        REG_OUTZ_H_G    = 0x27,
        REG_OUTX_L_A    = 0x28,
        REG_OUTX_H_A    = 0x29,
        REG_OUTY_L_A    = 0x2A,
        REG_OUTY_H_A    = 0x2B,
        REG_OUTZ_L_A    = 0x2C,
        REG_OUTZ_H_A    = 0x2D,
    };

    enum Ctrl3CBits : uint8_t {
        CTRL3_BOOT      = (1 << 7),
        CTRL3_BDU       = (1 << 6),
        CTRL3_H_LACTIVE = (1 << 5),
        CTRL3_PP_OD     = (1 << 4),
        CTRL3_SIM       = (1 << 3),
        CTRL3_IF_INC    = (1 << 2),
        CTRL3_SW_RESET  = (1 << 0),
    };

    enum AccelFs : uint8_t {
        ACCEL_FS_2G  = 0x00,
        ACCEL_FS_16G = 0x01,
        ACCEL_FS_4G  = 0x02,
        ACCEL_FS_8G  = 0x03,
    };

    enum GyroFs : uint8_t {
        GYRO_FS_250   = 0x00,
        GYRO_FS_500   = 0x01,
        GYRO_FS_1000  = 0x02,
        GYRO_FS_2000  = 0x03,
        GYRO_FS_125   = 0x04,
    };

    LSM6DSO(I2CBase &i2c, uint8_t addr = LSM6DSO_DEFAULT_ADDR);

    esp_err_t init();

    esp_err_t who_am_i(uint8_t &id);

    esp_err_t set_accel_odr(uint8_t odr);
    esp_err_t set_accel_fs(AccelFs fs);
    esp_err_t read_accel(float &x, float &y, float &z);

    esp_err_t set_gyro_odr(uint8_t odr);
    esp_err_t set_gyro_fs(GyroFs fs);
    esp_err_t read_gyro(float &x, float &y, float &z);

    esp_err_t read_temperature(float &temp_c);

    esp_err_t set_int1_ctrl(uint8_t mask);
    esp_err_t set_int2_ctrl(uint8_t mask);

    bool is_initialized() const { return _initialized; }

private:
    static constexpr float ACCEL_SENSITIVITY[4] = { 0.000061f, 0.000488f, 0.000122f, 0.000244f };
    static constexpr float GYRO_SENSITIVITY[5]  = { 0.00875f, 0.01750f, 0.0350f, 0.0700f, 0.004375f };

    I2CBase &_i2c;
    uint8_t _addr;
    float _accel_scale;
    float _gyro_scale;
    bool _initialized;
};

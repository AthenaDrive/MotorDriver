#pragma once

#include <cstdint>
#include "SPIBase.hpp"
#include "esp_err.h"

#define ENCODER_RW_READ       0x4000
#define ENCODER_RW_WRITE      0x0000
#define ENCODER_ADDR_MASK     0x3FFF
#define ENCODER_DATA_MASK     0x3FFF
#define ENCODER_PAR_BIT       0x8000

class AS5047P {
public:
    static constexpr uint16_t NOP        = 0x0000;
    static constexpr uint16_t ERRFL      = 0x3FFC;
    static constexpr uint16_t PROG       = 0x3FFD;
    static constexpr uint16_t ANGLEUNC   = 0x3FFE;
    static constexpr uint16_t ANGLECOM   = 0x3FFF;

    AS5047P(SPIBase &spi, uint8_t cs_gpio, int clock_speed_hz = 5000000);

    esp_err_t init();
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

    esp_err_t read_register(uint16_t address, uint16_t &data);
    esp_err_t write_register(uint16_t address, uint16_t data);

    esp_err_t read_angle(float &degrees, bool with_daec = true);
    esp_err_t read_angle_raw(uint16_t &raw, bool with_daec = true);

    esp_err_t pipeline_start();
    esp_err_t pipeline_read_angle(float &degrees, bool with_daec = true);

private:
    static uint16_t _apply_parity(uint16_t word);
    static float _raw_to_degrees(uint16_t raw);

    esp_err_t _transfer(uint16_t tx, uint16_t &rx);

    SPIBase &_spi;
    spi_device_handle_t _dev;
    uint8_t _cs_gpio;
    int _clock_speed_hz;
    bool _initialized;
    bool _pipeline_active;
};

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
    esp_err_t pipeline_read_angle(float &radians, bool with_daec = true);

    // Split pipelined read. pipeline_start_read() starts the 16-bit frame and
    // returns; pipeline_finish_read() waits for it and decodes the angle. The
    // caller can do independent work in between. The AS5047P returns the data
    // sampled at the end of the previous command, so splitting the transfer
    // inside one FOC iteration does not add any angle latency.
    esp_err_t pipeline_start_read(bool with_daec = true);
    esp_err_t pipeline_finish_read(float &radians);

private:
    static uint16_t _apply_parity(uint16_t word);
    static float _raw_to_degrees(uint16_t raw);
    static float _raw_to_radians(uint16_t raw);

    esp_err_t _transfer(uint16_t tx, uint16_t &rx);
    esp_err_t _transfer_start(uint16_t tx);
    esp_err_t _transfer_finish(uint16_t &rx);

    SPIBase &_spi;
    spi_device_handle_t _dev;
    uint8_t _cs_gpio;
    int _clock_speed_hz;
    bool _initialized;
    bool _pipeline_active;
    bool _transfer_pending;
    bool _first_pipeline_frame;
    uint8_t _tx_buf[2];
    uint8_t _rx_buf[2];
    spi_transaction_t _pending_trans;
};

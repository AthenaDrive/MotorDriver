#pragma once

#include <cstdint>
#include "hal/gpio_types.h"
#include "SPIBase.hpp"
#include "esp_err.h"

#define DRV8323_CS_GPIO    GPIO_NUM_48

class DRV8323 {
public:
    enum Register : uint8_t {
        REG_FAULT_STATUS_1 = 0x00,
        REG_VGS_STATUS_2   = 0x01,
        REG_DRIVER_CONTROL = 0x02,
        REG_GATE_DRIVE_HS  = 0x03,
        REG_GATE_DRIVE_LS  = 0x04,
        REG_OCP_CONTROL    = 0x05,
        REG_CSA_CONTROL    = 0x06,
    };

    enum DriverControlBits : uint16_t {
        PWM_MODE_6X        = (0x000 << 5),
        PWM_MODE_3X        = (0x001 << 5),
        PWM_MODE_1X        = (0x002 << 5),
        PWM_MODE_INDEP     = (0x003 << 5),
    };

    enum OCPControlBits : uint16_t {
        OCP_MODE_LATCH     = (0x000 << 6),
        OCP_MODE_AUTO_RETRY = (0x001 << 6),
        OCP_MODE_REPORT    = (0x002 << 6),
    };

    enum FaultBits : uint16_t {
        FAULT_FLT     = (1 << 10),
        FAULT_VDS_OCP = (1 << 9),
        FAULT_VDS_HA  = (1 << 8),
        FAULT_VDS_LA  = (1 << 7),
        FAULT_VDS_HB  = (1 << 6),
        FAULT_VDS_LB  = (1 << 5),
        FAULT_VDS_HC  = (1 << 4),
        FAULT_VDS_LC  = (1 << 3),
        FAULT_OTW     = (1 << 2),
        FAULT_OTSD    = (1 << 1),
        FAULT_UVLO    = (1 << 0),
    };

    DRV8323(SPIBase &spi, uint8_t cs_gpio = DRV8323_CS_GPIO,
            uint8_t spi_mode = 0, int clock_speed_hz = 5000000);

    esp_err_t init();
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

    esp_err_t write_register(uint8_t reg, uint16_t value);
    esp_err_t read_register(uint8_t reg, uint16_t &value);

    esp_err_t set_pwm_mode(uint16_t mode);
    esp_err_t set_3x_pwm_mode();
    esp_err_t read_fault_status(uint16_t &status);
    esp_err_t read_vgs_status(uint16_t &status);
    bool has_fault(uint16_t status, FaultBits bit) const;

private:
    esp_err_t _spi_transfer(uint16_t tx, uint16_t &rx);

    SPIBase &_spi;
    spi_device_handle_t _dev;
    uint8_t _cs_gpio;
    uint8_t _spi_mode;
    int _clock_speed_hz;
    bool _initialized;
};

#include "FOCTask.hpp"
#include "GlobalVariableManager.hpp"

#include <cmath>

FOCTask::FOCTask(FOCTaskConfig &config)
    : _config(config),
    _spi(SPI2_HOST, _config.cSPI0_CLK, _config.cSPI0_PICO, _config.cSPI0_POCI),
    _encoder(_spi, _config.cAS5047P_CS),
    _drv(_spi, _config.cDRV8323_CS, 1, 500000),
    _controller({1.0, 0.0, 1.0, 0.0, 10.0, 10.0}) {
}

void FOCTask::begin() {
    gpio_set_level(_config.cDRV8323_CS, 1);
    gpio_set_direction(_config.cDRV8323_CS, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(_pwm.init(30000, 40000000, nullptr, nullptr));
    ESP_ERROR_CHECK(_spi.init());
    ESP_ERROR_CHECK(_encoder.init());
    ESP_ERROR_CHECK(_drv.init());
    _drv.set_3x_pwm_mode();

    uint16_t drvReg;
    for (int i = 0; i < 8; i++) {
        auto err = _drv.read_register(i, drvReg);
        if (err != ESP_OK) {
            printf("Error when reading register %i.\n", i);
        }
        printf("Register %i: %i\n", i, drvReg);
    }
}

void FOCTask::update() {
    float angle, velocity;
    if (_encoder.completeRead(angle, velocity) == ESP_OK) {
        globalVariableManager.setAngle(angle);
        globalVariableManager.setVelocity(velocity);
    }


    uint16_t drv_fault, drv_vgs;
    if (_drv.read_fault_status(drv_fault) == ESP_OK) {
        if (_drv.has_fault(drv_fault, DRV8323::FAULT_FLT)) {
            // printf("DRV8323: FAULT=0x%04X\n", drv_fault);
        }
    }
    if (_drv.read_vgs_status(drv_vgs) == ESP_OK) {
        if (drv_vgs) {
            // printf("DRV8323: VGS=0x%04X\n", drv_vgs);
        }
    }

    float elPos = fmod((angle * globalVariableManager.getNumPolePairs()), GlobalVariableManager::TWO_PI);
    _out = _controller.update(0.0, elPos, velocity, 0.0, 0.0);

    if (fabs(_out.phaseA) > 0.1) {
        _out.phaseA = _out.phaseA < 0 ? -0.1 : 0.1;
    }

    if (fabs(_out.phaseB) > 0.1) {
        _out.phaseB = _out.phaseB < 0 ? -0.1 : 0.1;
    }

    if (fabs(_out.phaseC) > 0.1) {
        _out.phaseC = _out.phaseC < 0 ? -0.1 : 0.1;
    }

    _out.phaseA += 50.0;
    _out.phaseB += 50.0;
    _out.phaseC += 50.0;

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, _out.phaseA);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, _out.phaseB);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, _out.phaseC);

    printf("Angle: %f\n", angle);
}
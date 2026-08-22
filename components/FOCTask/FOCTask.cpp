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

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, 50.0f);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, 50.0f);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, 50.0f);

    globalVariableManager.setUdpCommandDebugFloat0(0.0f);
    globalVariableManager.setUdpCommandDebugFloat3(0.0f);
}

float constrain(float val, float minV, float maxV) {
    if (val > maxV) { return maxV; }
    if (val < minV) { return minV; }
    return val;
}

void FOCTask::update() {

    float angle, velocity, acceleration;
    if (_encoder.completeRead(angle, velocity, acceleration) == ESP_OK) {
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

    float iqRef  = globalVariableManager.getUdpCommandDebugFloat0();
    float offset = globalVariableManager.getUdpCommandDebugFloat3();
    float numPolePairs = -20.0;
    float elPos = fmod((angle * numPolePairs), GlobalVariableManager::TWO_PI);
    _out = _controller.update(iqRef, elPos + offset, 0.0f, 0.0f, 0.0f);

    float maxVal = 4.0f;
    if (fabs(_out.phaseA) > maxVal) {
        _out.phaseA = _out.phaseA < 0 ? -maxVal : maxVal;
    }

    if (fabs(_out.phaseB) > maxVal) {
        _out.phaseB = _out.phaseB < 0 ? -maxVal : maxVal;
    }

    if (fabs(_out.phaseC) > maxVal) {
        _out.phaseC = _out.phaseC < 0 ? -maxVal : maxVal;
    }

    _out.phaseA += 50.0;
    _out.phaseB += 50.0;
    _out.phaseC += 50.0;

    globalVariableManager.setUdpDataDebugFloat0(_out.phaseA);
    globalVariableManager.setUdpDataDebugFloat1(_out.phaseB);
    globalVariableManager.setUdpDataDebugFloat2(_out.phaseC);

    float tmpA = globalVariableManager.getUdpCommandDebugFloat0();
    float tmpB = globalVariableManager.getUdpCommandDebugFloat1();
    float tmpC = globalVariableManager.getUdpCommandDebugFloat2();

    // printf("Offset / Wanted / Computed: %f / %f, %f, %f / %f, %f, %f\n", offset, tmpA, tmpB, tmpC, _out.phaseA, _out.phaseB, _out.phaseC);

    tmpA = constrain(tmpA, 48.0f, 52.0f);
    tmpB = constrain(tmpB, 48.0f, 52.0f);
    tmpC = constrain(tmpC, 48.0f, 52.0f);

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, _out.phaseA);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, _out.phaseB);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, _out.phaseC);

    globalVariableManager.setAngle(angle);
    globalVariableManager.setVelocity(velocity);
}
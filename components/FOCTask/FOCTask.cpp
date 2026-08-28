#include "FOCTask.hpp"
#include "GlobalVariableManager.hpp"

#include <cmath>

FOCTask::FOCTask(FOCTaskConfig &config)
    : _config(config),
    _spi(SPI2_HOST, _config.cSPI0_CLK, _config.cSPI0_PICO, _config.cSPI0_POCI),
    _encoder(_spi, _config.cAS5047P_CS),
    _drv(_spi, _config.cDRV8323_CS, 1, 500000),
    _controller({1.0, 0.0, 1.0, 0.0, 10.0, 10.0}),
    _cascadePID({1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}),
    _prevTime(esp_timer_get_time()) {
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

    esp_timer_handle_t focTimer;
    esp_timer_create_args_t timerArgs = {
        .callback = taskEntry,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "FOCTimer",
        .skip_unhandled_events = true, // Never use light sleep, but whatever.
    };

    _adc.init(0.0035f, 20.0f, 1.65f);

    esp_timer_create(&timerArgs, &focTimer);
    // TODO!
    // Currently slower than 50us, just for debug.
    esp_timer_start_periodic(focTimer, 5000);
}

float constrain(float val, float minV, float maxV) {
    if (val > maxV) { return maxV; }
    if (val < minV) { return minV; }
    return val;
}

void FOCTask::update() {

    int64_t t0 = esp_timer_get_time();
    // Yes, i know dt is not in seconds.
    // Dont really care, just adjust PID tune.
    float dt = static_cast<float>(t0 - _prevTime);
    _prevTime = t0;

    float angle, cumulativeAngle, velocity, acceleration;
    if (_encoder.pipeline_read_angle(angle, true) != ESP_OK) {
        return;
    }

    float Ia, Ib, Ic;
    _adc.read_current_amps(ADCOneshot::CHANNEL_A, Ia);
    _adc.read_current_amps(ADCOneshot::CHANNEL_B, Ib);
    _adc.read_current_amps(ADCOneshot::CHANNEL_C, Ic);

    _stateEstimation.estimate(angle, cumulativeAngle, velocity, acceleration);
    globalVariableManager.setAngle(cumulativeAngle);
    globalVariableManager.setVelocity(velocity);
    globalVariableManager.setAcceleration(acceleration);

    uint16_t drv_fault = 0;
    uint16_t drv_vgs = 0;
    // if (_drv.read_fault_status(drv_fault) == ESP_OK) {
    //     if (_drv.has_fault(drv_fault, DRV8323::FAULT_FLT)) {
    //         // printf("DRV8323: FAULT=0x%04X\n", drv_fault);
    //     }
    // }
    // if (_drv.read_vgs_status(drv_vgs) == ESP_OK) {
    //     if (drv_vgs) {
    //         // printf("DRV8323: VGS=0x%04X\n", drv_vgs);
    //     }
    // }
    // TODO: Not sure if this will be fucky wucky since datatype is 16 bit.
    // globalVariableManager.setErrorFlags((drv_fault << 16) + drv_vgs);

    float iqRef = 0.0f;
    _cascadePID.compute(iqRef, dt);

    float numPolePairs = -20.0;
    float elPos = fmod((angle * numPolePairs), GlobalVariableManager::TWO_PI);

    // TODO: Need to actually use velocity when its not horribly noisy.
    _out = _controller.update(iqRef, elPos + _elPosOffset, 0.0f, 0.0f, 0.0f);

    float maxVal = 20.0f;
    _out.phaseA = constrain(_out.phaseA, -maxVal, maxVal);
    _out.phaseB = constrain(_out.phaseB, -maxVal, maxVal);
    _out.phaseC = constrain(_out.phaseC, -maxVal, maxVal);

    _out.phaseA += 50.0;
    _out.phaseB += 50.0;
    _out.phaseC += 50.0;

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, _out.phaseA);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, _out.phaseB);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, _out.phaseC);

    globalVariableManager.setTorqueSetpoint(iqRef);

    int64_t t1 = esp_timer_get_time();
    // TODO: Add small lowpass maybe? Or rename variable
    // I know in theory the 64 bit time could overflow
    // the 32 bit, dont care, probably not going to happen.
    globalVariableManager.setAvgLoopTimeFOC(t1 - t0);
}

void FOCTask::taskEntry(void *pvParameters) {
    FOCTask *focTask = static_cast<FOCTask *>(pvParameters);
    focTask->update();
}
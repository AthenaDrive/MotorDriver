
#include "PI.hpp"
#include "esp_attr.h"

PI_Reg::PI_Reg(float kp, float ki)
    : _kp(kp), _ki(ki){
}

IRAM_ATTR float PI_Reg::update(float error, float dt, float integralLimit) {
    // Integrate in volts: Ki [V/(A*s)] * error [A] * dt [s].
    // A non-positive dt (first sample or a clock glitch) must not corrupt the
    // state, so hold the integral and only run the proportional path.
    if (dt > 0.0f) {
        _integral += _ki * error * dt;
    }

    // Hard bound as a final guard: with a hostile dt or a huge error the
    // integral can never run off on its own.
    if (_integral > integralLimit) {
        _integral = integralLimit;
    } else if (_integral < -integralLimit) {
        _integral = -integralLimit;
    }

    return _kp * error + _integral;
}

IRAM_ATTR void PI_Reg::track(float error, float output, float integralLimit) {
    float integral = output - _kp * error;
    if (integral > integralLimit) {
        integral = integralLimit;
    } else if (integral < -integralLimit) {
        integral = -integralLimit;
    }
    _integral = integral;
}

void PI_Reg::setKp(float kp) {
    _kp = kp;
}

void PI_Reg::setKi(float ki) {
    _ki = ki;
}

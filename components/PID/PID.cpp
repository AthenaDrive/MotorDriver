#include <stdio.h>
#include "PID.hpp"
#include "esp_attr.h"


PID_Reg::PID_Reg(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd){
}

IRAM_ATTR void PID_Reg::setKp(float kp) {
    _kp = kp;
}

IRAM_ATTR void PID_Reg::setKi(float ki) {
    _ki = ki;
}

IRAM_ATTR void PID_Reg::setKd(float kd) {
    _kd = kd;
}

IRAM_ATTR void PID_Reg::setSetpoint(float sp) {
    _setpoint = sp;
}

IRAM_ATTR float PID_Reg::update(float currVal, float dt) {
    float error = _setpoint - currVal;

    _integral += error * dt;

    float derivative = 0.0f;
    // Skip the divide entirely when Kd is zero. The cascade gains currently
    // have Kd = 0, so this keeps the 10 kHz path off the soft-float helper.
    if (_kd != 0.0f && !_firstRun && dt > 0.0f) {
        derivative = (error - _prevError) / dt;
    }
    _firstRun = false;

    float out = _kp * error + _ki * _integral + _kd * derivative;

    // Avoid std::abs/fabsf here so the hot path stays a plain compare; this
    // function runs from the IRAM FOC loop and must not take a flash miss.
    if (out > _maxOut || out < -_maxOut) {
        _integral -= error * dt;    // anti-windup
        out = _maxOut * (out > 0.0f ? 1.0f : -1.0f);
    }

    _prevError = error;
    return out;
}

IRAM_ATTR void PID_Reg::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _firstRun = true;
}

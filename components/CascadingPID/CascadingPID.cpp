
#include "CascadingPID.hpp"
#include "esp_attr.h"

CascadingPID::CascadingPID(CascadePidConfig config)
    : _velocityPID({config.velKp, config.velKi, config.velKd}),
    _positionPID({config.posKp, config.posKi, config.posKd}),
    _gains(config) {}

IRAM_ATTR void CascadingPID::setGains(const CascadePidConfig &config) {
    bool doResetPos = false;
    bool doResetVel = false;
    
    if (config.posKp != _gains.posKp) {
        _positionPID.setKp(config.posKp);
        _gains.posKp = config.posKp;

        doResetPos = true;
    }
    if (config.posKi != _gains.posKi) {
        _positionPID.setKi(config.posKi);
        _gains.posKi = config.posKi;

        doResetPos = true;
    }
    if (config.posKd != _gains.posKd) {
        _positionPID.setKd(config.posKd);
        _gains.posKd = config.posKd;

        doResetPos = true;
    }
    if (config.velKp != _gains.velKp) {
        _velocityPID.setKp(config.velKp);
        _gains.velKp = config.velKp;

        doResetVel = true;
    }
    if (config.velKi != _gains.velKi) {
        _velocityPID.setKi(config.velKi);
        _gains.velKi = config.velKi;

        doResetVel = true;
    }
    if (config.velKd != _gains.velKd) {
        _velocityPID.setKd(config.velKd);
        _gains.velKd = config.velKd;

        doResetVel = true;
    }

    if (doResetPos) { _positionPID.reset(); }
    if (doResetVel) { _velocityPID.reset(); }
}

IRAM_ATTR esp_err_t CascadingPID::compute(float &iqRef, float dt, uint32_t drivingMode, CascadePidValues &values) {
    if (drivingMode != _lastMode) {
        _positionPID.reset();
        _velocityPID.reset();
        _lastMode = drivingMode;
    }

    if (
        (drivingMode == 3)
    ) {
        // Position Control

        _positionPID.setSetpoint(values.positionSetpoint);
        values.velocitySetpoint = _positionPID.update(values.position, dt);
    }

    if (
        (drivingMode == 3) ||
        (drivingMode == 4)
    ) {
        // Position or velocity control

        _velocityPID.setSetpoint(values.velocitySetpoint);
        values.torqueSetpoint = _velocityPID.update(values.velocity, dt);
    }

    if (
        (drivingMode == 3) ||
        (drivingMode == 4) ||
        (drivingMode == 5)
    ) {
        // Position, velocity, or torque control

        iqRef = values.torqueSetpoint;
        return ESP_OK;
    }

    // Not implemented different modes yet.
    return ESP_FAIL;
}

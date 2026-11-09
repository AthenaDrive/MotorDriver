
#include "CascadingPID.hpp"

CascadingPID::CascadingPID(CascadePidConfig config)
    : _velocityPID({config.velKp, config.velKi, config.velKd}),
    _positionPID({config.posKp, config.posKi, config.posKd}) {}

esp_err_t CascadingPID::compute(float &iqRef, float dt, uint32_t drivingMode, CascadePidValues &values) {
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

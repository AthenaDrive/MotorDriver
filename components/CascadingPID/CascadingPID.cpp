
#include "CascadingPID.hpp"
#include "GlobalVariableManager.hpp"

CascadingPID::CascadingPID(CascadePidConfig config)
    : _velocityPID({config.velKp, config.velKi, config.velKd}),
    _positionPID({config.posKp, config.posKi, config.posKd}) {}

esp_err_t CascadingPID::compute(float &iqRef, float dt) {
    uint32_t drivingMode = globalVariableManager.getDrivingMode();

    if (
        (drivingMode == 3)
    ) {
        // Position Control

        _positionPID.setSetpoint(globalVariableManager.getPositionSetpoint());
        float velSetpoint = _positionPID.update(globalVariableManager.getAngle(), dt);
        globalVariableManager.setVelocitySetpoint(velSetpoint);
    }

    if (
        (drivingMode == 3) ||
        (drivingMode == 4)
    ) {
        // Position or velocity control

        _velocityPID.setSetpoint(globalVariableManager.getVelocitySetpoint());
        float torqueSetpoint = _velocityPID.update(globalVariableManager.getVelocity(), dt);
        globalVariableManager.setTorqueSetpoint(torqueSetpoint);
    }

    if (
        (drivingMode == 3) ||
        (drivingMode == 4) ||
        (drivingMode == 5)
    ) {
        // Position, velocity, or torque control

        iqRef = globalVariableManager.getTorqueSetpoint();
        return ESP_OK;
    }

    // Not implemented different modes yet.
    return ESP_FAIL;
}

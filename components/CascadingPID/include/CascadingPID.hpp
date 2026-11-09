#pragma once

#include <cstdint>

#include "esp_err.h"

#include "PID.hpp"

struct CascadePidConfig {
    float posKp;
    float posKi;
    float posKd;
    float velKp;
    float velKi;
    float velKd;
};

// Values used and produced by one cascade step. The caller owns these and is
// responsible for syncing them with the GVM; this class has no GVM access.
struct CascadePidValues {
    float positionSetpoint = 0.0f;
    float position = 0.0f;
    float velocitySetpoint = 0.0f; // Updated by position mode (3).
    float velocity = 0.0f;
    float torqueSetpoint = 0.0f;   // Updated by position/velocity modes (3, 4).
};

class CascadingPID {
public:
    CascadingPID(CascadePidConfig config);

    // drivingMode: 3 = position, 4 = velocity, 5 = torque.
    // Returns ESP_FAIL for modes that are not implemented.
    esp_err_t compute(float &iqRef, float dt, uint32_t drivingMode, CascadePidValues &values);

private:
    PID_Reg _velocityPID;
    PID_Reg _positionPID;
};

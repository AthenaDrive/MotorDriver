
#pragma once

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

class CascadingPID {
public:
    CascadingPID(CascadePidConfig config);

    // Need to make sure to update GVM before calling.
    esp_err_t compute(float &iqRef, float dt);

private:
    PID_Reg _velocityPID;
    PID_Reg _positionPID;
};
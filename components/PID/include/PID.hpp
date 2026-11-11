#pragma once
#include <cmath>

class PID_Reg {
public:
    PID_Reg(float kp, float ki, float kd);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    
    void setSetpoint(float sp);
    void setMaxOut(float maxOut) { _maxOut = std::abs(maxOut); }
    float update(float currVal, float dt);

    // Clear the integral and derivative history, e.g. when the driving mode
    // changes and a stale integrator would otherwise kick the motor.
    void reset();

private:
    float _kp, _ki, _kd;
    float _setpoint = 0.0;
    float _integral = 0.0;
    float _prevError = 0.0;
    float _maxOut = 15.0;
    bool _firstRun = true;
};
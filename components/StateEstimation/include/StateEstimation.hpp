#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_timer.h"

class StateEstimation {
public:
    StateEstimation();
    esp_err_t estimate(float encoderAngleRead, float &cumulativeAngle, float &velocity, float &acceleration);

    // Currently using the double derivate.
    // Extremely noisy for velocity.
    // And straight up does NOT work for acceleration.
    // TODO: Kalman? PPL?

private:
    float _cumulativeAngle;
    float _velocity;
    float _acceleration;

    float _prev_angle = 0.0f;
    float _prev_velocity = 0.0f;
    
    int64_t _prev_time_us = 0;
    bool _has_prev_read = false;
};
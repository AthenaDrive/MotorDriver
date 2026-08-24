
#include "StateEstimation.hpp"

#include <cmath>

StateEstimation::StateEstimation() {}

esp_err_t StateEstimation::estimate(float encoderAngleRead, float &cumulativeAngle, float &velocity, float &acceleration) {

    float delta = encoderAngleRead - _prev_angle;

    while (delta > M_PI)  delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;

    int64_t now = esp_timer_get_time();
    if (_has_prev_read && now > _prev_time_us) {
        float dt = (now - _prev_time_us) / 1e6f;
        velocity = delta / dt;
        acceleration = (velocity - _prev_velocity) / dt;

        _cumulativeAngle += delta;
    } else {
        velocity = 0.0f;
        acceleration = 0.0f;

        _cumulativeAngle = encoderAngleRead;
    }

    cumulativeAngle = _cumulativeAngle;

    _prev_angle = encoderAngleRead;
    _prev_velocity = velocity;
    _prev_time_us = now;
    _has_prev_read = true;

    return ESP_OK;
}
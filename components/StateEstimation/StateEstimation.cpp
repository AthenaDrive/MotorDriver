
#include "StateEstimation.hpp"

#include <cmath>

#include "esp_attr.h"

// Keep the wrap in single precision. M_PI is a double, and on the ESP32-S3
// that silently turns every comparison/subtraction into a soft-double call
// (__gtdf2/__subdf3/...), which dominated this function's runtime.
static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 6.28318530717958647692f;

StateEstimation::StateEstimation() {}

IRAM_ATTR esp_err_t StateEstimation::estimate(float encoderAngleRead, float &cumulativeAngle, float &velocity, float &acceleration, float dt) {


    float delta = encoderAngleRead - _prev_angle;

    // One step at 10 kHz can never exceed half a mechanical turn; the range
    // check also covers the first sample, where _prev_angle is still zero.
    if (delta > PI_F) {
        delta -= TWO_PI_F;
    } else if (delta < -PI_F) {
        delta += TWO_PI_F;
    }

    if (_has_prev_read && now > _prev_time_us) {
        velocity = _lowpassVelocity.update(delta / dt);
        acceleration = _lowpassAcceleration.update((velocity - _prev_velocity) / dt);

        _cumulativeAngle += delta;
    } else {
        velocity = 0.0f;
        acceleration = 0.0f;

        _cumulativeAngle = encoderAngleRead;
    }

    cumulativeAngle = _lowpassAngle.update(_cumulativeAngle);

    _prev_angle = encoderAngleRead;
    _prev_velocity = velocity;
    _has_prev_read = true;

    return ESP_OK;
}
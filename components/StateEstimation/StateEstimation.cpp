#include "StateEstimation.hpp"

#include "esp_attr.h"

// Keep these in single precision. On the ESP32-S3 a double constant silently
// turns the wrap into soft-double calls (__subdf3/__gtdf2/...), which used to
// dominate this function's runtime.
static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 6.28318530717958647692f;
static constexpr float INV_TWO_PI_F = 0.15915494309189535f;

// Fold an angle into (-pi, pi]. This is the error between two angles, where
// the sign matters; fold to [0, 2*pi) for the PLL phase state instead.
static inline IRAM_ATTR float wrapSigned(float a) {
    const int turns = static_cast<int>(a * INV_TWO_PI_F + (a >= 0.0f ? 0.5f : -0.5f));
    a -= static_cast<float>(turns) * TWO_PI_F;
    if (a > PI_F) {
        a -= TWO_PI_F;
    } else if (a < -PI_F) {
        a += TWO_PI_F;
    }
    return a;
}

static inline IRAM_ATTR float wrapPositive(float a) {
    const int turns = static_cast<int>(a * INV_TWO_PI_F);
    a -= static_cast<float>(turns) * TWO_PI_F;
    if (a < 0.0f) {
        a += TWO_PI_F;
    } else if (a >= TWO_PI_F) {
        a -= TWO_PI_F;
    }
    return a;
}

StateEstimation::StateEstimation(const StateEstimationConfig &config)
    : _config(config) {
    const float wn = TWO_PI_F * _config.cBandwidthHz;
    _twoZetaWn = 2.0f * _config.cDamping * wn;
    _wnSq = wn * wn;

    const float accelWn = TWO_PI_F * _config.cAccelLowpassHz;
    _accelTau = 1.0f / accelWn;
}

IRAM_ATTR void StateEstimation::reset() {
    _thetaHat = 0.0f;
    _omegaHat = 0.0f;
    _cumulativeAngle = 0.0f;
    _acceleration = 0.0f;
    _prevAngle = 0.0f;
    _prevOmega = 0.0f;
    _pendingDt = 0.0f;
    _initialized = false;
}

IRAM_ATTR esp_err_t StateEstimation::estimate(float encoderAngleRead, float &cumulativeAngle,
                                              float &velocity, float &acceleration, float dt) {
    // Reject a zero/negative/NaN dt without touching the state. The caller
    // keeps the previous outputs, so a scheduling hiccup does not inject a
    // huge velocity spike.
    if (!(dt > 0.0f)) {
        cumulativeAngle = _cumulativeAngle;
        velocity = _omegaHat;
        acceleration = _acceleration;
        return ESP_ERR_INVALID_ARG;
    }

    if (!_initialized) {
        // First sample: seed the phase from the encoder, not from zero. The
        // old filter implementation ramped the published position up from 0
        // over its time constant because its internal state started at 0.
        _thetaHat = wrapPositive(encoderAngleRead);
        _omegaHat = 0.0f;
        _acceleration = 0.0f;
        _prevOmega = 0.0f;
        _cumulativeAngle = encoderAngleRead;
        _prevAngle = encoderAngleRead;
        _pendingDt = 0.0f;
        _initialized = true;

        cumulativeAngle = _cumulativeAngle;
        velocity = _omegaHat;
        acceleration = _acceleration;
        return ESP_OK;
    }

    const float dtTotal = dt + _pendingDt;

    if (dtTotal > _config.cMaxDtSeconds) {
        // The loop stalled or samples were rejected for a long time. Re-seed
        // the phase from the measurement but keep the velocity state, so
        // recovery does not look like a hard stop.
        const float delta = wrapSigned(encoderAngleRead - _prevAngle);
        _cumulativeAngle += delta;
        _thetaHat = wrapPositive(encoderAngleRead);
        _prevAngle = encoderAngleRead;
        _prevOmega = _omegaHat;
        _acceleration = 0.0f;
        _pendingDt = 0.0f;

        cumulativeAngle = _cumulativeAngle;
        velocity = _omegaHat;
        acceleration = _acceleration;
        return ESP_OK;
    }

    const float delta = wrapSigned(encoderAngleRead - _prevAngle);

    // Gross-error gate. Normal motion at 2000 rad/s moves at most 0.2 rad per
    // 100 us sample, well below the half-turn jump a corrupt frame can cause.
    // The 0.01 rad margin covers quantization and timing jitter.
    const float maxStep = _config.cMaxSpeedRadPerSec * dtTotal + 0.01f;
    if (delta > maxStep || delta < -maxStep) {
        // Keep _prevAngle at the last accepted sample and carry the elapsed
        // time forward, so the next good sample is integrated over the whole
        // gap instead of being treated as one 100 us step.
        _pendingDt = dtTotal;
        cumulativeAngle = _cumulativeAngle;
        velocity = _omegaHat;
        acceleration = _acceleration;
        return ESP_ERR_INVALID_RESPONSE;
    }

    _cumulativeAngle += delta;

    // Type-2 tracking loop. The position correction is folded into the phase
    // update; the velocity integrator is what makes constant-speed tracking
    // lag-free.
    const float error = wrapSigned(encoderAngleRead - _thetaHat);
    _omegaHat += _wnSq * dtTotal * error;

    // Clamp the state as well as gating the input, so a marginal glitch cannot
    // wind the velocity up beyond what the motor can physically do.
    if (_omegaHat > _config.cMaxSpeedRadPerSec) {
        _omegaHat = _config.cMaxSpeedRadPerSec;
    } else if (_omegaHat < -_config.cMaxSpeedRadPerSec) {
        _omegaHat = -_config.cMaxSpeedRadPerSec;
    }

    _thetaHat = wrapPositive(_thetaHat + _omegaHat * dtTotal + _twoZetaWn * dtTotal * error);

    // Acceleration is a filtered derivative of the (already smooth) velocity
    // state. alpha is the backward-Euler form of a first-order low-pass, so it
    // stays correct when dt varies and avoids expf() in the FOC loop.
    const float accelRaw = (_omegaHat - _prevOmega) / dtTotal;
    const float alpha = dtTotal / (_accelTau + dtTotal);
    _acceleration += (accelRaw - _acceleration) * alpha;
    _prevOmega = _omegaHat;

    _prevAngle = encoderAngleRead;
    _pendingDt = 0.0f;

    cumulativeAngle = _cumulativeAngle;
    velocity = _omegaHat;
    acceleration = _acceleration;
    return ESP_OK;
}

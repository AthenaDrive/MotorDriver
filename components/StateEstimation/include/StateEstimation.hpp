#pragma once

#include "esp_err.h"

// Rotor state estimator.
//
// The previous implementation differentiated the encoder angle and ran the
// result through first-order low-pass filters. At 10 kHz an alpha of 0.001 is
// a 1.6 Hz filter (tau = 100 ms), so it could not be both quiet and fast.
// This class uses a type-2 tracking PLL instead:
//
//     e        = wrap(thetaMeas - thetaHat)
//     omegaHat += wn^2 * dt * e
//     thetaHat += omegaHat * dt + 2*zeta*wn*dt * e
//
// Velocity is a loop state, not a derivative of a quantized angle, and a
// type-2 loop has no steady-state lag at constant speed. Under constant
// acceleration the velocity state lags by 2*zeta*accel/wn, so raise the
// bandwidth if that matters. Position is accumulated from the raw deltas and
// is NOT filtered: the position loop and the Park transform need the real
// angle, and 14-bit quantization is only ~0.022 degrees.
struct StateEstimationConfig {
    // PLL natural frequency in Hz. Higher tracks faster, lower is quieter.
    // 100 Hz is about 60x faster than the old 1.6 Hz velocity filter while
    // keeping the estimated velocity noise near 0.02 rad/s at 10 kHz.
    float cBandwidthHz = 50.0f;

    // PLL damping. 1.0 is critically damped; 0.9-1.2 is a reasonable range.
    float cDamping = 1.0f;

    // Low-pass on the acceleration output (telemetry / feedforward only).
    float cAccelLowpassHz = 50.0f;

    // Plausibility limit. A sample that implies a larger speed is rejected,
    // and the velocity state is clamped to this value.
    float cMaxSpeedRadPerSec = 2000.0f;

    // If samples are missing for longer than this, re-seed the phase instead of
    // applying one huge update. The PLL discretization is only well-conditioned
    // while wn*dt is small; 1 ms is 10 missed 10 kHz samples and still stable.
    float cMaxDtSeconds = 0.001f;
};

class StateEstimation {
public:
    explicit StateEstimation(const StateEstimationConfig &config = StateEstimationConfig{});

    // encoderAngleRead is the wrapped mechanical angle in radians. dt is the
    // measured time since the last accepted sample, in seconds.
    //
    // Returns ESP_OK when the sample was used, ESP_ERR_INVALID_ARG for a bad dt
    // and ESP_ERR_INVALID_RESPONSE when the sample failed the plausibility
    // check. On either error the outputs keep their previous values.
    esp_err_t estimate(float encoderAngleRead, float &cumulativeAngle, float &velocity,
                       float &acceleration, float dt);

    // Forget the loop state. The next estimate() call re-seeds from its input.
    void reset();

private:
    StateEstimationConfig _config;

    // Precomputed from the config, so the hot path is multiply-adds only.
    float _twoZetaWn;
    float _wnSq;
    float _accelTau;

    // PLL states. _thetaHat is kept in [0, 2*pi) so it does not lose float
    // resolution after a long run.
    float _thetaHat = 0.0f;
    float _omegaHat = 0.0f;

    float _cumulativeAngle = 0.0f;
    float _acceleration = 0.0f;

    float _prevAngle = 0.0f;
    float _prevOmega = 0.0f;

    // Time carried over when samples are rejected, so the next accepted sample
    // is integrated over the real elapsed time.
    float _pendingDt = 0.0f;

    bool _initialized = false;
};

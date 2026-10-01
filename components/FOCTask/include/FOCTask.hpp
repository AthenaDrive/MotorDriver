#pragma once

#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "MCPWMDriver.hpp"
#include "ADCOneshot.hpp"
#include "Controller.hpp"
#include "StateEstimation.hpp"
#include "CascadingPID.hpp"

#include "AS5047P.hpp"
#include "DRV8323.hpp"

struct FOCTaskConfig
{
    gpio_num_t cDRV8323_CS;
    gpio_num_t cAS5047P_CS;
    gpio_num_t cSPI0_CLK;
    gpio_num_t cSPI0_PICO;
    gpio_num_t cSPI0_POCI;
};


class FOCTask {
public:
    FOCTask(FOCTaskConfig &config);
    void begin();

    void update();

private:
    FOCTaskConfig _config;

    SPIBase _spi;
    MCPWMDriver _pwm;
    AS5047P _encoder;
    ADCOneshot _adc;
    StateEstimation _stateEstimation;
    DRV8323 _drv;
    Controller _controller;
    CascadingPID _cascadePID;
    Output _out;

    int64_t _prevTime;

    // --- Rotor / encoder electrical offset calibration ---
    //
    // Hold a known stator voltage vector, let the rotor lock onto it, then
    // measure where it locked. That mechanical angle, converted to electrical
    // radians, becomes _elPosOffset.
    enum class AlignPhase : uint8_t {
        SETTLE,   // Rotor is swinging into alignment, do not measure yet.
        MEASURE,  // Rotor should be locked, accumulate angle samples.
        DONE,
    };

    // Alignment vector magnitude in duty percentage points. The vector is
    // (d, -d/2, -d/2), which is alpha = d, beta = 0 in the frame Controller
    // uses, so it sits at exactly 0 electrical degrees. Big enough to beat
    // cogging, small enough to stay gentle on the windings.
    static constexpr float ALIGN_VOLTAGE = 2.0f;

    // Loops spent letting the rotor swing before we start trusting readings.
    static constexpr uint32_t ALIGN_SETTLE_LOOPS = 5000;

    // Loops spent averaging the locked rotor angle.
    static constexpr uint32_t ALIGN_MEASURE_LOOPS = 1000;

    // If the rotor jumps further than this (radians) between consecutive
    // samples during measurement the offset is probably noisy. Note this is
    // computed with a signed wrap, so it measures real displacement.
    static constexpr float ALIGN_MAX_STEP_WARN = 0.05f;

    // Mean resultant length below this means the samples had no common
    // direction, i.e. the rotor was still spinning rather than locked onto the
    // alignment field. 0.0 = uniformly spread, 1.0 = perfectly stationary.
    static constexpr float ALIGN_MIN_RESULTANT = 0.9f;

    AlignPhase _alignPhase = AlignPhase::SETTLE;
    uint32_t _alignLoops = 0;

    // Circular mean accumulators. Summing sin and cos keeps the result correct
    // across the +-pi wrap, where a plain average of angles collapses toward
    // the middle of the range and can be wrong by up to pi.
    float _alignSinSum = 0.0f;
    float _alignCosSum = 0.0f;
    uint32_t _alignSamples = 0;

    // Largest step between consecutive samples, used only to sanity check that
    // the rotor really was stationary while we measured.
    float _alignPrevAngle = 0.0f;
    float _alignMaxStep = 0.0f;

    // Electrical zero point in radians, added to (angle * numPolePairs).
    float _elPosOffset = 0.0f;

    static void taskEntry(void *pvParameters);
};


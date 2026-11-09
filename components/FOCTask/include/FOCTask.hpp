#pragma once

#include <cstdint>
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

enum class PhaseOrderMode : uint8_t {
    AUTO,      // Detect the phase order during startup.
    NORMAL,    // Force A-B-C phase order.
    REVERSED,  // Force A-C-B phase order (software wire swap).
};

struct FOCTaskConfig
{
    gpio_num_t cDRV8323_CS;
    gpio_num_t cAS5047P_CS;
    gpio_num_t cSPI0_CLK;
    gpio_num_t cSPI0_PICO;
    gpio_num_t cSPI0_POCI;
    gpio_num_t cDRV8323_ENABLE;
    gpio_num_t cDRV8323_INL;
    gpio_num_t cDRV8323_nFAULT;

    // The DRV8323 cannot start before the bus has reached its minimum
    // operating voltage. Keep a margin above the 6 V datasheet minimum.
    float cMinBusVoltage = 9.0f;

    // Extra time the bus must stay above cMinBusVoltage before the driver is
    // enabled, so the supply has finished settling.
    uint32_t cStartupSettleMs = 500;

    // AUTO runs a short direction test after the alignment lock. NORMAL and
    // REVERSED skip the test and force the corresponding phase order.
    PhaseOrderMode cPhaseOrderMode = PhaseOrderMode::AUTO;

    // When false, the FOC Clarke transform uses phase A and C and reconstructs
    // phase B as Ib = -Ia - Ic. The real B current is still measured and
    // published through the GVM, but it does not affect the control loop.
    bool cUsePhaseBForFoc = false;
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

    int CURRENT_BASELINE_LOOPS = 10000;
    int CURRENT_BASELINE_SETTLE = 5000;

    int64_t sumPhaseOffsetA = 0;
    int64_t sumPhaseOffsetB = 0;
    int64_t sumPhaseOffsetC = 0;

    int numCurrentReadings = 0;

    int phaseOffsetA;
    int phaseOffsetB;
    int phaseOffsetC;

    LowpassFilter lowpassCurrentA{0.1};
    LowpassFilter lowpassCurrentB{0.1};
    LowpassFilter lowpassCurrentC{0.1};

    // --- Rotor / encoder electrical offset calibration ---
    //
    // Hold a known stator voltage vector, let the rotor lock onto it, then
    // measure where it locked. That mechanical angle, converted to electrical
    // radians, becomes _elPosOffset.
    enum class AlignPhase : uint8_t {
        NO_VOLTAGE,      // Wait for the bus before touching the DRV8323.
        VOLTAGE_SETTLE,  // Bus is high; give the supply time to settle.
        INIT_DRV,        // Configure and enable the DRV8323.
        START_ONE_PHASE, // Apply the alignment field.
        SETTLE,          // Rotor is swinging into alignment, do not measure yet.
        MEASURE,         // Rotor should be locked, accumulate angle samples.
        DIRECTION_RAMP,    // Rotate the field +90 electrical.
        DIRECTION_SETTLE,  // Let the rotor finish moving.
        DIRECTION_MEASURE, // Measure which way the rotor moved.
        CURRENT,         // Measure the zero-current ADC offsets.
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

    // Direction check: ramp the alignment field from 0 to +90 electrical
    // degrees, let the rotor settle, then measure which way it moved. A slow
    // ramp makes the rotor follow the field instead of relying on a single
    // nudge that cogging could swallow.
    static constexpr uint32_t DIRECTION_RAMP_LOOPS = 1000;
    static constexpr uint32_t DIRECTION_SETTLE_LOOPS = 1000;
    static constexpr uint32_t DIRECTION_MEASURE_LOOPS = 500;

    // If the rotor moved less than this (electrical radians) during the
    // direction check the result is not trustworthy.
    static constexpr float DIRECTION_MIN_ELEC_DELTA = 0.05f;

    AlignPhase _alignPhase = AlignPhase::NO_VOLTAGE;
    uint32_t _alignLoops = 0;

    // Timestamp for the VOLTAGE_SETTLE phase. A timestamp instead of a loop
    // count keeps the delay independent of the timer period.
    int64_t _voltageSettleStartUs = 0;

    void initializeDriver();
    void applyAlignmentVoltage();
    void applyVoltageVector(float electricalAngle);

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

    // Mechanical angle where the rotor locked during the 0-degree alignment.
    // Used as the reference for the direction check.
    float _alignLockedAngle = 0.0f;

    // True when the motor phase sequence is reversed relative to the encoder.
    bool _reversePhaseSequence = false;

    static void taskEntry(void *pvParameters);
};


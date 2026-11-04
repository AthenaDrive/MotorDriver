#include "FOCTask.hpp"
#include "GlobalVariableManager.hpp"

#include <cmath>

FOCTask::FOCTask(FOCTaskConfig &config)
    : _config(config),
    _spi(SPI2_HOST, _config.cSPI0_CLK, _config.cSPI0_PICO, _config.cSPI0_POCI),
    _encoder(_spi, _config.cAS5047P_CS),
    _drv(_spi, _config.cDRV8323_CS, 1, 500000),
    _controller({1.0, 0.0, 1.0, 0.0, 10.0, 10.0}),
    _cascadePID({1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}),
    _prevTime(esp_timer_get_time()) {
}

void FOCTask::begin() {
    if (_config.cPhaseOrderMode != PhaseOrderMode::AUTO) {
        const bool reversed = (_config.cPhaseOrderMode == PhaseOrderMode::REVERSED);
        _controller.setPhaseOrder(reversed);
        printf("FOC: phase order forced to %s.\n", reversed ? "REVERSED" : "NORMAL");
    }

    // Keep CS deasserted while the bus and the other devices are set up. The
    // DRV8323 itself is initialized later, after the bus voltage is valid.
    gpio_set_direction(_config.cDRV8323_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(_config.cDRV8323_CS, 1);

    gpio_config_t drvControl = {};
    drvControl.pin_bit_mask = (1ULL << _config.cDRV8323_ENABLE) |
                              (1ULL << _config.cDRV8323_INL);
    drvControl.mode = GPIO_MODE_OUTPUT;
    drvControl.pull_up_en = GPIO_PULLUP_DISABLE;
    drvControl.pull_down_en = GPIO_PULLDOWN_DISABLE;
    drvControl.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&drvControl));

    gpio_set_level(_config.cDRV8323_ENABLE, 1);
    gpio_set_level(_config.cDRV8323_INL, 0);
    gpio_set_direction(_config.cDRV8323_nFAULT, GPIO_MODE_INPUT);

    ESP_ERROR_CHECK(_pwm.init(30000, 40000000, nullptr, nullptr));
    _pwm.set_duty(MCPWMDriver::CHANNEL_A, 50.0f);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, 50.0f);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, 50.0f);

    ESP_ERROR_CHECK(_spi.init());
    ESP_ERROR_CHECK(_encoder.init());
    ESP_ERROR_CHECK(_adc.init(0.0035f, 20.0f, 1.65f));

    esp_timer_handle_t focTimer;
    esp_timer_create_args_t timerArgs = {
        .callback = taskEntry,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "FOCTimer",
        .skip_unhandled_events = true, // Never use light sleep, but whatever.
    };

    esp_timer_create(&timerArgs, &focTimer);
    // TODO!
    // Currently slower than 50us, just for debug. Also need optimizing, current peaks at 800+ us.
    esp_timer_start_periodic(focTimer, 1000);
}

void FOCTask::initializeDriver() {
    ESP_ERROR_CHECK(_drv.init());

    esp_err_t err = _drv.set_3x_pwm_mode();
    if (err != ESP_OK) {
        printf("FOC: failed to set DRV8323 3x PWM mode (%s).\n", esp_err_to_name(err));
    }

    uint16_t drvReg = 0;
    for (int i = 0; i < 8; i++) {
        auto readErr = _drv.read_register(i, drvReg);
        if (readErr != ESP_OK) {
            printf("Error when reading register %i.\n", i);
        }
        printf("Register %i: %i\n", i, drvReg);
    }

    gpio_set_level(_config.cDRV8323_INL, 1);
}

void FOCTask::applyVoltageVector(float electricalAngle) {
    // Inverse Clarke for a stator voltage vector of magnitude ALIGN_VOLTAGE at
    // the given electrical angle. At 0 degrees this is (d, -d/2, -d/2), i.e.
    // alpha = d, beta = 0. At +90 degrees it is (0, +sqrt(3)/2*d, -sqrt(3)/2*d).
    const float alpha = ALIGN_VOLTAGE * std::cos(electricalAngle);
    const float beta = ALIGN_VOLTAGE * std::sin(electricalAngle);

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, 50.0f + alpha);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, 50.0f - 0.5f * alpha + GlobalVariableManager::SQRT_3_DIV_2 * beta);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, 50.0f - 0.5f * alpha - GlobalVariableManager::SQRT_3_DIV_2 * beta);
}

void FOCTask::applyAlignmentVoltage() {
    // Hold a known stator field so the rotor locks to a defined electrical
    // angle before we try to commutate against the encoder. 50 is neutral for
    // this driver (see the duty mapping at the bottom of update()).
    applyVoltageVector(0.0f);
}

float constrain(float val, float minV, float maxV) {
    if (val > maxV) { return maxV; }
    if (val < minV) { return minV; }
    return val;
}

// Fold an angle into [0, 2*pi). Unlike a bare fmod this also handles negative
// inputs, which fmod leaves negative, and NaN-safe it stays out of the way of
// sin/cos since only periodicity matters.
static inline float wrapAngle(float a) {
    a = std::fmod(a, GlobalVariableManager::TWO_PI);
    if (a < 0.0f) {
        a += GlobalVariableManager::TWO_PI;
    }
    return a;
}

// Fold an angle into (-pi, pi]. Use this for differences between two angles,
// where you need to know which direction the rotor actually moved. wrapAngle
// is the wrong tool here: it returns [0, 2*pi), so a step of -0.0004 comes
// back as 6.2828 and looks like a full reverse revolution.
static inline float wrapAngleSigned(float a) {
    a = std::fmod(a, GlobalVariableManager::TWO_PI);
    if (a > GlobalVariableManager::PI) {
        a -= GlobalVariableManager::TWO_PI;
    } else if (a < -GlobalVariableManager::PI) {
        a += GlobalVariableManager::TWO_PI;
    }
    return a;
}

void FOCTask::update() {

    int64_t t0 = esp_timer_get_time();
    // Yes, i know dt is not in seconds.
    // Dont really care, just adjust PID tune.
    float dt = static_cast<float>(t0 - _prevTime);
    _prevTime = t0;

    float angle, cumulativeAngle, velocity, acceleration;
    if (_encoder.pipeline_read_angle(angle, true) != ESP_OK) {
        return;
    }

    _stateEstimation.estimate(angle, cumulativeAngle, velocity, acceleration);
    globalVariableManager.setAngle(cumulativeAngle);
    globalVariableManager.setVelocity(velocity);
    globalVariableManager.setAcceleration(acceleration);

    uint16_t drv_fault = 0;
    uint16_t drv_vgs = 0;
    // if (_drv.read_fault_status(drv_fault) == ESP_OK) {
    //     if (_drv.has_fault(drv_fault, DRV8323::FAULT_FLT)) {
    //         // printf("DRV8323: FAULT=0x%04X\n", drv_fault);
    //     }
    // }
    // if (_drv.read_vgs_status(drv_vgs) == ESP_OK) {
    //     if (drv_vgs) {
    //         // printf("DRV8323: VGS=0x%04X\n", drv_vgs);
    //     }
    // }
    // TODO: Not sure if this will be fucky wucky since datatype is 16 bit.
    // globalVariableManager.setErrorFlags((drv_fault << 16) + drv_vgs);

    const float numPolePairs = static_cast<float>(globalVariableManager.getNumPolePairs());

    // --- Electrical offset calibration -------------------------------------
    //
    // applyAlignmentVoltage() applies a stator voltage vector at exactly 0
    // electrical degrees (duty offsets of +d, -d/2, -d/2 map to alpha = d,
    // beta = 0). When the rotor has locked onto that vector its d-axis is
    // therefore at 0 electrical degrees, so the angle we feed the Controller
    // must read 0 at that moment:
    //
    //     (angle * numPolePairs) + _elPosOffset == 0
    //
    // which gives _elPosOffset == -(numPolePairs * angleAtLock).
    //
    // This is a single constant that stays valid at every rotor position,
    // because (angle * numPolePairs) is taken modulo 2*pi further down and the
    // rotor simply walks the same way the electrical angle does.
    if (_alignPhase != AlignPhase::DONE) {
        if (_alignPhase == AlignPhase::NO_VOLTAGE) {
            if (globalVariableManager.getBusVoltage() < _config.cMinBusVoltage) {
                return;
            }

            printf("FOC: bus voltage reached %.1f V, settling for %u ms.\n",
                   _config.cMinBusVoltage,
                   (unsigned int)_config.cStartupSettleMs);
            _voltageSettleStartUs = esp_timer_get_time();
            _alignPhase = AlignPhase::VOLTAGE_SETTLE;
        }

        if (_alignPhase == AlignPhase::VOLTAGE_SETTLE) {
            if (globalVariableManager.getBusVoltage() < _config.cMinBusVoltage) {
                _alignPhase = AlignPhase::NO_VOLTAGE;
                return;
            }

            const int64_t settleUs = static_cast<int64_t>(_config.cStartupSettleMs) * 1000;
            if (esp_timer_get_time() - _voltageSettleStartUs < settleUs) {
                return;
            }

            printf("FOC: bus voltage stable, starting DRV8323.\n");
            _alignPhase = AlignPhase::INIT_DRV;
        }

        if (_alignPhase == AlignPhase::INIT_DRV) {
            initializeDriver();
            _alignPhase = AlignPhase::START_ONE_PHASE;
        }

        if (_alignPhase == AlignPhase::START_ONE_PHASE) {
            applyAlignmentVoltage();
            _alignLoops = 0;
            _alignPhase = AlignPhase::SETTLE;
            return;
        }

        _alignLoops += 1;

        if (_alignPhase == AlignPhase::SETTLE) {
            // The rotor swings to the alignment angle on startup, so early
            // readings are transient. Discard them.
            if (_alignLoops >= ALIGN_SETTLE_LOOPS) {
                _alignPhase = AlignPhase::MEASURE;
                _alignLoops = 0;
                _alignSamples = 0;
                _alignMaxStep = 0.0f;
            }
        } else if (_alignPhase == AlignPhase::MEASURE) {
            if (_alignSamples == 0) {
                _alignPrevAngle = angle;
            } else {
                // Signed wrap, so a small step stays small no matter which
                // direction the rotor moved.
                const float step = std::fabs(wrapAngleSigned(angle - _alignPrevAngle));
                if (step > _alignMaxStep) {
                    _alignMaxStep = step;
                }
            }
            _alignPrevAngle = angle;

            _alignSinSum += std::sin(angle);
            _alignCosSum += std::cos(angle);
            _alignSamples += 1;

            if (_alignLoops >= ALIGN_MEASURE_LOOPS) {
                const float lockedAngle = std::atan2(_alignSinSum, _alignCosSum);

                // Mean resultant length: 1.0 means every sample landed on the
                // exact same angle, 0.0 means they were uniformly spread around
                // the circle and carry no direction information at all. This is
                // the real "did it lock" test, and unlike max step it is
                // insensitive to encoder jitter on a stationary rotor.
                const float resultant = std::sqrt(_alignSinSum * _alignSinSum +
                                                  _alignCosSum * _alignCosSum) /
                                        static_cast<float>(_alignSamples);

                _elPosOffset = wrapAngle(-numPolePairs * lockedAngle);

                if (resultant < ALIGN_MIN_RESULTANT) {
                    printf("Align: rotor never settled (resultant %.4f, 1.0 is locked). "
                           "Offset is unreliable, raise ALIGN_VOLTAGE.\n", resultant);
                }
                if (_alignMaxStep > ALIGN_MAX_STEP_WARN) {
                    printf("Align: rotor jumped %.4f rad between samples, offset may be noisy.\n",
                           _alignMaxStep);
                }
                printf("Align: locked at %.4f rad mechanical, %u pole pairs, "
                       "elPosOffset = %.4f rad electrical, resultant %.4f, max step %.4f rad\n",
                       lockedAngle,
                       (unsigned int)numPolePairs,
                       _elPosOffset,
                       resultant,
                       _alignMaxStep);

                _alignLockedAngle = lockedAngle;

                if (_config.cPhaseOrderMode == PhaseOrderMode::AUTO) {
                    // Keep the 0-degree field applied and slowly rotate it to
                    // +90 electrical degrees. The direction the rotor follows
                    // tells us whether the motor phases and encoder count the
                    // same way. The 0-degree vector has B == C, so this does
                    // not invalidate the offset we just measured.
                    _alignLoops = 0;
                    _alignPhase = AlignPhase::DIRECTION_RAMP;
                } else {
                    _controller.setPhaseOrder(_config.cPhaseOrderMode == PhaseOrderMode::REVERSED);
                    _pwm.set_duty(MCPWMDriver::CHANNEL_A, 50.0f);
                    _pwm.set_duty(MCPWMDriver::CHANNEL_B, 50.0f);
                    _pwm.set_duty(MCPWMDriver::CHANNEL_C, 50.0f);
                    _alignPhase = AlignPhase::CURRENT;
                }
            }
        } else if (_alignPhase == AlignPhase::DIRECTION_RAMP) {
            float progress = static_cast<float>(_alignLoops) /
                             static_cast<float>(DIRECTION_RAMP_LOOPS);
            if (progress > 1.0f) {
                progress = 1.0f;
            }
            applyVoltageVector(progress * GlobalVariableManager::PI_DIV_2);

            if (progress >= 1.0f) {
                _alignLoops = 0;
                _alignPhase = AlignPhase::DIRECTION_SETTLE;
            }
        } else if (_alignPhase == AlignPhase::DIRECTION_SETTLE) {
            // The +90-degree vector is still applied; wait for the rotor to
            // finish swinging before measuring.
            if (_alignLoops >= DIRECTION_SETTLE_LOOPS) {
                _alignSinSum = 0.0f;
                _alignCosSum = 0.0f;
                _alignSamples = 0;
                _alignLoops = 0;
                _alignPhase = AlignPhase::DIRECTION_MEASURE;
            }
        } else if (_alignPhase == AlignPhase::DIRECTION_MEASURE) {
            _alignSinSum += std::sin(angle);
            _alignCosSum += std::cos(angle);
            _alignSamples += 1;

            if (_alignLoops >= DIRECTION_MEASURE_LOOPS) {
                const float angle90 = std::atan2(_alignSinSum, _alignCosSum);
                const float delta = wrapAngleSigned(angle90 - _alignLockedAngle);
                const float resultant = std::sqrt(_alignSinSum * _alignSinSum +
                                                  _alignCosSum * _alignCosSum) /
                                        static_cast<float>(_alignSamples);

                if (resultant < ALIGN_MIN_RESULTANT) {
                    printf("Direction: rotor did not settle (resultant %.4f), "
                           "direction result may be unreliable.\n", resultant);
                }

                const float deltaElectrical = delta * numPolePairs;
                if (std::fabs(deltaElectrical) < DIRECTION_MIN_ELEC_DELTA) {
                    printf("Direction: rotor barely moved (%.4f rad electrical). "
                           "Set cPhaseOrderMode manually if the motor still pulls "
                           "to a fixed angle.\n", std::fabs(deltaElectrical));
                }

                _reversePhaseSequence = (delta < 0.0f);
                _controller.setPhaseOrder(_reversePhaseSequence);

                printf("Direction: moved %.4f rad mechanical (%.4f rad electrical), "
                       "phase order %s.\n",
                       delta,
                       deltaElectrical,
                       _reversePhaseSequence ? "REVERSED" : "NORMAL");

                _pwm.set_duty(MCPWMDriver::CHANNEL_A, 50.0f);
                _pwm.set_duty(MCPWMDriver::CHANNEL_B, 50.0f);
                _pwm.set_duty(MCPWMDriver::CHANNEL_C, 50.0f);

                _alignPhase = AlignPhase::CURRENT;
            }
        } else if (_alignPhase == AlignPhase::CURRENT) {
            CURRENT_BASELINE_LOOPS -= 1;
            if (CURRENT_BASELINE_LOOPS < CURRENT_BASELINE_SETTLE) {
                int mvPhaseA, mvPhaseB, mvPhaseC;
                _adc.read_voltage(ADCOneshot::CHANNEL_A, mvPhaseA);
                _adc.read_voltage(ADCOneshot::CHANNEL_B, mvPhaseB);
                _adc.read_voltage(ADCOneshot::CHANNEL_C, mvPhaseC);

                sumPhaseOffsetA += mvPhaseA;
                sumPhaseOffsetB += mvPhaseB;
                sumPhaseOffsetC += mvPhaseC;
                numCurrentReadings++;
            }

            if (CURRENT_BASELINE_LOOPS == 0) {
                _alignPhase = AlignPhase::DONE;

                phaseOffsetA = sumPhaseOffsetA / numCurrentReadings;
                phaseOffsetB = sumPhaseOffsetB / numCurrentReadings;
                phaseOffsetC = sumPhaseOffsetC / numCurrentReadings;
                // phaseOffsetB = static_cast<int>(static_cast<double>(sumPhaseOffsetA) / static_cast<double>(numCurrentReadings));

                printf("Finished current thingy. Offsets: %i, %i, %i\n", phaseOffsetA, phaseOffsetB, phaseOffsetC);
            }

        } else {
            // Something wrong?
        }

        return;
    }

    // wrapAngle also does the fmod, and folds negatives up into [0, 2*pi).
    const float elPos = wrapAngle(angle * numPolePairs);

    float iqRef = globalVariableManager.getTorqueSetpoint();

    int mvPhaseA, mvPhaseB, mvPhaseC;
    _adc.read_voltage(ADCOneshot::CHANNEL_A, mvPhaseA);
    _adc.read_voltage(ADCOneshot::CHANNEL_B, mvPhaseB);
    _adc.read_voltage(ADCOneshot::CHANNEL_C, mvPhaseC);

    // Gain: 50V/V
    // Shunt: 1mOhm
    float currentA = lowpassCurrentA.update(static_cast<float>(mvPhaseA - phaseOffsetA) / 50.0f);
    float currentB = lowpassCurrentB.update(static_cast<float>(mvPhaseB - phaseOffsetB) / 50.0f);
    float currentC = lowpassCurrentC.update(static_cast<float>(mvPhaseC - phaseOffsetC) / 50.0f);

    float omegaE = velocity * numPolePairs;
    _out = _controller.update(iqRef, elPos + _elPosOffset, omegaE, currentA, currentB);

    float maxVal = 30.0f;
    _out.phaseA = constrain(_out.phaseA, -maxVal, maxVal);
    _out.phaseB = constrain(_out.phaseB, -maxVal, maxVal);
    _out.phaseC = constrain(_out.phaseC, -maxVal, maxVal);

    _out.phaseA += 50.0;
    _out.phaseB += 50.0;
    _out.phaseC += 50.0;

    _pwm.set_duty(MCPWMDriver::CHANNEL_A, _out.phaseA);
    _pwm.set_duty(MCPWMDriver::CHANNEL_B, _out.phaseB);
    _pwm.set_duty(MCPWMDriver::CHANNEL_C, _out.phaseC);

    globalVariableManager.setIa(currentA);
    globalVariableManager.setIb(currentB);
    globalVariableManager.setIc(currentC);

    // globalVariableManager.setIa(static_cast<float>(mvPhaseA - phaseOffsetA));
    // globalVariableManager.setIb(static_cast<float>(mvPhaseB - phaseOffsetB));
    // globalVariableManager.setIc(static_cast<float>(mvPhaseC - phaseOffsetC));

    int64_t t1 = esp_timer_get_time();
    // TODO: Add small lowpass maybe? Or rename variable
    // I know in theory the 64 bit time could overflow
    // the 32 bit, dont care, probably not going to happen.
    globalVariableManager.setAvgLoopTimeFOC(t1 - t0);
}

void FOCTask::taskEntry(void *pvParameters) {
    FOCTask *focTask = static_cast<FOCTask *>(pvParameters);
    focTask->update();
}
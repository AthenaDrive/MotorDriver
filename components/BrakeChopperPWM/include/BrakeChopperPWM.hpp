#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

// Single-channel PWM for the brake chopper. The output is active high:
// duty 0.0 keeps the chopper off, duty 1.0 turns it on continuously.
//
// MCPWM group 1 is used because group 0 belongs to the 3-phase FOC driver.
class BrakeChopperPWM {
public:
    // pin: GPIO driving the brake-chopper gate.
    // freq_hz: PWM frequency; 20 kHz keeps it above the audible range.
    // resolution_hz: MCPWM counter resolution. 40 MHz gives 2000 ticks at
    // 20 kHz, i.e. 0.05% duty resolution.
    BrakeChopperPWM(gpio_num_t pin, uint32_t freq_hz = 20000,
                    uint32_t resolution_hz = 40000000);
    ~BrakeChopperPWM();

    BrakeChopperPWM(const BrakeChopperPWM &) = delete;
    BrakeChopperPWM &operator=(const BrakeChopperPWM &) = delete;

    esp_err_t init();
    esp_err_t setDuty(float duty = 0.0f); // 0.0 = off, 1.0 = full on
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

private:
    void _cleanup();
    esp_err_t _setForceLevel(int level);

    gpio_num_t _pin;
    uint32_t _freq_hz;
    uint32_t _resolution_hz;
    uint32_t _period_ticks;

    mcpwm_timer_handle_t _timer;
    mcpwm_oper_handle_t _oper;
    mcpwm_cmpr_handle_t _cmpr;
    mcpwm_gen_handle_t _gen;

    // -1 = PWM running from the comparator, 0/1 = output forced low/high.
    int8_t _forced_level;
    bool _timer_enabled;
    bool _initialized;
};

#include "BrakeChopperPWM.hpp"

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "BrakeChopperPWM";

// MCPWM group 0 is already used by the 3-phase FOC driver.
static constexpr int BRAKE_CHOPPER_MCPWM_GROUP = 1;

BrakeChopperPWM::BrakeChopperPWM(gpio_num_t pin, uint32_t freq_hz,
                                 uint32_t resolution_hz)
    : _pin(pin)
    , _freq_hz(freq_hz)
    , _resolution_hz(resolution_hz)
    , _period_ticks(0)
    , _timer(nullptr)
    , _oper(nullptr)
    , _cmpr(nullptr)
    , _gen(nullptr)
    , _forced_level(-1)
    , _timer_enabled(false)
    , _initialized(false) {}

BrakeChopperPWM::~BrakeChopperPWM() {
    if (_initialized) {
        _cleanup();
    }
}

esp_err_t BrakeChopperPWM::init() {
    if (_initialized) {
        return ESP_OK;
    }
    if (_pin < 0) {
        ESP_LOGE(TAG, "Invalid GPIO %d", (int)_pin);
        return ESP_ERR_INVALID_ARG;
    }
    if (_freq_hz == 0 || _resolution_hz == 0) {
        ESP_LOGE(TAG, "Invalid PWM frequency or resolution");
        return ESP_ERR_INVALID_ARG;
    }

    _period_ticks = _resolution_hz / _freq_hz;
    if (_period_ticks < 2) {
        ESP_LOGE(TAG, "Resolution %lu Hz is too low for %lu Hz PWM",
                 (unsigned long)_resolution_hz, (unsigned long)_freq_hz);
        return ESP_ERR_INVALID_ARG;
    }

    // Hold the gate low before the MCPWM signal is connected, so the chopper
    // cannot switch while the peripheral is being configured.
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << _pin;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d config failed: %s", (int)_pin,
                 esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level(_pin, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d set low failed: %s", (int)_pin,
                 esp_err_to_name(err));
        return err;
    }

    mcpwm_timer_config_t timer_cfg = {
        .group_id = BRAKE_CHOPPER_MCPWM_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = _resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = _period_ticks,
        .intr_priority = 0,
        .flags = {},
    };
    err = mcpwm_new_timer(&timer_cfg, &_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create timer failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    mcpwm_operator_config_t oper_cfg = {
        .group_id = BRAKE_CHOPPER_MCPWM_GROUP,
        .intr_priority = 0,
        .flags = {},
    };
    err = mcpwm_new_operator(&oper_cfg, &_oper);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create operator failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    err = mcpwm_operator_connect_timer(_oper, _timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Connect timer failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    mcpwm_comparator_config_t cmpr_cfg = {
        .intr_priority = 0,
        .flags = {},
    };
    err = mcpwm_new_comparator(_oper, &cmpr_cfg, &_cmpr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create comparator failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = _pin,
        .flags = {},
    };
    err = mcpwm_new_generator(_oper, &gen_cfg, &_gen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create generator failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    // Up-count mode: high at the start of the period, low when the counter
    // reaches the compare value. Compare = 0 would race the start-of-period
    // event, so the exact 0%/100% endpoints use the force-level path instead.
    err = mcpwm_generator_set_action_on_timer_event(
        _gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set timer action failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    err = mcpwm_generator_set_action_on_compare_event(
        _gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr,
                                       MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set compare action failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    err = mcpwm_generator_set_force_level(_gen, 0, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Force output low failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }
    _forced_level = 0;

    err = mcpwm_timer_enable(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enable timer failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }
    _timer_enabled = true;

    err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Start timer failed: %s", esp_err_to_name(err));
        _cleanup();
        return err;
    }

    _initialized = true;
    ESP_LOGI(TAG, "Initialized on GPIO %d: %lu Hz, %lu ticks",
             (int)_pin, (unsigned long)_freq_hz,
             (unsigned long)_period_ticks);
    return ESP_OK;
}

IRAM_ATTR esp_err_t BrakeChopperPWM::setDuty(float duty) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Keep the requested endpoints exact. Without this, a compare value of 0
    // would leave a possible race with the start-of-period HIGH action.
    // The negated comparison also treats NaN as "off" instead of converting it
    // to an undefined compare value.
    if (!(duty > 0.0f)) {
        return _setForceLevel(0);
    }
    if (duty >= 1.0f) {
        return _setForceLevel(1);
    }

    uint32_t cmp = (uint32_t)(duty * (float)_period_ticks + 0.5f);
    if (cmp == 0) {
        return _setForceLevel(0);
    }
    if (cmp >= _period_ticks) {
        return _setForceLevel(1);
    }

    esp_err_t err = mcpwm_comparator_set_compare_value(_cmpr, cmp);
    if (err != ESP_OK) {
        return err;
    }

    if (_forced_level >= 0) {
        // init() and _setForceLevel() use hold_on=true (continuous force).
        // Removing it must also use hold_on=true; using false here only
        // clears the non-continuous action and leaves the output forced low.
        err = mcpwm_generator_set_force_level(_gen, -1, true);
        if (err == ESP_OK) {
            _forced_level = -1;
        }
    }
    return err;
}

esp_err_t BrakeChopperPWM::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    _cleanup();
    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

esp_err_t BrakeChopperPWM::_setForceLevel(int level) {
    if (_forced_level == level) {
        return ESP_OK;
    }

    esp_err_t err = mcpwm_generator_set_force_level(_gen, level, true);
    if (err == ESP_OK) {
        _forced_level = level;
    }
    return err;
}

void BrakeChopperPWM::_cleanup() {
    // Force the gate off before tearing the timer down.
    if (_gen) {
        mcpwm_generator_set_force_level(_gen, 0, true);
    }
    if (_timer && _timer_enabled) {
        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(_timer);
        _timer_enabled = false;
    }

    if (_gen) {
        mcpwm_del_generator(_gen);
        _gen = nullptr;
    }
    if (_cmpr) {
        mcpwm_del_comparator(_cmpr);
        _cmpr = nullptr;
    }
    if (_oper) {
        mcpwm_del_operator(_oper);
        _oper = nullptr;
    }
    if (_timer) {
        mcpwm_del_timer(_timer);
        _timer = nullptr;
    }

    _forced_level = -1;
    _initialized = false;

    // Leave the pin actively low instead of floating when the driver is gone.
    if (_pin >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << _pin;
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        if (gpio_config(&io) == ESP_OK) {
            gpio_set_level(_pin, 0);
        }
    }
}

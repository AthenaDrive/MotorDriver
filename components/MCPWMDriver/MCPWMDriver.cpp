#include "MCPWMDriver.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "MCPWMDriver";

static const gpio_num_t CHANNEL_GPIOS[3] = {
    GPIO_NUM_42,
    GPIO_NUM_41,
    GPIO_NUM_40,
};

MCPWMDriver::MCPWMDriver()
    : _timer(nullptr)
    , _period_ticks(0)
    , _peak_ticks(0)
    , _initialized(false) {
    _channels[0] = {};
    _channels[1] = {};
    _channels[2] = {};
}

MCPWMDriver::~MCPWMDriver() {
    if (_initialized) {
        deinit();
    }
}

esp_err_t MCPWMDriver::init(uint32_t freq_hz, uint32_t resolution_hz,
                            mcpwm_timer_event_cb_t peak_cb, void *peak_cb_arg) {
    if (_initialized) {
        return ESP_OK;
    }

    _period_ticks = resolution_hz / freq_hz;
    _peak_ticks = _period_ticks / 2;

    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = _period_ticks,
        .intr_priority = 0,
        .flags = {},
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &_timer), TAG, "create timer failed");

    if (peak_cb) {
        mcpwm_timer_event_callbacks_t cbs = {
            .on_full = peak_cb,
            .on_empty = nullptr,
            .on_stop = nullptr,
        };
        ESP_RETURN_ON_ERROR(mcpwm_timer_register_event_callbacks(_timer, &cbs, peak_cb_arg),
                            TAG, "register peak callback failed");
    }

    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t oper_cfg = {
            .group_id = 0,
            .intr_priority = 0,
            .flags = {},
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &_channels[i].oper), TAG, "create operator %d failed", i);
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(_channels[i].oper, _timer), TAG, "connect operator %d failed", i);

        mcpwm_comparator_config_t cmpr_cfg = {
            .intr_priority = 0,
            .flags = {},
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(_channels[i].oper, &cmpr_cfg, &_channels[i].cmpr), TAG, "create comparator %d failed", i);

        mcpwm_generator_config_t gen_cfg = {
            .gen_gpio_num = CHANNEL_GPIOS[i],
            .flags = {},
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(_channels[i].oper, &gen_cfg, &_channels[i].gen), TAG, "create generator %d failed", i);

        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(_channels[i].cmpr, 0), TAG, "set compare 0 on ch %d failed", i);

        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(
            _channels[i].gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
            TAG, "set timer event action on ch %d failed", i);

        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(
            _channels[i].gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _channels[i].cmpr, MCPWM_GEN_ACTION_LOW)),
            TAG, "set compare up action on ch %d failed", i);

        ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(
            _channels[i].gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, _channels[i].cmpr, MCPWM_GEN_ACTION_HIGH)),
            TAG, "set compare down action on ch %d failed", i);
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(_timer), TAG, "enable timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP), TAG, "start timer failed");

    _initialized = true;
    ESP_LOGI(TAG, "MCPWM initialized: %lu Hz centre-aligned, %lu ticks period", freq_hz, _period_ticks);
    return ESP_OK;
}

esp_err_t MCPWMDriver::set_duty(Channel ch, float percent) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    uint32_t cmp = (uint32_t)(_peak_ticks * percent / 100.0f);
    return mcpwm_comparator_set_compare_value(_channels[ch].cmpr, cmp);
}

esp_err_t MCPWMDriver::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(_timer);

    for (int i = 0; i < 3; i++) {
        if (_channels[i].gen) {
            mcpwm_del_generator(_channels[i].gen);
            _channels[i].gen = nullptr;
        }
        if (_channels[i].cmpr) {
            mcpwm_del_comparator(_channels[i].cmpr);
            _channels[i].cmpr = nullptr;
        }
        if (_channels[i].oper) {
            mcpwm_del_operator(_channels[i].oper);
            _channels[i].oper = nullptr;
        }
    }

    mcpwm_del_timer(_timer);
    _timer = nullptr;
    _initialized = false;

    ESP_LOGI(TAG, "MCPWM deinitialized");
    return ESP_OK;
}

#pragma once

#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

class MCPWMDriver {
public:
    enum Channel : uint8_t {
        CHANNEL_A = 0,
        CHANNEL_B = 1,
        CHANNEL_C = 2,
    };

    MCPWMDriver();
    ~MCPWMDriver();

    MCPWMDriver(const MCPWMDriver &) = delete;
    MCPWMDriver &operator=(const MCPWMDriver &) = delete;

    esp_err_t init(uint32_t freq_hz = 30000, uint32_t resolution_hz = 40000000,
                   mcpwm_timer_event_cb_t peak_cb = nullptr, void *peak_cb_arg = nullptr);
    esp_err_t set_duty(Channel ch, float percent);
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

private:
    struct ChannelCtx {
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t cmpr;
        mcpwm_gen_handle_t gen;
    };

    mcpwm_timer_handle_t _timer;
    ChannelCtx _channels[3];
    uint32_t _period_ticks;
    uint32_t _peak_ticks;
    bool _initialized;
};

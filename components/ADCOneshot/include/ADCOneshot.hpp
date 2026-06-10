#pragma once

#include <cstdint>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

class ADCOneshot {
public:
    enum Channel : uint8_t {
        CHANNEL_A = 0,
        CHANNEL_B = 1,
        CHANNEL_C = 2,
    };

    ADCOneshot();
    ~ADCOneshot();

    ADCOneshot(const ADCOneshot &) = delete;
    ADCOneshot &operator=(const ADCOneshot &) = delete;

    esp_err_t init(float shunt_ohm = 0, float csa_gain = 0, float vref = 0);
    esp_err_t read_raw(Channel ch, int &raw);
    esp_err_t read_voltage(Channel ch, float &voltage_mv);
    esp_err_t read_current_amps(Channel ch, float &current);
    esp_err_t calibrate_raw(int raw, float &voltage_mv) const;
    esp_err_t raw_to_current(int raw, float &current) const;
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

private:
    adc_oneshot_unit_handle_t _adc_handle;
    adc_cali_handle_t _cali_handle;
    adc_channel_t _channels[3];
    bool _cali_enabled;
    bool _initialized;
    float _shunt_ohm;
    float _csa_gain;
    float _vref;
};

#include "ADCOneshot.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "ADCOneshot";

static const gpio_num_t CHANNEL_GPIOS[3] = {
    GPIO_NUM_6,
    GPIO_NUM_5,
    GPIO_NUM_4,
};

ADCOneshot::ADCOneshot()
    : _adc_handle(nullptr)
    , _cali_handle(nullptr)
    , _cali_enabled(false)
    , _initialized(false)
    , _shunt_ohm(0)
    , _csa_gain(0)
    , _vref(0) {
    _channels[0] = ADC_CHANNEL_5;
    _channels[1] = ADC_CHANNEL_4;
    _channels[2] = ADC_CHANNEL_3;
}

ADCOneshot::~ADCOneshot() {
    if (_initialized) {
        deinit();
    }
}

esp_err_t ADCOneshot::init(float shunt_ohm, float csa_gain, float vref) {
    if (_initialized) {
        return ESP_OK;
    }

    _shunt_ohm = shunt_ohm;
    _csa_gain = csa_gain;
    _vref = vref;

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &_adc_handle), TAG, "create ADC unit failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(_adc_handle, _channels[i], &chan_cfg),
                            TAG, "config channel %d failed", i);
    }

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_5,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &_cali_handle);
    if (cali_ret == ESP_OK) {
        _cali_enabled = true;
    } else {
        ESP_LOGW(TAG, "calibration not supported, raw values only");
        _cali_enabled = false;
    }

    _initialized = true;
    ESP_LOGI(TAG, "ADC oneshot initialized");
    return ESP_OK;
}

esp_err_t ADCOneshot::read_raw(Channel ch, int &raw) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return adc_oneshot_read(_adc_handle, _channels[ch], &raw);
}

esp_err_t ADCOneshot::read_voltage(Channel ch, float &voltage_mv) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    int raw;
    esp_err_t ret = adc_oneshot_read(_adc_handle, _channels[ch], &raw);
    if (ret != ESP_OK) {
        return ret;
    }
    if (_cali_enabled) {
        int cali_result;
        ret = adc_cali_raw_to_voltage(_cali_handle, raw, &cali_result);
        if (ret == ESP_OK) {
            voltage_mv = (float)cali_result;
        }
        return ret;
    }
    voltage_mv = (float)raw;
    return ESP_OK;
}

esp_err_t ADCOneshot::read_current_amps(Channel ch, float &current) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_shunt_ohm == 0 || _csa_gain == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    float voltage_mv;
    esp_err_t ret = read_voltage(ch, voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    current = ((voltage_mv / 1000.0f) - _vref) / (_shunt_ohm * _csa_gain);
    return ESP_OK;
}

esp_err_t ADCOneshot::calibrate_raw(int raw, float &voltage_mv) const {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_cali_enabled) {
        int cali_result;
        esp_err_t ret = adc_cali_raw_to_voltage(_cali_handle, raw, &cali_result);
        if (ret == ESP_OK) {
            voltage_mv = (float)cali_result;
        }
        return ret;
    }
    voltage_mv = (float)raw;
    return ESP_OK;
}

esp_err_t ADCOneshot::raw_to_current(int raw, float &current) const {
    if (!_initialized || _shunt_ohm == 0 || _csa_gain == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    float voltage_mv;
    esp_err_t ret = calibrate_raw(raw, voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    current = ((voltage_mv / 1000.0f) - _vref) / (_shunt_ohm * _csa_gain);
    return ESP_OK;
}

esp_err_t ADCOneshot::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (_cali_enabled) {
        adc_cali_delete_scheme_curve_fitting(_cali_handle);
        _cali_handle = nullptr;
        _cali_enabled = false;
    }

    adc_oneshot_del_unit(_adc_handle);
    _adc_handle = nullptr;
    _initialized = false;

    ESP_LOGI(TAG, "ADC oneshot deinitialized");
    return ESP_OK;
}

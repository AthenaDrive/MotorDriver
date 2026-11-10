#include "ADCContinuous.hpp"

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "ADCContinuous";

// parse() casts the DMA bytes straight to adc_digi_output_data_t.
static_assert(sizeof(adc_digi_output_data_t) == SOC_ADC_DIGI_RESULT_BYTES,
              "unexpected ADC continuous result size");

// Board v3 phase-current pins: A = GPIO6, B = GPIO5, C = GPIO4, all ADC1.
static const adc_channel_t CHANNEL_ADC[ADCContinuous::CHANNEL_COUNT] = {
    ADC_CHANNEL_5,
    ADC_CHANNEL_4,
    ADC_CHANNEL_3,
};

static constexpr uint8_t ALL_CHANNELS_MASK =
    (1u << ADCContinuous::CHANNEL_COUNT) - 1u;

// One frame holds 12 results, i.e. four complete A/B/C scans. At 60 kHz that
// is one DMA interrupt every 200 us, so the newest sample the FOC loop can
// pick up is at most one frame old. The ring buffer holds about 4 ms of
// results; the FOC loop normally drains it every 100 us.
static constexpr uint32_t CONV_FRAME_BYTES = 48;
static constexpr uint32_t RING_BUFFER_BYTES = 1024;

// The FOC loop only needs the newest value of each channel, but a single
// adc_continuous_read() is limited to the buffer size. 512 bytes covers many
// 100 us FOC periods (about 24 bytes each at 60 kHz), so the normal path is
// one call per iteration.
static constexpr uint32_t READ_CHUNK_BYTES = 512;

ADCContinuous::ADCContinuous()
    : _adc_handle(nullptr)
    , _cali_handle(nullptr)
    , _cali_enabled(false)
    , _initialized(false)
    , _shunt_ohm(0)
    , _csa_gain(0)
    , _vref(0)
    , _sample_freq_hz(0)
    , _seen_mask(0) {
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        _latest_raw[i] = 0;
    }
    for (uint32_t i = 0; i < CALI_LUT_SIZE; i++) {
        _mv_lut[i] = 0;
    }
}

ADCContinuous::~ADCContinuous() {
    if (_initialized) {
        deinit();
    }
}

esp_err_t ADCContinuous::init(float shunt_ohm, float csa_gain, float vref,
                              uint32_t sample_freq_hz) {
    if (_initialized) {
        return ESP_OK;
    }

    _shunt_ohm = shunt_ohm;
    _csa_gain = csa_gain;
    _vref = vref;
    _sample_freq_hz = sample_freq_hz;

    adc_continuous_handle_cfg_t adc_cfg = {
        .max_store_buf_size = RING_BUFFER_BYTES,
        .conv_frame_size = CONV_FRAME_BYTES,
        .flags = {
            // The FOC loop drains the pool every iteration. If it is ever
            // delayed long enough to fill the pool, keep the newest results
            // instead of an old backlog.
            .flush_pool = 1,
        },
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&adc_cfg, &_adc_handle),
                        TAG, "create ADC continuous handle failed");

    adc_digi_pattern_config_t pattern[CHANNEL_COUNT] = {};
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        pattern[i].atten = ADC_ATTEN_DB_12;
        pattern[i].channel = CHANNEL_ADC[i];
        pattern[i].unit = ADC_UNIT_1;
        pattern[i].bit_width = ADC_BITWIDTH_12;
    }

    // The output format member is deprecated and ignored on the ESP32-S3 (the
    // driver always selects TYPE2), but it must still be initialized.
    adc_continuous_config_t dig_cfg = {
        .pattern_num = CHANNEL_COUNT,
        .adc_pattern = pattern,
        .sample_freq_hz = _sample_freq_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    esp_err_t ret = adc_continuous_config(_adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "config ADC continuous failed (%s)", esp_err_to_name(ret));
        adc_continuous_deinit(_adc_handle);
        _adc_handle = nullptr;
        return ret;
    }

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = CHANNEL_ADC[CHANNEL_A],
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &_cali_handle);
    if (cali_ret == ESP_OK) {
        _cali_enabled = true;

        // Build the raw -> mV lookup table once. The curve-fitting conversion
        // is far too expensive to run in the FOC loop.
        for (uint32_t i = 0; i < CALI_LUT_SIZE; i++) {
            int raw = static_cast<int>(i * CALI_LUT_STEP);
            if (raw > 4095) {
                raw = 4095;
            }

            int mv = 0;
            esp_err_t lut_ret = adc_cali_raw_to_voltage(_cali_handle, raw, &mv);
            if (lut_ret != ESP_OK) {
                ESP_LOGW(TAG, "calibration LUT build failed at raw %d (%s), raw values only",
                         raw, esp_err_to_name(lut_ret));
                adc_cali_delete_scheme_curve_fitting(_cali_handle);
                _cali_handle = nullptr;
                _cali_enabled = false;
                break;
            }
            _mv_lut[i] = static_cast<int16_t>(mv);
        }
    } else {
        ESP_LOGW(TAG, "calibration not supported, raw values only");
        _cali_enabled = false;
    }

    ret = adc_continuous_start(_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start ADC continuous failed (%s)", esp_err_to_name(ret));
        if (_cali_enabled) {
            adc_cali_delete_scheme_curve_fitting(_cali_handle);
            _cali_handle = nullptr;
            _cali_enabled = false;
        }
        adc_continuous_deinit(_adc_handle);
        _adc_handle = nullptr;
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG, "ADC continuous initialized: %lu Hz total, %lu Hz per channel "
                  "(DMA ISR allocated on core %d)",
             static_cast<unsigned long>(_sample_freq_hz),
             static_cast<unsigned long>(_sample_freq_hz / CHANNEL_COUNT),
             esp_cpu_get_core_id());
    return ESP_OK;
}

// Drain everything the DMA engine has produced so far. Only the last result
// for each channel is kept; older results are not useful to the control loop.
IRAM_ATTR esp_err_t ADCContinuous::refresh() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // The results are 32-bit words, so keep the copy buffer aligned before
    // casting slices to adc_digi_output_data_t.
    alignas(4) uint8_t buf[READ_CHUNK_BYTES];
    for (;;) {
        uint32_t length = 0;
        esp_err_t ret = adc_continuous_read(_adc_handle, buf, sizeof(buf), &length, 0);
        if (ret == ESP_ERR_TIMEOUT) {
            // Nothing new since the last call. The cached values stay valid.
            return ESP_OK;
        }
        if (ret != ESP_OK) {
            return ret;
        }

        parse_latest(buf, length);

        // A full buffer may leave more results queued. A short read means the
        // pool is empty for now.
        if (length < sizeof(buf)) {
            return ESP_OK;
        }
    }
}

IRAM_ATTR void ADCContinuous::parse_latest(const uint8_t *data, uint32_t size) {
    // ESP32-S3 continuous results use the 4-byte type2 layout: 12-bit raw
    // value in bits 0..11, channel in bits 13..16, unit in bit 17.
    //
    // The newest result is at the end of the buffer. Walk backwards and stop
    // as soon as every channel has been seen, so a normal iteration only
    // touches the last three or four results instead of the whole DMA drain.
    const uint32_t aligned_size = size - (size % SOC_ADC_DIGI_RESULT_BYTES);
    if (aligned_size == 0) {
        return;
    }

    uint16_t latest[CHANNEL_COUNT] = {};
    uint8_t found = 0;

    for (int32_t i = static_cast<int32_t>(aligned_size) - SOC_ADC_DIGI_RESULT_BYTES;
         i >= 0 && found != ALL_CHANNELS_MASK;
         i -= SOC_ADC_DIGI_RESULT_BYTES) {
        const adc_digi_output_data_t *result =
            reinterpret_cast<const adc_digi_output_data_t *>(data + i);

        if (result->type2.unit != 0) {
            continue; // ADC2 result, not part of this pattern
        }

        for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
            const uint8_t bit = static_cast<uint8_t>(1u << ch);
            if ((found & bit) == 0 && result->type2.channel == CHANNEL_ADC[ch]) {
                latest[ch] = result->type2.data;
                found |= bit;
                break;
            }
        }
    }

    for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
        const uint8_t bit = static_cast<uint8_t>(1u << ch);
        if (found & bit) {
            _latest_raw[ch] = latest[ch];
            _seen_mask |= bit;
        }
    }
}

esp_err_t ADCContinuous::read_raw(Channel ch, int &raw) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ch >= CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = refresh();
    if (ret != ESP_OK) {
        return ret;
    }
    if ((_seen_mask & (1u << ch)) == 0) {
        return ESP_ERR_INVALID_STATE; // no sample for this channel yet
    }

    raw = _latest_raw[ch];
    return ESP_OK;
}

IRAM_ATTR esp_err_t ADCContinuous::raw_to_voltage(int raw, int &voltage_mv) const {
    if (_cali_enabled) {
        // Clamp to the range covered by the lookup table (12-bit raw).
        if (raw < 0) {
            raw = 0;
        } else if (raw > 4095) {
            raw = 4095;
        }

        const uint32_t index = static_cast<uint32_t>(raw) / CALI_LUT_STEP;
        const int32_t frac = static_cast<int32_t>(raw % CALI_LUT_STEP);
        const int32_t lo = _mv_lut[index];
        const int32_t hi = _mv_lut[index + 1];
        voltage_mv = lo + (((hi - lo) * frac) / static_cast<int32_t>(CALI_LUT_STEP));
        return ESP_OK;
    }
    voltage_mv = raw;
    return ESP_OK;
}

esp_err_t ADCContinuous::read_voltage(Channel ch, int &voltage_mv) {
    int raw;
    esp_err_t ret = read_raw(ch, raw);
    if (ret != ESP_OK) {
        return ret;
    }
    return raw_to_voltage(raw, voltage_mv);
}

IRAM_ATTR esp_err_t ADCContinuous::read_voltage_all(int &voltage_a_mv, int &voltage_b_mv, int &voltage_c_mv) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = refresh();
    if (ret != ESP_OK) {
        return ret;
    }
    if (_seen_mask != ALL_CHANNELS_MASK) {
        return ESP_ERR_INVALID_STATE; // not every channel has produced a sample yet
    }

    ret = raw_to_voltage(_latest_raw[CHANNEL_A], voltage_a_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = raw_to_voltage(_latest_raw[CHANNEL_B], voltage_b_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    return raw_to_voltage(_latest_raw[CHANNEL_C], voltage_c_mv);
}

esp_err_t ADCContinuous::read_current_amps(Channel ch, float &current) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (_shunt_ohm == 0 || _csa_gain == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int voltage_mv;
    esp_err_t ret = read_voltage(ch, voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    current = (((float)voltage_mv / 1000.0f) - _vref) / (_shunt_ohm * _csa_gain);
    return ESP_OK;
}

esp_err_t ADCContinuous::calibrate_raw(int raw, float &voltage_mv) const {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int mv;
    esp_err_t ret = raw_to_voltage(raw, mv);
    if (ret == ESP_OK) {
        voltage_mv = static_cast<float>(mv);
    }
    return ret;
}

esp_err_t ADCContinuous::raw_to_current(int raw, float &current) const {
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

esp_err_t ADCContinuous::deinit() {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = adc_continuous_stop(_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "stop ADC continuous failed (%s)", esp_err_to_name(ret));
    }

    if (_cali_enabled) {
        adc_cali_delete_scheme_curve_fitting(_cali_handle);
        _cali_handle = nullptr;
        _cali_enabled = false;
    }

    ret = adc_continuous_deinit(_adc_handle);
    _adc_handle = nullptr;
    _initialized = false;

    ESP_LOGI(TAG, "ADC continuous deinitialized");
    return ret;
}

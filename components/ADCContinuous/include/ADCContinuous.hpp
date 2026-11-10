#pragma once

#include <cstdint>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

// Free-running ADC1 sampling for the three phase-current amplifiers.
//
// The oneshot driver converts one channel per call and blocks until the result
// is ready, which costs roughly 50 us per channel in the FOC loop. Continuous
// mode lets the ADC DMA engine sample the three channels in the background.
// The FOC loop only drains the results that arrived since the previous
// iteration and keeps the newest raw value for each channel.
class ADCContinuous {
public:
    enum Channel : uint8_t {
        CHANNEL_A = 0,
        CHANNEL_B = 1,
        CHANNEL_C = 2,
        CHANNEL_COUNT = 3,
    };

    // Total conversion rate for the whole pattern. With three channels each
    // channel is sampled at sample_freq_hz / CHANNEL_COUNT. 60 kHz total gives
    // 20 kHz per phase (one sample every 50 us), faster than the 100 us FOC
    // loop and still below the ESP32-S3 12-bit continuous-mode limit (83333 Hz).
    static constexpr uint32_t DEFAULT_SAMPLE_FREQ_HZ = 60000;

    // Calibration lookup table. The IDF curve-fitting conversion is a 5-term
    // 64-bit polynomial with runtime 64-bit divisions, which costs far more
    // than the ADC read itself when called three times per FOC iteration.
    // Precompute it on a coarse grid and interpolate instead. A 64-count step
    // is about 70 mV; the remaining interpolation error is a few mV (well
    // under 0.1 A at 50 mV/A), and the startup zero-current offset removes the
    // constant part anyway.
    static constexpr uint32_t CALI_LUT_STEP = 64;
    static constexpr uint32_t CALI_LUT_SIZE = (4096 / CALI_LUT_STEP) + 1;

    ADCContinuous();
    ~ADCContinuous();

    ADCContinuous(const ADCContinuous &) = delete;
    ADCContinuous &operator=(const ADCContinuous &) = delete;

    esp_err_t init(float shunt_ohm = 0, float csa_gain = 0, float vref = 0,
                   uint32_t sample_freq_hz = DEFAULT_SAMPLE_FREQ_HZ);
    esp_err_t read_raw(Channel ch, int &raw);
    esp_err_t read_voltage(Channel ch, int &voltage_mv);

    // Refresh the DMA results once and convert all three channels from that
    // same batch. Use this in the FOC loop instead of three read_voltage()
    // calls, so the pattern is drained only once per iteration.
    esp_err_t read_voltage_all(int &voltage_a_mv, int &voltage_b_mv, int &voltage_c_mv);

    esp_err_t read_current_amps(Channel ch, float &current);
    esp_err_t calibrate_raw(int raw, float &voltage_mv) const;
    esp_err_t raw_to_current(int raw, float &current) const;
    esp_err_t deinit();
    bool is_initialized() const { return _initialized; }

private:
    // These run in the FOC hot path. Keep them in IRAM (attribute applied on
    // the definitions in the .cpp) so they do not fight core 0
    // (Ethernet/lwIP) for the shared flash instruction cache.
    esp_err_t refresh();
    esp_err_t raw_to_voltage(int raw, int &voltage_mv) const;
    void parse_latest(const uint8_t *data, uint32_t size);

    adc_continuous_handle_t _adc_handle;
    adc_cali_handle_t _cali_handle;
    bool _cali_enabled;
    bool _initialized;
    float _shunt_ohm;
    float _csa_gain;
    float _vref;
    uint32_t _sample_freq_hz;
    uint16_t _latest_raw[CHANNEL_COUNT];
    int16_t _mv_lut[CALI_LUT_SIZE];
    uint8_t _seen_mask;
};

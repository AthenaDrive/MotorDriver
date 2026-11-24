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
//
// The DMA completion callback pushes each finished frame into the per-channel
// history on core 0; the FOC loop only takes a snapshot. Returning the newest
// sample is wrong here: the phase current carries a
// switching ripple from the 30 kHz carrier (several amps peak-to-peak with a
// low-inductance motor) and the ADC runs asynchronously, so one sample is an
// arbitrary point on that ripple. The driver therefore keeps the last
// AVERAGE_SAMPLES results of each channel and returns their mean. The sample
// rate is picked so consecutive samples of one channel are not an integer
// number of carrier periods apart (see DEFAULT_SAMPLE_FREQ_HZ); that spreads
// the averaged points across the ripple and cancels it. This is not the same
// as oversampling: the ESP32-S3 continuous mode cannot run fast enough for
// that, and this gets the same result with fewer samples by using the phase.
class ADCContinuous {
public:
    enum Channel : uint8_t {
        CHANNEL_A = 0,
        CHANNEL_B = 1,
        CHANNEL_C = 2,
        CHANNEL_COUNT = 3,
    };

    // Total conversion rate for the whole pattern. With three channels each
    // channel is sampled at sample_freq_hz / CHANNEL_COUNT.
    //
    // 71 kHz is deliberate, do not "round" it to 72 kHz. The ESP32-S3 ADC
    // digital controller triggers from a 2.5 MHz APB-derived clock and uses an
    // integer trigger interval, so the achievable rates are 2.5 MHz / n.
    // Requesting 71 kHz selects n = 35, i.e. 71.43 kHz total or 23.81 kHz per
    // phase. The MCPWM carrier is 40 MHz / 1333 = 30.0075 kHz, so consecutive
    // samples of one phase are 1.2603 carrier periods apart. Four of them land
    // at 0, 0.26, 0.52 and 0.78 of a carrier period, which averages the ripple
    // from about 2.5 A down to about 0.24 A worst case (24 V bus, 16 uH motor,
    // 30 % modulation). n = 34 (a 72 kHz request) leaves about 0.39 A, and
    // n = 41 (the old 60 kHz request) leaves about 1.6 A because two of the
    // four points bunch up.
    static constexpr uint32_t DEFAULT_SAMPLE_FREQ_HZ = 71000;

    // Number of consecutive per-channel samples averaged into one reading.
    // Four samples span 126 us, which is about 9 degrees of extra phase lag at
    // the 400 Hz current-loop crossover. Keep this a power of two; the ring
    // wraps with a mask.
    static constexpr uint32_t AVERAGE_SAMPLES = 4;

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
    // calls. The continuous-mode callback keeps the per-channel averages
    // fresh, so this only takes a consistent snapshot and applies the
    // calibration table; it does not touch the ADC DMA ring.
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
    static bool _onConvDone(adc_continuous_handle_t handle,
                            const adc_continuous_evt_data_t *edata,
                            void *user_data);
    esp_err_t raw_to_voltage(int raw, int &voltage_mv) const;
    void accumulate(const uint8_t *data, uint32_t size);
    void push_sample(uint8_t ch, uint16_t raw);
    int averaged_raw(Channel ch) const;

    adc_continuous_handle_t _adc_handle;
    adc_cali_handle_t _cali_handle;
    bool _cali_enabled;
    bool _initialized;
    float _shunt_ohm;
    float _csa_gain;
    float _vref;
    uint32_t _sample_freq_hz;
    uint16_t _latest_raw[CHANNEL_COUNT];
    uint16_t _avg_ring[CHANNEL_COUNT][AVERAGE_SAMPLES];
    uint32_t _avg_sum[CHANNEL_COUNT];
    uint8_t _avg_pos[CHANNEL_COUNT];
    uint8_t _avg_count[CHANNEL_COUNT];
    int16_t _mv_lut[CALI_LUT_SIZE];
    uint8_t _seen_mask;
    // Seqlock written by the ADC completion callback on core 0 and read by
    // the FOC task on core 1. The callback is the only writer, so a plain
    // read-modify-write is enough; the reader retries if the sequence changed
    // while the snapshot was being taken.
    volatile uint32_t _seq;
};

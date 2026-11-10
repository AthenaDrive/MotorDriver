#include <stdio.h>
#include "LowpassFilter.hpp"
#include "esp_attr.h"

LowpassFilter::LowpassFilter(float alpha) {
    _alpha = alpha;
}

IRAM_ATTR float LowpassFilter::update(float value) {
    _filtered = _filtered * (1.0f - _alpha) + value * _alpha;
    return _filtered;
}

float LowpassFilter::getValue() {
    return _filtered;
}
#include "fire.h"
#include <math.h>


// ---------------------------------------------------------------------------
// Phototransistor::readSensor
// ---------------------------------------------------------------------------
float Phototransistor::readSensor() {
    float v = _adc->read_voltage(_read_pin);
    if (!_initialised) {
        _filtered_v = v;
        _initialised = true;
    } else {
        _filtered_v = _alpha * v + (1.0f - _alpha) * _filtered_v;
    }
    mapping_reading_ = _filtered_v;
    return _filtered_v;
}


// ---------------------------------------------------------------------------
// Fan
// ---------------------------------------------------------------------------
void Fan::begin() {
    if (_hw) _hw->begin();
    _on = false;
}

void Fan::on() {
    if (_on) return;
    if (_hw) _hw->on();
    _on = true;
    if (_clock) _on_started_ms = _clock->now_ms();
}

void Fan::off() {
    if (_hw) _hw->off();
    _on = false;
}

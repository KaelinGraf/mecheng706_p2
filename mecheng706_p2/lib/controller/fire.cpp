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
// FireBank
// ---------------------------------------------------------------------------
void FireBank::update() {
    front->readSensor();
    right->readSensor();
    rear->readSensor();
    left->readSensor();
}

float FireBank::maxV() const {
    float v = front->getFilteredV();
    if (right->getFilteredV() > v) v = right->getFilteredV();
    if (rear->getFilteredV() > v) v = rear->getFilteredV();
    if (left->getFilteredV() > v) v = left->getFilteredV();
    return v;
}

bool FireBank::allBelow(float threshold) const {
    return front->getFilteredV() < threshold &&
           right->getFilteredV() < threshold &&
           rear->getFilteredV() < threshold &&
           left->getFilteredV() < threshold;
}

float FireBank::estimateBearing(bool* valid, float threshold) const {
    float vf = front->getFilteredV();
    float vl = left->getFilteredV();
    float vb = rear->getFilteredV();
    float vr = right->getFilteredV();

    float ambient = vf;
    if (vl < ambient) ambient = vl;
    if (vb < ambient) ambient = vb;
    if (vr < ambient) ambient = vr;

    float wf = vf - ambient;
    float wl = vl - ambient;
    float wb = vb - ambient;
    float wr = vr - ambient;

    float x = wr - wl;
    float y = wf - wb;

    if (valid) *valid = (maxV() >= threshold) && (fabsf(x) + fabsf(y) > 1e-3f);
    if (fabsf(x) < 1e-6f && fabsf(y) < 1e-6f) return 0.0f;

    return atan2f(-x, y);
}

void FireBank::reset() {
    front->reset();
    right->reset();
    rear->reset();
    left->reset();
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

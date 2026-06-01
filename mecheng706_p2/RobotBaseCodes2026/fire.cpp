#include "Arduino.h"
#include "fire.h"

// ---------------------------------------------------------------------------
// Phototransistor::readSensor
// ---------------------------------------------------------------------------
// One-pin ADC read with an exponential moving average filter. The filter is
// initialised on the very first read to avoid a slow ramp from zero whenever
// the state machine resets a cell.
float Phototransistor::readSensor() {
    float v = readVoltage(_read_pin);
    if (!_initialised) {
        _filtered_v  = v;       // seed the EWMA from the first sample
        _initialised = true;
    } else {
        _filtered_v  = _alpha * v + (1.0f - _alpha) * _filtered_v;
    }
    mapping_reading_ = _filtered_v;
    return _filtered_v;
}


// ---------------------------------------------------------------------------
// FireBank::update
// ---------------------------------------------------------------------------
// Refresh all four cell readings. Cheap (4 ADC reads), so safe to call every
// loop iteration; this keeps the EWMAs converging at the loop frequency.
void FireBank::update() {
    _sl->readSensor();
    _l->readSensor();
    _r->readSensor();
    _sr->readSensor();
}

float FireBank::maxV() const {
    float v = _sl->getFilteredV();
    if (_l->getFilteredV() > v) v = _l->getFilteredV();
    if (_r->getFilteredV() > v) v = _r ->getFilteredV();
    if (_sr ->getFilteredV() > v) v = _sr ->getFilteredV();
    return v;
}
float FireBank::maxVMid() const {
    float v = _l->getFilteredV();
    //if (_l->getFilteredV() > v) v = _l->getFilteredV();
    if (_r->getFilteredV() > v) v = _r ->getFilteredV();
    //if (_sr ->getFilteredV() > v) v = _sr ->getFilteredV();
    return v;
}

bool FireBank::allBelow(float threshold) const {
    return _sl->getFilteredV() < threshold &&
           _l->getFilteredV() < threshold &&
           _r->getFilteredV() < threshold &&
           _sr ->getFilteredV() < threshold;
}

void FireBank::printFireSensors() {
    Serial.print(_sl->getFilteredV());
    Serial.print(" ");
    Serial.print(_l->getFilteredV());
    Serial.print(" ");
    Serial.print(_r->getFilteredV());
    Serial.print(" ");
    Serial.print(_sr->getFilteredV());
    Serial.println(" ");
}


// ---------------------------------------------------------------------------
// FireBank::estimateBearing
// ---------------------------------------------------------------------------
// Outer-pair bearing. Replaces the old four-cell sin/cos vector sum, which no
// longer matches the hardware now that the outer pair (_sl/_sr) are mounted
// flat and forward-facing. Returns the fire's angle in DEGREES relative to the
// turret's current optical axis: 0 = aimed, sign = side (+ = fire toward _sl /
// outer-left -- VERIFY against the servo direction on the bench).
//
// The gain-balanced, normalised differential nulls when the array faces the
// source and is ~independent of range and LED brightness. Invalid (returns 0)
// when there is no signal (outer sum below floor) or the cells are saturated
// (too close to range reliably -- hand off to ultrasonic / "close enough, run
// fan"). Calibration constants live in mappings.h and came from the bench sweep
// characterisation (analysis/sensor_usage_guide). The `threshold` arg is no
// longer used (presence is gated on the outer sum, not maxV()).
float FireBank::estimateBearing(float threshold) {
    (void)threshold;
    const float sl  = _sl->getFilteredV();
    const float sr  = _sr->getFilteredV();
    const float sum = sl + sr;

    // Presence & saturation gate.
    if (sum < BEARING_MIN_SUM_V || sl >= PHOTO_SAT_V || sr >= PHOTO_SAT_V) {
        _angleValid = false;
        return 0.0f;
    }

    // Gain-balanced, normalised differential: nulls when the array faces the
    // source; ~independent of range and LED brightness.
    const float nb = (sl - PHOTO_SR_GAIN * sr) / (sl + PHOTO_SR_GAIN * sr);
    float deg = BEARING_DEG_PER_UNIT * nb;        // + = fire toward _sl
    if (deg >  BEARING_MAX_DEG) deg =  BEARING_MAX_DEG;
    if (deg < -BEARING_MAX_DEG) deg = -BEARING_MAX_DEG;

    _angleValid = true;
    return deg;
}

void FireBank::reset() {
    _sl->reset();
    _l->reset();
    _r ->reset();
    _sr ->reset();
}

// ---------------------------------------------------------------------------
// Fan
// ---------------------------------------------------------------------------
void Fan::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _on = false;
}

// Soft-start ramp: short PWM ramp from 0 to 255 over 250 ms before pinning
// the output high. analogWrite on a non-PWM pin would just go HIGH at >127,
// which is fine - we still get a slower turn-on dominated by the MOSFET gate
// charge time. The end state is a hard digitalWrite HIGH so we don't keep
// the timer running.
void Fan::on() {
    if (_on) return;
    digitalWrite(_pin, HIGH);
    _on = true;
    _on_started_ms = millis();
}

void Fan::off() {
    digitalWrite(_pin, LOW);
    _on = false;
}

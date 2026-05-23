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
// Treat the four forward-facing cells as unit vectors.
// Left/Right inner are angled at 10 degrees, outer are angled at 20 degrees.
// Vectors are weighted by their filtered voltage above an ambient floor, 
// returning the angle of their vector sum.
float FireBank::estimateBearing(float threshold) {
    float sl = _sl->getFilteredV();
    float l  = _l->getFilteredV();
    float r  = _r->getFilteredV();
    float sr = _sr->getFilteredV();

    // Early exit if we don't meet the confidence threshold across any sensor
    if (maxV() < threshold) {
        _angleValid = false;
        return 0.0f;
    }

    // Find the ambient floor (minimum reading)
    float ambient = sl;
    if (l < ambient) ambient = l;
    if (r < ambient) ambient = r;
    if (sr < ambient) ambient = sr;

    // Calculate weights (voltage above ambient)
    float w_sl = sl - ambient;
    float w_l  = l - ambient;
    float w_r  = r - ambient;
    float w_sr = sr - ambient;

    // Define vector components based on physical angles
    // 20 degrees: sin(20) = 0.342, cos(20) = 0.940
    // 10 degrees: sin(10) = 0.174, cos(10) = 0.985
    const float sin20 = 0.34202f;
    const float cos20 = 0.93969f;
    const float sin10 = 0.17365f;
    const float cos10 = 0.98481f;

    // Vector sum in robot frame: 
    // X is positive to the right, negative to the left.
    // Y is positive forward.
    float x = sin20*(w_sr - w_sl) + sin10*(w_r - w_l);
    float y = cos20*(w_sl + w_sr) + cos10*(w_l + w_r);

    // Check if vectors cancelled out entirely to avoid an unstable atan2(0,0)
    if (fabs(x) < 1e-6f && fabs(y) < 1e-6f) {
        _angleValid = false;
        return 0.0f;
    }

    _angleValid = true;

    // atan2(y, x) gives angle from +X axis. We want angle from +Y (forward),
    // measured CCW positive (toward body left), which is atan2(-x, y).
    return atan2f(-x, y) * 57.2958;
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
    for (int duty = 0; duty <= 255; duty += 32) {
        analogWrite(_pin, duty);
        delay(30);                              // ~240 ms total ramp
    }
    digitalWrite(_pin, HIGH);
    _on = true;
    _on_started_ms = millis();
}

void Fan::off() {
    digitalWrite(_pin, LOW);
    _on = false;
}

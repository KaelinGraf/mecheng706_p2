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
    front->readSensor();
    right->readSensor();
    rear->readSensor();
    left->readSensor();
}

float FireBank::maxV() const {
    float v = front->getFilteredV();
    if (right->getFilteredV() > v) v = right->getFilteredV();
    if (rear ->getFilteredV() > v) v = rear ->getFilteredV();
    if (left ->getFilteredV() > v) v = left ->getFilteredV();
    return v;
}

bool FireBank::allBelow(float threshold) const {
    return front->getFilteredV() < threshold &&
           right->getFilteredV() < threshold &&
           rear ->getFilteredV() < threshold &&
           left ->getFilteredV() < threshold;
}


// ---------------------------------------------------------------------------
// FireBank::estimateBearing
// ---------------------------------------------------------------------------
// Treat the four cells as unit vectors at 0 / +90 / 180 / -90 deg (front /
// left / rear / right) weighted by their filtered voltage above an ambient
// floor, then return the angle of their vector sum.
//
// Why this works (intuitively): a fire dead ahead lights up "front" the most,
// "rear" the least, and the side cells roughly equally - so the resultant
// vector lies along +Y. A fire to the front-left lights up "front" and "left"
// strongly and "rear"/"right" weakly, so the resultant tilts toward
// (forward, left). The estimator is coarse (quantisation ~ +/-30 deg in
// practice) but it's a behaviour-control bearing, not a metrology bearing.
//
// We subtract the minimum cell value as an ambient offset so a uniformly
// lit room doesn't bias the result toward zero.
float FireBank::estimateBearing(bool* valid, float threshold) const {
    float vf = front->getFilteredV();
    float vl = left ->getFilteredV();
    float vb = rear ->getFilteredV();
    float vr = right->getFilteredV();

    float ambient = vf;
    if (vl < ambient) ambient = vl;
    if (vb < ambient) ambient = vb;
    if (vr < ambient) ambient = vr;

    float wf = vf - ambient;
    float wl = vl - ambient;
    float wb = vb - ambient;
    float wr = vr - ambient;

    // Vector sum in robot frame: x = right - left, y = front - rear.
    // (Bearing convention uses CCW positive: +ve angle <=> +Y x,+X y vector.)
    float x = wr - wl;
    float y = wf - wb;

    if (valid) *valid = (maxV() >= threshold) && (fabs(x) + fabs(y) > 1e-3f);
    if (fabs(x) < 1e-6f && fabs(y) < 1e-6f) return 0.0f;

    // atan2(y, x) gives angle from +X axis. We want angle from +Y (forward),
    // measured CCW positive (toward body left), which is atan2(-x, y).
    return atan2f(-x, y);
}

void FireBank::reset() {
    front->reset();
    right->reset();
    rear ->reset();
    left ->reset();
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

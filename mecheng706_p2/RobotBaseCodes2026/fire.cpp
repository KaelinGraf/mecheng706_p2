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
float FireBank::estimateBearing(float threshold) {
    float sl = _sl->getFilteredV();
    float l  = _l->getFilteredV();
    float r  = _r->getFilteredV();
    float sr = _sr->getFilteredV();

    // Early exit if we don't meet the absolute minimum confidence threshold
    if (maxV() < threshold) {
        _angleValid = false;
        return 0.0f;
    }

    // --- CLOSE RANGE: Inner-pair vector sum ---
    // Trigger if the max of the two middle sensors is above 1.0V
    if (l > 0.6f || r > 0.6f) {
        // 10 degrees: sin(10) = 0.174, cos(10) = 0.985
        const float sin10 = 0.17365f;
        const float cos10 = 0.98481f;
        
        float x = sin10 * (r - l);
        float y = cos10 * (l + r);
        
        if (fabs(x) < 1e-6f && fabs(y) < 1e-6f) {
            _angleValid = false;
            return 0.0f;
        }
        
        _angleValid = true;
        return atan2f(-x, y) * 57.2958f;
    }
    
    // --- FAR RANGE: Outer-pair differential ---
    else {
        // Calibrated constants from the guide
        const float PHOTO_SR_GAIN = 1.0f;
        const float BEARING_DEG_PER_UNIT = 58.0f;
        const float PHOTO_SAT_V = 4.80f;
        const float BEARING_MIN_SUM_V = 1;
        const float BEARING_MAX_DEG = 28.0f;
        
        float sum = sl + sr;

        if (sum < BEARING_MIN_SUM_V) {
            _angleValid = false;
            return 0.0f; 
        }
        
        // Saturation and presence gating
        if (sl >= PHOTO_SAT_V && sr >= PHOTO_SAT_V) {
            _angleValid = true;
            return 0.0f; 
        }
        
        // Gain-balanced, normalised differential equation
        float nb = (sl - PHOTO_SR_GAIN * sr) / (sl + PHOTO_SR_GAIN * sr);
        float deg = BEARING_DEG_PER_UNIT * nb;
        
        // Clamp to valid bearing envelope
        if (deg > BEARING_MAX_DEG) deg = BEARING_MAX_DEG;
        if (deg < -BEARING_MAX_DEG) deg = -BEARING_MAX_DEG;
        
        _angleValid = true;
        return deg;
    }
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

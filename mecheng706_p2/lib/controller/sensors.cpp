#include "sensors.h"


// ---------------------------------------------------------------------------
// Sensor base
// ---------------------------------------------------------------------------
Sensor::Sensor(IAnalogInput* adc, uint8_t read_pin)
    : _read_pin(read_pin), _adc(adc), mapping_reading_(0.0f) {}

float Sensor::readSensor() {
    return applyCalibration(_adc->read_voltage(_read_pin));
}

float Sensor::readSensorRaw() {
    return _adc->read_voltage(_read_pin);
}

void Sensor::setReadPin(uint8_t new_pin) { _read_pin = new_pin; }

uint8_t Sensor::getReadPin() { return _read_pin; }

float Sensor::readSensorFiltered(IClock* clock, int nSamples, int delayMs) {
    const int MAX_SAMPLES = 20;
    if (nSamples > MAX_SAMPLES) nSamples = MAX_SAMPLES;
    if (nSamples < 1) nSamples = 1;

    float samples[MAX_SAMPLES];
    int valid = 0;

    for (int i = 0; i < nSamples; i++) {
        float reading = readSensor();
        bool finite = !(reading != reading);  // NaN check (no <math.h> dependency)
        if (finite && reading > 0.0f) {
            int j = valid;
            while (j > 0 && samples[j - 1] > reading) {
                samples[j] = samples[j - 1];
                j--;
            }
            samples[j] = reading;
            valid++;
        }
        if (delayMs > 0 && i < nSamples - 1 && clock) {
            unsigned long start = clock->now_ms();
            while (clock->now_ms() - start < (unsigned long)delayMs) {
                // busy-wait; Arduino delay() would block exactly the same way
            }
        }
    }

    if (valid == 0) {
        mapping_reading_ = -1.0f;
        return -1.0f;
    }
    if (valid == 1) {
        mapping_reading_ = samples[0];
        return samples[0];
    }

    if (valid % 2 == 1) {
        mapping_reading_ = samples[valid / 2];
        return samples[valid / 2];
    } else {
        float read = (samples[valid / 2 - 1] + samples[valid / 2]) / 2.0f;
        mapping_reading_ = read;
        return read;
    }
}


// ---------------------------------------------------------------------------
// ShortRangeIR
// ---------------------------------------------------------------------------
float ShortRangeIR::readSensor() {
    float new_reading = applyCalibration(_adc->read_voltage(_read_pin));
    mapping_reading_ = new_reading;
    return new_reading;
}

float ShortRangeIR::getAvg() {
    float val = this->readSensor();
    if (val != -1.0f) {
        _prev_measurements->push(val);
    }
    float median = _prev_measurements->median();
    mapping_reading_ = median;
    return median;
}

float ShortRangeIR::applyCalibration(float adc_voltage) {
    // Datasheet linear region: V = (1 / [L + 0.42]) * m + c, valid 0.3V..3V.
    const float c = 0.1097f;
    const float m = 11.33f;
    if (adc_voltage < _min_voltage) return -1.0f;
    if (adc_voltage > _max_voltage) return -1.0f;
    float x = (adc_voltage - c) / m;
    return (1.0f / x) - 0.42f;
}


// ---------------------------------------------------------------------------
// LongRangeIR
// ---------------------------------------------------------------------------
float LongRangeIR::readSensor() {
    float new_reading = applyCalibration(_adc->read_voltage(_read_pin));
    mapping_reading_ = new_reading;
    return new_reading;
}

float LongRangeIR::getAvg() {
    float val = this->readSensor();
    if (val != -1.0f) {
        _prev_measurements->push(val);
    }
    float median = _prev_measurements->median();
    mapping_reading_ = median;
    return median;
}

float LongRangeIR::applyCalibration(float adc_voltage) {
    // Datasheet linear region: V = (1 / L) * m + c. L = m / (V - c). 0.3V..3V.
    const float m = 18.744f;
    const float c = 0.3196f;
    if (adc_voltage < _min_voltage) return -1.0f;
    if (adc_voltage > _max_voltage) return -1.0f;
    return 1.0f / ((adc_voltage - c) / m);
}

float LongRangeIR::readSensorKalman() {
    float raw_dist = this->readSensor();
    if (raw_dist < 0.0f) return _kalman_estimate;

    if (_kalman_estimate < 0.0f) {
        _kalman_estimate = raw_dist;
        return _kalman_estimate;
    }

    float a_priori_est = _kalman_estimate;
    float a_priori_var = _last_y_var + process_noise_;

    float kalman_gain = a_priori_var / (a_priori_var + sensor_noise_);
    float a_post_est = a_priori_est + kalman_gain * (raw_dist - a_priori_est);
    float a_post_var = (1.0f - kalman_gain) * a_priori_var;

    _kalman_estimate = a_post_est;
    _last_y_var = a_post_var;
    return _kalman_estimate;
}

void LongRangeIR::resetKalman() {
    _kalman_estimate = -1.0f;
    _last_y_var = 0.1f;
}


// ---------------------------------------------------------------------------
// Ultrasonic
// ---------------------------------------------------------------------------
Ultrasonic::Ultrasonic(IUltrasonicSource* source, IClock* clock, int max_dist)
    : _source(source), _clock(clock), _max_dist((unsigned int)max_dist) {
    _prev_measurements = new RingBuffer<float, 3>();
    initUltrasonic();
}

void Ultrasonic::runUltrasonic() {
    if (_source) _source->trigger();
}

void Ultrasonic::initUltrasonic() {
    runUltrasonic();
    (void)readSensor();
}

float Ultrasonic::readSensor() {
    if (!_source) {
        mapping_reading_ = -1.0f;
        return -1.0f;
    }

    unsigned long pulse_width = _source->last_pulse_width_us();

    // 58us per cm, per the HC-SR04 datasheet (round-trip / speed-of-sound).
    float cm = (float)pulse_width / 58.0f;

    runUltrasonic();

    if (pulse_width > _max_dist || cm < 10.0f || cm > 200.0f) {
        mapping_reading_ = -1.0f;
        return -1.0f;
    }

    mapping_reading_ = cm;
    return cm;
}

float Ultrasonic::getAvg() {
    float val = this->readSensor();
    if (val != -1.0f && val <= 250.0f) {
        _prev_measurements->push(val);
    }
    float median = _prev_measurements->median();
    mapping_reading_ = median;
    return mapping_reading_;
}


// ---------------------------------------------------------------------------
// Gyroscope
// ---------------------------------------------------------------------------
Gyroscope::Gyroscope(IGyroSource* source, IClock* clock)
    : _source(source), _clock(clock) {
    _prev_measurements = new RingBuffer<float, 4>();
    if (_clock) _prev_micros = _clock->now_us();
}

float Gyroscope::readSensor(bool apply_filter) {
    float omega = 0.0f;
    if (!_source || !_source->poll_omega(&omega)) {
        return -1001.0f;
    }

    uint32_t now = _clock ? _clock->now_us() : _prev_micros;
    float dt = (now - _prev_micros) / 1000000.0f;
    _prev_micros = now;
    _rad += omega * dt;

    _prev_measurements->push(omega);
    _last_omega = omega;
    mapping_reading_ = omega;
    return apply_filter ? _prev_measurements->average() : omega;
}

void Gyroscope::resetAngle() {
    _rad = 0.0f;
    if (_clock) _prev_micros = _clock->now_us();
}

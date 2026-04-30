#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "mappings.h"
#include "utils.h"
#include "ianalog_input.h"
#include "igyro_source.h"
#include "iultrasonic_source.h"
#include "iclock.h"

// Class methods for reading and calibrating different sensor variants.
// All Arduino-specific I/O has been pushed behind the HAL interfaces; the
// algorithm code below (calibration curves, ring-buffer medians, Kalman
// filtering, gyro integration) compiles unchanged on host and on Mega.


class Sensor {
protected:
    uint8_t _read_pin;
    IAnalogInput* _adc;
    virtual float applyCalibration(float adc_voltage) = 0;

public:
    Sensor(IAnalogInput* adc, uint8_t read_pin);
    virtual ~Sensor() {}
    virtual float readSensor();
    float readSensorRaw();
    float readSensorFiltered(IClock* clock, int nSamples, int delayMs = 0);
    virtual void setReadPin(uint8_t new_pin);
    virtual uint8_t getReadPin();
    float mapping_reading_;
};


class ShortRangeIR : public Sensor {
private:
    float _min_voltage = 0.45;
    float _max_voltage = 2.9;
    RingBuffer<float, 3>* _prev_measurements;

public:
    ShortRangeIR(IAnalogInput* adc, uint8_t read_pin) : Sensor(adc, read_pin) {
        _prev_measurements = new RingBuffer<float, 3>();
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
    float getAvg();
    inline void resetBuffer() { this->_prev_measurements->reset(); }
};


class LongRangeIR : public Sensor {
private:
    float _kalman_estimate;
    float _min_voltage = 0.4;
    float _max_voltage = 3.0;
    float _last_y_var = 0.1;
    float process_noise_ = 0.001;
    float sensor_noise_ = 0.2;
    RingBuffer<float, 3>* _prev_measurements;

public:
    LongRangeIR(IAnalogInput* adc, uint8_t read_pin) : Sensor(adc, read_pin) {
        _kalman_estimate = -1.0;
        _prev_measurements = new RingBuffer<float, 3>();
    }
    float readSensor() override;
    float readSensorKalman();
    void resetKalman();
    float applyCalibration(float adc_voltage) override;
    inline float getKalmanEst() { return _kalman_estimate; }
    float getAvg();
    inline void resetBuffer() { this->_prev_measurements->reset(); }
};


class Ultrasonic {
private:
    IUltrasonicSource* _source;
    IClock* _clock;
    unsigned int _max_dist;
    RingBuffer<float, 3>* _prev_measurements;

public:
    Ultrasonic(IUltrasonicSource* source, IClock* clock, int max_dist);

    float mapping_reading_ = -1.0f;

    float readSensor();
    float getAvg();

    // Issue a fresh ping. On Arduino this drives the trigger pin high for
    // ~10us; on sim it's a no-op since the source is always up to date.
    void runUltrasonic();

    // Initialise the trigger / first ping. Safe to call multiple times.
    void initUltrasonic();

    inline void resetBuffer() { this->_prev_measurements->reset(); }
};


class Gyroscope {
private:
    IGyroSource* _source;
    IClock* _clock;
    float _rad = 0.0;
    float _last_omega = 0.0;
    uint32_t _prev_micros = 0;
    RingBuffer<float, 4>* _prev_measurements;
    float _deadband = 0.01;

public:
    Gyroscope(IGyroSource* source, IClock* clock);

    float mapping_reading_ = 0.0f;

    // apply_filter=true returns the moving-average of the last samples,
    // false returns the latest single sample.
    float readSensor(bool apply_filter = false);

    void resetAngle();
    inline float getAngle() const { return _rad; }
};


struct gyroData {};


#endif

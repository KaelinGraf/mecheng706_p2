#include <Arduino.h>
#include "arduino_gyro_source.h"

ArduinoGyroSource::ArduinoGyroSource(Adafruit_BNO08x* bno, sh2_SensorValue_t* sensor_value)
    : _bno(bno), _sensor_value(sensor_value), _initialised(false) {}

bool ArduinoGyroSource::begin() {
    if (_initialised || !_bno) return _initialised;
    // Block until the BNO is ready - matches the original gyro init sequence.
    while (!_bno->begin_I2C() ||
           !_bno->enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
        // Keep retrying; the original code printed an "IMU failed" line here.
        // Without a printer reference we just spin, which is the same effect.
    }
    _bno->enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    _initialised = true;
    return true;
}

bool ArduinoGyroSource::poll_omega(float* omega_z) {
    if (!_bno || !_sensor_value) return false;

    if (_bno->wasReset()) {
        _bno->enableReport(SH2_GYROSCOPE_CALIBRATED);
    }

    if (_bno->getSensorEvent(_sensor_value)) {
        if (_sensor_value->sensorId == SH2_GYROSCOPE_CALIBRATED) {
            if (omega_z) *omega_z = _sensor_value->un.gyroscope.z;
            return true;
        }
    }
    return false;
}

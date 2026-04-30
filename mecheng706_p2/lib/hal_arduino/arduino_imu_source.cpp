#include <Arduino.h>
#include <math.h>
#include "arduino_imu_source.h"

namespace {
// Report cadence in microseconds. 10 ms = 100 Hz, matches the original
// gyro-only path and stays well under the BNO's max rates.
constexpr uint32_t kReportPeriodUs = 10000;
}

ArduinoIMUSource::ArduinoIMUSource(Adafruit_BNO08x* bno,
                                   sh2_SensorValue_t* sensor_value,
                                   IClock* clock)
    : _bno(bno), _sv(sensor_value), _clock(clock),
      _initialised(false),
      _cache(),
      _yaw_fresh(false), _omega_fresh(false), _accel_fresh(false) {}

bool ArduinoIMUSource::begin() {
    if (_initialised || !_bno) return _initialised;
    while (!_bno->begin_I2C()) {
        // Original gyro init spun here; preserve that behavior.
    }
    _enable_reports();
    _initialised = true;
    return true;
}

void ArduinoIMUSource::_enable_reports() {
    _bno->enableReport(SH2_GAME_ROTATION_VECTOR,  kReportPeriodUs);
    _bno->enableReport(SH2_GYROSCOPE_CALIBRATED,  kReportPeriodUs);
    _bno->enableReport(SH2_LINEAR_ACCELERATION,   kReportPeriodUs);
}

void ArduinoIMUSource::_drain_events() {
    if (!_bno || !_sv) return;

    if (_bno->wasReset()) {
        _enable_reports();
    }

    while (_bno->getSensorEvent(_sv)) {
        switch (_sv->sensorId) {
            case SH2_GAME_ROTATION_VECTOR: {
                const float qi = _sv->un.gameRotationVector.i;
                const float qj = _sv->un.gameRotationVector.j;
                const float qk = _sv->un.gameRotationVector.k;
                const float qr = _sv->un.gameRotationVector.real;
                // Yaw around device Z from quaternion (qi, qj, qk, qr=w).
                _cache.yaw_rad = atan2f(2.0f * (qr * qk + qi * qj),
                                        1.0f - 2.0f * (qj * qj + qk * qk));
                _cache.has_yaw = true;
                _yaw_fresh = true;
                break;
            }
            case SH2_GYROSCOPE_CALIBRATED:
                _cache.omega_z = _sv->un.gyroscope.z;
                _cache.has_omega_z = true;
                _omega_fresh = true;
                break;
            case SH2_LINEAR_ACCELERATION:
                _cache.ax = _sv->un.linearAcceleration.x;
                _cache.ay = _sv->un.linearAcceleration.y;
                _cache.az = _sv->un.linearAcceleration.z;
                _cache.has_accel = true;
                _accel_fresh = true;
                break;
            default:
                break;
        }
    }
}

bool ArduinoIMUSource::poll(IMUSample* out) {
    _drain_events();
    const bool any = _yaw_fresh || _omega_fresh || _accel_fresh;
    if (any && out) {
        *out = _cache;
        // Per-modality freshness in the returned sample mirrors the
        // per-consumer freshness this poll is consuming.
        out->has_yaw     = _yaw_fresh;
        out->has_omega_z = _omega_fresh;
        out->has_accel   = _accel_fresh;
        if (_clock) out->t_us = _clock->now_us();
    }
    _yaw_fresh = false;
    _omega_fresh = false;
    _accel_fresh = false;
    return any;
}

bool ArduinoIMUSource::poll_omega(float* omega_z) {
    _drain_events();
    if (!_omega_fresh) return false;
    if (omega_z) *omega_z = _cache.omega_z;
    _omega_fresh = false;
    return true;
}

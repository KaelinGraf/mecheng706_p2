#include <math.h>
#include "imu.h"

namespace {
inline float wrap_pi(float a) {
    constexpr float kPi = 3.14159265358979323846f;
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}
}

IMU::IMU(IIMUSource* src, IClock* clock)
    : _src(src), _clock(clock),
      _yaw(0.0f), _omega(0.0f),
      _ax(0.0f), _ay(0.0f), _az(0.0f),
      _yaw_offset(0.0f),
      _yaw_valid(false), _degraded(false),
      _last_yaw_ms(0), _last_healthy_ms(0) {}

void IMU::update() {
    if (!_src || !_clock) return;
    const unsigned long now_ms = _clock->now_ms();

    IMUSample s;
    if (_src->poll(&s)) {
        if (s.has_yaw) {
            _yaw = wrap_pi(s.yaw_rad - _yaw_offset);
            _yaw_valid = true;
            _last_yaw_ms = now_ms;
        }
        if (s.has_omega_z) _omega = s.omega_z;
        if (s.has_accel) {
            _ax = s.ax;
            _ay = s.ay;
            _az = s.az;
        }
    }

    // Hysteretic degradation flag: true after kDegradeMs of no fresh yaw,
    // false again only after kRestoreMs of healthy yaw flow.
    const unsigned long since_yaw =
        (_last_yaw_ms == 0) ? now_ms : (now_ms - _last_yaw_ms);

    if (!_degraded && _yaw_valid && since_yaw > kDegradeMs) {
        _degraded = true;
        _last_healthy_ms = now_ms;     // restart the restore timer
    } else if (_degraded && since_yaw <= kDegradeMs) {
        if ((now_ms - _last_healthy_ms) > kRestoreMs) {
            _degraded = false;
        }
    } else if (_degraded) {
        // Still missing samples - keep the restore clock from advancing.
        _last_healthy_ms = now_ms;
    }
}

void IMU::tare_yaw() {
    if (!_yaw_valid) return;
    _yaw_offset = wrap_pi(_yaw_offset + _yaw);
    _yaw = 0.0f;
}

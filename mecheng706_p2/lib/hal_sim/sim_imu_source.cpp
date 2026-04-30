#include "sim_imu_source.h"

bool SimIMUSource::poll(IMUSample* out) {
    const bool any = _yaw_fresh || _omega_fresh || _accel_fresh;
    if (any && out) {
        *out = _cache;
        out->has_yaw     = _yaw_fresh;
        out->has_omega_z = _omega_fresh;
        out->has_accel   = _accel_fresh;
    }
    _yaw_fresh = false;
    _omega_fresh = false;
    _accel_fresh = false;
    return any;
}

bool SimIMUSource::poll_omega(float* omega_z) {
    if (!_omega_fresh) return false;
    if (omega_z) *omega_z = _cache.omega_z;
    _omega_fresh = false;
    return true;
}

void SimIMUSource::push_yaw(float yaw_rad) {
    _cache.yaw_rad = yaw_rad;
    _cache.has_yaw = true;
    _yaw_fresh = true;
}

void SimIMUSource::push_omega(float omega_z) {
    _cache.omega_z = omega_z;
    _cache.has_omega_z = true;
    _omega_fresh = true;
}

void SimIMUSource::push_accel(float ax, float ay, float az) {
    _cache.ax = ax;
    _cache.ay = ay;
    _cache.az = az;
    _cache.has_accel = true;
    _accel_fresh = true;
}

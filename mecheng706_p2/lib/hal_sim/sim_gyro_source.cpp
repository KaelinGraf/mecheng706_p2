#include "sim_gyro_source.h"

bool SimGyroSource::poll_omega(float* omega_z) {
    if (!_fresh) return false;
    if (omega_z) *omega_z = _omega_z;
    _fresh = false;
    return true;
}

void SimGyroSource::push_omega(float omega_z) {
    _omega_z = omega_z;
    _fresh = true;
}

#ifndef HAL_SIM_IMU_SOURCE_H
#define HAL_SIM_IMU_SOURCE_H

#include "iimu_source.h"
#include "igyro_source.h"

// Sim IMU source. The host pushes the latest yaw / omega / accel each
// tick; poll() and poll_omega() return the most recently pushed values
// guarded by per-modality freshness flags. Implements both interfaces so
// a single instance serves the new IMU sensor wrapper and the legacy
// Gyroscope sensor wrapper during the migration.
class SimIMUSource : public IIMUSource, public IGyroSource {
public:
    SimIMUSource() = default;

    bool poll(IMUSample* out) override;
    bool poll_omega(float* omega_z) override;

    void push_yaw(float yaw_rad);
    void push_omega(float omega_z);
    void push_accel(float ax, float ay, float az);

private:
    IMUSample _cache{};
    bool _yaw_fresh = false;
    bool _omega_fresh = false;
    bool _accel_fresh = false;
};

#endif

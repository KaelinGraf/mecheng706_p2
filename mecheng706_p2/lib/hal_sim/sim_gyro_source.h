#ifndef HAL_SIM_GYRO_SOURCE_H
#define HAL_SIM_GYRO_SOURCE_H

#include "igyro_source.h"

// Sim gyro source. The host pushes the latest omega each tick; poll_omega()
// returns the most recently pushed value. Track a "fresh" flag so multiple
// polls within one tick don't double-count - matches the Arduino path where
// the BNO doesn't always have a fresh sample on every loop iteration.
class SimGyroSource : public IGyroSource {
public:
    SimGyroSource() : _omega_z(0.0f), _fresh(false) {}

    bool poll_omega(float* omega_z) override;

    // Push a new omega sample (rad/s). Marks the value fresh so the next
    // poll() returns true. Call once per sim tick at the gyro's effective
    // sample rate.
    void push_omega(float omega_z);

private:
    float _omega_z;
    bool _fresh;
};

#endif

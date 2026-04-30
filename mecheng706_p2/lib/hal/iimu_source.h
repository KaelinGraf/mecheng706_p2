#ifndef HAL_IIMU_SOURCE_H
#define HAL_IIMU_SOURCE_H

#include <stdint.h>

// Multi-output IMU sample. Each `has_*` flag tells the consumer which
// fields were refreshed since the last successful poll. Fields without a
// fresh sample retain their previous value (caller can ignore them) -
// flags exist so the consumer can timestamp arrival of each modality
// independently.
//
// Frame: device frame as mounted on the robot. Yaw is the rotation
// around the local Z axis derived from the BNO's game-rotation-vector
// quaternion (no magnetometer, immune to motor-induced field noise).
struct IMUSample {
    bool  has_yaw      = false;
    float yaw_rad      = 0.0f;     // wrapped (-pi, pi]

    bool  has_omega_z  = false;
    float omega_z      = 0.0f;     // rad/s, calibrated gyro Z

    bool  has_accel    = false;
    float ax           = 0.0f;     // m/s^2, gravity removed (linear accel)
    float ay           = 0.0f;
    float az           = 0.0f;

    uint32_t t_us      = 0;        // capture timestamp, 0 if unavailable
};

// Multi-axis IMU source. The Arduino backend wraps the BNO08x and dispatches
// rotation-vector / gyro / linear-accel reports into the cached sample; the
// sim backend lets the host push values directly. poll() returns true when
// any modality refreshed since the last successful poll, and clears its own
// freshness flags.
class IIMUSource {
public:
    virtual ~IIMUSource() = default;
    virtual bool poll(IMUSample* out) = 0;
};

#endif

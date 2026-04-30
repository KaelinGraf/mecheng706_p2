#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "iimu_source.h"
#include "iclock.h"

// Controller-side IMU wrapper. Consumes IIMUSource each update() and
// exposes a small API the rest of the controller can depend on without
// caring about poll cadence or BNO sensor IDs.
//
// Yaw is the quaternion-derived absolute yaw produced by the BNO's
// game-rotation-vector report (no magnetometer; immune to motor noise).
// degraded() returns true after no fresh yaw sample for kDegradeMs - the
// MotionController checks this before running closed-loop heading PID
// and falls back to open-loop yaw rate when set.
class IMU {
public:
    static const unsigned long kDegradeMs = 250;
    static const unsigned long kRestoreMs = 1000;

    IMU(IIMUSource* src, IClock* clock);

    // Drain pending samples and update cached state. Call once per loop
    // before any consumer reads yaw / omega / degraded.
    void update();

    inline float yaw() const        { return _yaw; }
    inline float omega_z() const    { return _omega; }
    inline float ax() const         { return _ax; }
    inline float ay() const         { return _ay; }
    inline float az() const         { return _az; }

    inline bool yaw_valid() const   { return _yaw_valid; }
    inline bool degraded() const    { return _degraded; }

    // Reset the absolute-yaw reference to "current heading is 0 rad".
    // Call after a confident initial alignment (e.g. immediately after
    // the brief startup nudge so the robot's facing is taken as forward).
    void tare_yaw();

private:
    IIMUSource* _src;
    IClock*     _clock;

    float _yaw;
    float _omega;
    float _ax, _ay, _az;
    float _yaw_offset;           // subtracted from raw yaw on read

    bool _yaw_valid;
    bool _degraded;

    unsigned long _last_yaw_ms;
    unsigned long _last_healthy_ms;
};

#endif

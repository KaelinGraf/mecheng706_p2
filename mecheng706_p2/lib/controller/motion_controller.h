#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "motion_command.h"
#include "servo_control.h"
#include "heading_pid.h"
#include "imu.h"
#include "iclock.h"

// Sole caller of driveMotors. Behaviors (or, transitionally, FSM states)
// hand a MotionCommand to apply() each tick; this layer:
//   * passes OPEN_LOOP_RATE commands straight through unchanged,
//   * runs HeadingPid for CLOSED_LOOP_ABS / CLOSED_LOOP_BEARING when an
//     IMU is wired and reports valid yaw,
//   * silently degrades closed-loop to open-loop (using vtheta_cmd, or
//     zero if absent) when the IMU is missing or degraded.
//
// In step 2 of the FSM-to-behavior migration, only OPEN_LOOP_RATE is in
// use; closed-loop wiring becomes live in step 6.
class MotionController {
public:
    MotionController(driveMotors* motors, IClock* clock, IMU* imu = nullptr);

    void apply(const MotionCommand& cmd);

    inline HeadingPid& pid() { return _pid; }
    inline void set_imu(IMU* imu) { _imu = imu; }

private:
    driveMotors* _motors;
    IClock*      _clock;
    IMU*         _imu;
    HeadingPid   _pid;
    uint32_t     _last_us;
};

#endif

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

// Body-frame motion request handed to MotionController. Behaviors emit one
// of these every tick; the arbiter forwards the winning behavior's command
// untouched. MotionController is responsible for closing the heading loop
// (in CLOSED_LOOP_* modes) and forwarding to driveMotors.
//
// Units mirror the existing driveMotors API: vx/vy are body-frame strafe
// and forward in the +/-300 control-effort scale (open-loop PWM mixing,
// see lib/controller/servo_control.cpp). vtheta_cmd is in the same scale.
// yaw_target is in radians, wrapped (-pi, pi].
struct MotionCommand {
    enum HeadingMode {
        OPEN_LOOP_RATE,            // forward vtheta_cmd straight through
        CLOSED_LOOP_ABS,           // PID(yaw_target - imu.yaw())
        CLOSED_LOOP_BEARING,       // PID(yaw_target - 0); behavior precomputed err
    };

    float vx;
    float vy;
    float vtheta_cmd;
    float yaw_target;
    HeadingMode heading_mode;

    // Default-constructible to all-zero / open-loop. The explicit constructor
    // (rather than C++14 default-member-initializer aggregate init) keeps
    // brace-init at call sites compatible with avr-gcc.
    MotionCommand()
        : vx(0.0f), vy(0.0f), vtheta_cmd(0.0f),
          yaw_target(0.0f), heading_mode(OPEN_LOOP_RATE) {}

    MotionCommand(float vx_, float vy_, float vtheta_)
        : vx(vx_), vy(vy_), vtheta_cmd(vtheta_),
          yaw_target(0.0f), heading_mode(OPEN_LOOP_RATE) {}

    MotionCommand(float vx_, float vy_, float vtheta_,
                  float yaw_target_, HeadingMode hm)
        : vx(vx_), vy(vy_), vtheta_cmd(vtheta_),
          yaw_target(yaw_target_), heading_mode(hm) {}
};

#endif

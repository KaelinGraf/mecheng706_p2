#include <math.h>
#include "motion_controller.h"

namespace {
inline float wrap_pi(float a) {
    constexpr float kPi = 3.14159265358979323846f;
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}
}

MotionController::MotionController(driveMotors* motors, IClock* clock, IMU* imu)
    : _motors(motors), _clock(clock), _imu(imu), _pid(), _last_us(0) {}

void MotionController::apply(const MotionCommand& cmd) {
    if (!_motors) return;

    float vtheta = cmd.vtheta_cmd;

    // Compute dt for the heading PID (clamped against long stalls).
    float dt_s = 0.0f;
    if (_clock) {
        const uint32_t now_us = _clock->now_us();
        if (_last_us != 0) {
            const uint32_t du = now_us - _last_us;
            dt_s = du * 1e-6f;
            if (dt_s > 0.2f) dt_s = 0.2f;
        }
        _last_us = now_us;
    }

    const bool imu_ok = _imu && _imu->yaw_valid() && !_imu->degraded();

    switch (cmd.heading_mode) {
        case MotionCommand::OPEN_LOOP_RATE:
            // Pass-through; vtheta already populated.
            break;

        case MotionCommand::CLOSED_LOOP_ABS:
            if (imu_ok) {
                const float err = wrap_pi(cmd.yaw_target - _imu->yaw());
                vtheta = _pid.compute(err, dt_s);
            }
            // else: fall back to vtheta_cmd already in `vtheta`.
            break;

        case MotionCommand::CLOSED_LOOP_BEARING:
            // yaw_target is interpreted as a body-frame error (target -
            // current already pre-computed by the behavior). PID just
            // smooths it. Falls back to vtheta_cmd if no IMU.
            if (imu_ok) {
                const float err = wrap_pi(cmd.yaw_target);
                vtheta = _pid.compute(err, dt_s);
            }
            break;
    }

    _motors->writeAllMotors(cmd.vx, cmd.vy, vtheta);
}

#ifndef HEADING_PID_H
#define HEADING_PID_H

#include "iclock.h"

// Tiny PID for heading control. Used by MotionController in CLOSED_LOOP_*
// heading modes to convert a yaw error into a vtheta effort in the same
// +/-300 scale as the open-loop path. Derivative term uses the controller
// dt (clock-based) rather than measurement derivative.
class HeadingPid {
public:
    HeadingPid(float kp = 80.0f, float ki = 0.0f, float kd = 5.0f,
               float effort_limit = 200.0f);

    // Reset internal state. Call when re-engaging closed-loop after an
    // open-loop interlude so accumulated error does not surprise.
    void reset();

    // Set gains at runtime (for tuning over BT).
    void set_gains(float kp, float ki, float kd);

    // Compute control effort (yaw rate units, same scale as vtheta_cmd).
    // err_rad is target minus measured, wrapped to (-pi, pi].
    // dt_s is the time since the last call; if zero or negative the
    // derivative term is skipped (single-step bootstrap).
    float compute(float err_rad, float dt_s);

private:
    float _kp;
    float _ki;
    float _kd;
    float _i_acc;
    float _last_err;
    bool  _has_last;
    float _effort_limit;
};

#endif

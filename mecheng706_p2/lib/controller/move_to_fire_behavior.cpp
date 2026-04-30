#include <math.h>
#include "move_to_fire_behavior.h"
#include "mappings.h"

namespace {
// Returns the smallest forward-arc distance that is currently valid (>0).
// Used to gate "should ExtinguishFire take over" logic.
inline float _front_min(const WorldView& w) {
    float best = 1e6f;
    if (w.us_cm > 0.0f          && w.us_cm < best)          best = w.us_cm;
    if (w.front_left_ir > 0.0f  && w.front_left_ir < best)  best = w.front_left_ir;
    if (w.front_right_ir > 0.0f && w.front_right_ir < best) best = w.front_right_ir;
    return best;
}
}

BehaviorOutput MoveToFireBehavior::tick(const WorldView& w) {
    BehaviorOutput out;

    if (!w.turret.is_locked) {
        out.active = false;
        return out;
    }

    // ExtinguishFire owns this once we are in range and aimed; abstain.
    const float front = _front_min(w);
    const bool in_range = (front <= EXTINGUISH_RANGE_CM);
    const bool aimed   = fabsf(w.turret.body_relative_bearing_rad) < EXTINGUISH_AIM_RAD;
    if (in_range && aimed) {
        out.active = false;
        return out;
    }

    out.active = true;
    out.tag = "move_to_fire";

    const float bearing = w.turret.body_relative_bearing_rad;

    float vy;
    if (fabsf(bearing) > MOVETOFIRE_PIVOT_RAD) {
        vy = 0.0f;                               // pivot in place when wide
    } else {
        vy = MOVETOFIRE_FORWARD_SPEED * cosf(bearing);
    }

    // Closed-loop heading on the turret's world bearing: while the body
    // rotates, the world bearing stays constant (turret holds it via the
    // IMU), so the PID drives body_yaw -> world_bearing. Fall back to
    // open-loop on the body-relative bearing if the IMU is degraded -
    // MotionController checks IMU::degraded() and uses vtheta_cmd in that
    // case. Mathematically equivalent in the steady state, but the
    // closed-loop form is robust to integrated-yaw drift.
    const float fallback_vtheta = MOVETOFIRE_KP_TURN * bearing;
    out.motion = MotionCommand(0.0f, vy, fallback_vtheta,
                               w.turret.world_bearing_rad,
                               MotionCommand::CLOSED_LOOP_ABS);
    return out;
}

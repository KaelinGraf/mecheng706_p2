#include <math.h>
#include "avoid_obstacle_behavior.h"
#include "mappings.h"

namespace {
inline bool _blocked(float v, float thr) {
    return (v > 0.0f) && (v < thr);
}
inline bool _clear(float v, float thr) {
    return (v < 0.0f) || (v >= thr);
}

// Hard timeout before the strafe-and-creep gives up and rotates in place.
constexpr unsigned long kAvoidHardTimeoutMs = 3000;
constexpr float         kAvoidCreepForward  = 50.0f;
}

AvoidObstacleBehavior::AvoidObstacleBehavior()
    : _engaged(false), _started_ms(0), _strafe_sign(+1) {}

void AvoidObstacleBehavior::on_select() {
    _engaged = false;        // populated on first tick that picks the sign
}
void AvoidObstacleBehavior::on_deselect() {
    _engaged = false;
}

bool AvoidObstacleBehavior::_gated_active(const WorldView& w) const {
    // Front-arc reading "close" gate.
    const bool blocked = _blocked(w.us_cm,          OBSTACLE_TRIGGER_CM) ||
                         _blocked(w.front_left_ir,  OBSTACLE_TRIGGER_CM) ||
                         _blocked(w.front_right_ir, OBSTACLE_TRIGGER_CM);
    if (!blocked) return false;

    // Mask: if the turret is locked on a bright candle in roughly the
    // direction we are about to flag as an obstacle, treat as the fire
    // and abstain - MoveToFire / ExtinguishFire will take over.
    if (w.turret.is_locked && w.turret.intensity >= FIRE_DETECT_V) {
        if (fabsf(w.turret.body_relative_bearing_rad) < AVOID_FIRE_MASK_RAD) {
            return false;
        }
    }
    return true;
}

BehaviorOutput AvoidObstacleBehavior::tick(const WorldView& w) {
    BehaviorOutput out;
    out.tag = "avoid_obstacle";

    if (!_engaged) {
        if (!_gated_active(w)) {
            out.active = false;
            return out;
        }
        // Decide strafe direction from the front-IR pair.
        const bool lf_valid = (w.front_left_ir  > 0.0f);
        const bool rf_valid = (w.front_right_ir > 0.0f);
        if (lf_valid && rf_valid) {
            _strafe_sign = (w.front_left_ir > w.front_right_ir) ? -1 : +1;
        } else if (lf_valid) {
            _strafe_sign = -1;
        } else if (rf_valid) {
            _strafe_sign = +1;
        } else {
            _strafe_sign = +1;
        }
        _started_ms = w.now_ms;
        _engaged = true;
    }

    const unsigned long elapsed = w.now_ms - _started_ms;

    // Once we've strafed long enough, release the moment the forward
    // arc is clear.
    if (elapsed > AVOID_STRAFE_MS) {
        if (_clear(w.us_cm,          OBSTACLE_CLEAR_CM) &&
            _clear(w.front_left_ir,  OBSTACLE_CLEAR_CM) &&
            _clear(w.front_right_ir, OBSTACLE_CLEAR_CM)) {
            _engaged = false;
            out.active = false;
            return out;
        }
    }

    // Hard timeout: rotate in place toward the more-clear side.
    if (elapsed > kAvoidHardTimeoutMs) {
        const float vtheta = -SEARCH_TURN_SPEED * (float)_strafe_sign;
        out.active = true;
        out.motion = MotionCommand(0.0f, 0.0f, vtheta);
        return out;
    }

    // Default: strafe + creep forward.
    out.active = true;
    out.motion = MotionCommand(AVOID_STRAFE_SPEED * (float)_strafe_sign,
                               kAvoidCreepForward,
                               0.0f);
    return out;
}

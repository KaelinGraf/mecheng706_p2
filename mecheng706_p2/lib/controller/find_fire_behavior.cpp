#include "find_fire_behavior.h"
#include "mappings.h"

namespace {
inline bool _blocked(float v, float thr) {
    return (v > 0.0f) && (v < thr);
}
}

FindFireBehavior::FindFireBehavior()
    : _last_nudge_ms(0),
      _nudge_started_ms(0),
      _nudge_active(false),
      _nudge_dir(+1) {}

void FindFireBehavior::on_select() {
    _nudge_active = false;
}

BehaviorOutput FindFireBehavior::tick(const WorldView& w) {
    BehaviorOutput out;
    out.active = true;
    out.tag = "find_fire";

    // Wall-bias on the rear short-range IRs: nudge yaw away from a side
    // that is reporting close. Same shape as the legacy SEARCH layer 1.
    float vtheta = 0.0f;
    if (_blocked(w.rear_left_ir, WALL_FOLLOW_CM)) {
        vtheta -= SEARCH_TURN_SPEED * (1.0f - w.rear_left_ir / WALL_FOLLOW_CM);
    }
    if (_blocked(w.rear_right_ir, WALL_FOLLOW_CM)) {
        vtheta += SEARCH_TURN_SPEED * (1.0f - w.rear_right_ir / WALL_FOLLOW_CM);
    }

    // Periodic exploratory yaw nudge so the boom-mounted turret sees
    // fresh arena even on a long straight run.
    if (!_nudge_active && (w.now_ms - _last_nudge_ms) > FINDFIRE_NUDGE_PERIOD_MS) {
        _nudge_active = true;
        _nudge_started_ms = w.now_ms;
        _nudge_dir = -_nudge_dir;
    }
    if (_nudge_active) {
        if ((w.now_ms - _nudge_started_ms) < FINDFIRE_NUDGE_LEN_MS) {
            vtheta += FINDFIRE_NUDGE_TURN * _nudge_dir;
        } else {
            _nudge_active = false;
            _last_nudge_ms = w.now_ms;
        }
    }

    out.motion = MotionCommand(0.0f, FINDFIRE_FORWARD_SPEED, vtheta);
    return out;
}

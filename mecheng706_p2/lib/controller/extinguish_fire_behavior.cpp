#include <math.h>
#include "extinguish_fire_behavior.h"
#include "mappings.h"

namespace {
inline float _front_min(const WorldView& w) {
    float best = 1e6f;
    if (w.us_cm > 0.0f          && w.us_cm < best)          best = w.us_cm;
    if (w.front_left_ir > 0.0f  && w.front_left_ir < best)  best = w.front_left_ir;
    if (w.front_right_ir > 0.0f && w.front_right_ir < best) best = w.front_right_ir;
    return best;
}
}

ExtinguishFireBehavior::ExtinguishFireBehavior()
    : _active_window(false),
      _started_ms(0),
      _below_count(0),
      _just_completed(false),
      _last_seen_count(0) {}

void ExtinguishFireBehavior::on_select() {
    _active_window = true;
    _started_ms = 0;       // populated on the first tick()
    _below_count = 0;
    _just_completed = false;
}

void ExtinguishFireBehavior::on_deselect() {
    _active_window = false;
}

BehaviorOutput ExtinguishFireBehavior::tick(const WorldView& w) {
    BehaviorOutput out;

    // Did we already complete this fire? Stay abstaining until the
    // arbiter has had a chance to deselect us (which clears the latch).
    if (_just_completed) {
        out.active = false;
        return out;
    }

    // Activation gate: turret locked + body in range + body aimed.
    const float front = _front_min(w);
    const bool gate_locked = w.turret.is_locked;
    const bool gate_range  = (front <= EXTINGUISH_RANGE_CM);
    const bool gate_aimed  = fabsf(w.turret.body_relative_bearing_rad) < EXTINGUISH_AIM_RAD;
    if (!(gate_locked && gate_range && gate_aimed)) {
        out.active = false;
        return out;
    }

    // First tick of activation needs a wall-clock reference for the
    // hard-timeout. on_select() has zeroed _started_ms, populate it now.
    if (_started_ms == 0) _started_ms = w.now_ms;

    // Monitor turret intensity for the fire-out debounce.
    if (w.turret.intensity < FIRE_OUT_V) {
        if (_below_count < 255) ++_below_count;
    } else {
        _below_count = 0;
    }

    const bool fire_out = _below_count >= EXTINGUISH_OUT_DEBOUNCE;
    const bool timed_out = (w.now_ms - _started_ms) >= EXTINGUISH_HARD_TIMEOUT_MS;

    if (fire_out || timed_out) {
        // One-shot: emit the increment and abstain. The arbiter forwards
        // the increment, then the next tick we self-suppress (until
        // on_deselect re-arms).
        _just_completed = true;
        out.active = true;
        out.tag = fire_out ? "extinguish_fire/done(led-out)"
                           : "extinguish_fire/done(timeout)";
        out.motion = MotionCommand();           // zeros
        out.fan_on = false;
        out.request_turret_hold = false;
        out.fires_extinguished_increment = true;
        return out;
    }

    out.active = true;
    out.tag = "extinguish_fire";
    out.motion = MotionCommand();               // zeros
    out.fan_on = true;
    out.request_turret_hold = true;
    return out;
}

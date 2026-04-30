#ifndef EXTINGUISH_FIRE_BEHAVIOR_H
#define EXTINGUISH_FIRE_BEHAVIOR_H

#include <stdint.h>
#include "behavior.h"

// Active when the body is within the 20 cm extinguish radius of a candle
// the turret is locked onto and aimed at:
//   * w.turret.is_locked, AND
//   * front_arc_distance <= EXTINGUISH_RANGE_CM, AND
//   * |w.turret.body_relative_bearing_rad| < EXTINGUISH_AIM_RAD.
//
// Output: zero body motion, fan_on, request_turret_hold (so the boom-
// mounted fan keeps aiming as the LED fades). Monitors the turret photo
// array's intensity dropping below FIRE_OUT_V; after EXTINGUISH_OUT_DEBOUNCE
// consecutive sub-threshold ticks (or EXTINGUISH_HARD_TIMEOUT_MS), emits
// the one-shot fires_extinguished_increment flag and abstains so the
// arbiter falls through to MoveToFire / FindFire / MissionComplete.
class ExtinguishFireBehavior : public IBehavior {
public:
    ExtinguishFireBehavior();

    const char* name() const override { return "extinguish_fire"; }
    BehaviorOutput tick(const WorldView& w) override;
    void on_select() override;
    void on_deselect() override;

private:
    bool          _active_window;
    unsigned long _started_ms;
    uint8_t       _below_count;
    bool          _just_completed;
    int           _last_seen_count;
};

#endif

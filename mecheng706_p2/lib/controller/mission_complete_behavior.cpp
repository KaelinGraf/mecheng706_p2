#include "mission_complete_behavior.h"

BehaviorOutput MissionCompleteBehavior::tick(const WorldView& w) {
    if (w.fires_extinguished >= 2) _latched = true;

    BehaviorOutput out;
    if (!_latched) {
        out.active = false;
        return out;
    }
    out.active = true;
    out.fan_on = false;
    out.request_turret_hold = false;
    out.tag = "mission_complete";
    // motion defaults to all-zero / OPEN_LOOP_RATE.
    return out;
}

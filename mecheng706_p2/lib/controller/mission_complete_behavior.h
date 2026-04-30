#ifndef MISSION_COMPLETE_BEHAVIOR_H
#define MISSION_COMPLETE_BEHAVIOR_H

#include "behavior.h"

// Top-priority behavior. Latches once two fires have been extinguished
// (matches the brief: "the robot must cease all movement immediately
// after extinguishing the second fire"). Output: zero motion, fan off,
// no turret hold. Once latched, returns active=true forever - the
// arbiter never falls through to lower layers, the body stays still.
class MissionCompleteBehavior : public IBehavior {
public:
    MissionCompleteBehavior() : _latched(false) {}

    const char* name() const override { return "mission_complete"; }
    BehaviorOutput tick(const WorldView& w) override;

private:
    bool _latched;
};

#endif

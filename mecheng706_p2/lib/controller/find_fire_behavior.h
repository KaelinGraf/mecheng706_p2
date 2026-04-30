#ifndef FIND_FIRE_BEHAVIOR_H
#define FIND_FIRE_BEHAVIOR_H

#include "behavior.h"

// Lowest-priority behavior. Always active - the arbiter falls through to
// it when nothing else has anything to say. Goal: keep the body moving
// across the arena so the panning turret has fresh sight lines for its
// SWEEP. Slow forward cruise + light wall-bias + occasional yaw nudges.
//
// Holds two pieces of internal state: the next nudge time and the
// nudge-direction sign (alternates so the body weaves rather than
// drifting one way).
class FindFireBehavior : public IBehavior {
public:
    FindFireBehavior();

    const char* name() const override { return "find_fire"; }
    BehaviorOutput tick(const WorldView& w) override;
    void on_select() override;

private:
    unsigned long _last_nudge_ms;
    unsigned long _nudge_started_ms;
    bool          _nudge_active;
    int8_t        _nudge_dir;
};

#endif

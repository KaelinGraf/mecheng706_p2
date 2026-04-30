#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "motion_command.h"
#include "world_view.h"

// Output of a single behavior tick. The arbiter forwards the winning
// behavior's output to MotionController (motion), the Fan (fan_on), and
// the TurretController (request_turret_hold). When a behavior abstains it
// returns active=false and the arbiter falls through to the next layer.
//
// The fires_extinguished_increment flag is a one-shot signal from
// ExtinguishFireBehavior to the FireFighter: when set, the fire counter
// is bumped exactly once before the output is forwarded. The behavior is
// responsible for only setting it on the tick that observes fire-out.
struct BehaviorOutput {
    bool          active;
    MotionCommand motion;
    bool          fan_on;
    bool          request_turret_hold;
    bool          fires_extinguished_increment;
    const char*   tag;

    BehaviorOutput()
        : active(false),
          motion(),
          fan_on(false),
          request_turret_hold(false),
          fires_extinguished_increment(false),
          tag("") {}
};

// Behavior interface. tick() is the per-loop step; on_select / on_deselect
// fire when this behavior wins / loses arbitration relative to the
// previous tick (used to reset internal one-shot timers / debouncers).
//
// Behaviors are weak-pure: they read from a const WorldView and return a
// BehaviorOutput. They may hold their own internal state (timers,
// debounce counters, captured bearings) but should not mutate any global
// state outside their own object.
class IBehavior {
public:
    virtual ~IBehavior() = default;
    virtual const char* name() const = 0;
    virtual BehaviorOutput tick(const WorldView& w) = 0;
    virtual void on_select() {}
    virtual void on_deselect() {}
};

#endif

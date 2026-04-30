#ifndef AVOID_OBSTACLE_BEHAVIOR_H
#define AVOID_OBSTACLE_BEHAVIOR_H

#include <stdint.h>
#include "behavior.h"

// Strafe-and-creep obstacle avoidance, extracted from the legacy AVOID
// FSM state. Active when the front-arc sensors detect a blocker that is
// NOT the candle (a candle is identified by an aligned, bright turret
// lock - see AVOID_FIRE_MASK_RAD).
//
// Strategy (matches lib/controller/avoid.cpp):
//   1. Decide strafe sign on first tick from the front-IR delta.
//   2. Strafe + creep forward at AVOID_STRAFE_SPEED for at least
//      AVOID_STRAFE_MS so the chassis clears the cylinder.
//   3. After the minimum strafe time, release as soon as the forward
//      arc is clear (>= OBSTACLE_CLEAR_CM on every front sensor).
//   4. Hard timeout (3 s) - rotate in place toward the more-clear side.
class AvoidObstacleBehavior : public IBehavior {
public:
    AvoidObstacleBehavior();

    const char* name() const override { return "avoid_obstacle"; }
    BehaviorOutput tick(const WorldView& w) override;
    void on_select() override;
    void on_deselect() override;

private:
    bool _gated_active(const WorldView& w) const;

    bool          _engaged;
    unsigned long _started_ms;
    int8_t        _strafe_sign;        // +1 = right, -1 = left
};

#endif

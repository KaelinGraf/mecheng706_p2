#ifndef MOVE_TO_FIRE_BEHAVIOR_H
#define MOVE_TO_FIRE_BEHAVIOR_H

#include "behavior.h"

// Drives the body toward a candle the turret has locked onto, but only
// while the body is not yet inside the 20 cm extinguish range. Active
// when:
//   * turret.is_locked is true (the turret thinks it has a candle), AND
//   * the front-arc distance reading is greater than EXTINGUISH_RANGE_CM
//     (otherwise ExtinguishFireBehavior takes over).
//
// Output: forward cruise scaled by cos(bearing) (zero if pivoting wider
// than MOVETOFIRE_PIVOT_RAD), open-loop yaw rate proportional to the
// turret's body-relative bearing. Step 6 swaps this for closed-loop
// CLOSED_LOOP_BEARING heading control.
class MoveToFireBehavior : public IBehavior {
public:
    MoveToFireBehavior() = default;

    const char* name() const override { return "move_to_fire"; }
    BehaviorOutput tick(const WorldView& w) override;
};

#endif

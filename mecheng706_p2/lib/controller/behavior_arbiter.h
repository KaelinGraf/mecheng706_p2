#ifndef BEHAVIOR_ARBITER_H
#define BEHAVIOR_ARBITER_H

#include <stdint.h>
#include "behavior.h"

// Strict-priority arbiter. Layer 0 is highest priority (top wins). The
// arbiter calls each layer's tick() in order and returns the first one
// whose output is active=true. on_select / on_deselect are dispatched on
// winner-change relative to the previous tick.
//
// Layout (top to bottom for this project):
//   0. mission_complete   - latches forever after 2nd fire is out
//   1. avoid_obstacle     - body sensors detect a blocker (not the candle)
//   2. extinguish_fire    - turret locked + body in 20 cm range
//   3. move_to_fire       - turret locked, body out of range
//   4. find_fire          - default cruise/sweep aid
class BehaviorArbiter {
public:
    static const uint8_t MAX_LAYERS = 5;

    BehaviorArbiter();

    // Install a behavior at a given priority slot (0 = highest).
    void install(uint8_t idx, IBehavior* behavior);

    // Run one arbitration cycle. Returns the winner's output, or a
    // zero/idle output if every layer abstained. Invokes on_select /
    // on_deselect on the layers as the winner changes.
    BehaviorOutput tick(const WorldView& w);

    inline IBehavior* last_winner() const { return _last_winner; }

private:
    IBehavior* _layers[MAX_LAYERS];
    IBehavior* _last_winner;
};

#endif

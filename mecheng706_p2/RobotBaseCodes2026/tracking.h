#ifndef TRACKING_H
#define TRACKING_H

#include "behavior.h"

class FireFighter;

// TRACKING owns the SEARCH sub-behaviours:
// - FIND_FIRE: cruise, wall-follow, and exploratory nudging
// - AVOID: strafe/rotate around an obstacle
// - MOVE_TO_FIRE: steer toward the detected bearing
class Tracking {
public:
    enum class AvoidMode {
        NONE,
        LEFT,
        RIGHT,
        SIDE,
        AHEAD,
    };

    explicit Tracking(FireFighter *firefighter);

    void begin();
    void end();
    void poll();

private:
    FireFighter *firefighter_;
    BehaviorNS::SearchBehaviour active_behavior_;

    float bearing_;
    float resume_bearing_;
    bool resume_to_move_;

    int strafe_sign_;
    AvoidMode last_avoid_mode_;
    unsigned long behavior_start_ms_;
    unsigned long last_seen_ms_;

    unsigned long last_nudge_ms_;
    unsigned long nudge_start_ms_;
    bool nudge_active_;
    int nudge_dir_;
};

#endif // TRACKING_H
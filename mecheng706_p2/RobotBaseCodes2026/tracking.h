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
    explicit Tracking(FireFighter *firefighter);

    void begin();
    void end();
    void poll();

private:
    void enterFindFire();
    void enterAvoid(bool resume_to_move);
    void enterMoveToFire(float bearing);

    FireFighter *firefighter_;
    BehaviorNS::SearchBehaviour active_behavior_;

    float bearing_;
    float resume_bearing_;
    bool resume_to_move_;

    int strafe_sign_;
    unsigned long behavior_start_ms_;
    unsigned long last_seen_ms_;

    unsigned long last_nudge_ms_;
    unsigned long nudge_start_ms_;
    bool nudge_active_;
    int nudge_dir_;
};

#endif // TRACKING_H
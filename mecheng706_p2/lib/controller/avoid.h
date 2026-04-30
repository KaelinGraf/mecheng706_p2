#ifndef AVOID_H
#define AVOID_H

#include <math.h>
#include "state.h"
#include "pid.h"

// AVOID: steer/strafe around an obstacle or wall using IR + ultrasonic cues.
// Returns to SEARCH (or APPROACH if fire bearing is still hot) when clear.
class Avoid : public State {
  public:
    Avoid(FireFighter* firefighter) : State(State::AVOID, firefighter) {};
    ~Avoid() {};

    void begin() override;
    void begin(StateData data) override;
    void end() override;
    void poll() override;

  private:
    // Direction we're strafing this AVOID episode. +1 = right (vx > 0),
    // -1 = left (vx < 0). Decided once at begin() based on which front IR
    // reports more clearance, and held for the duration so we don't dither.
    int _strafe_sign = 1;

    // When this episode started (millis). Used to give the strafe a minimum
    // duration before we re-test for clearance, and an upper bound after
    // which we fall back to rotating in place.
    unsigned long _start_ms = 0;

    // If AVOID was entered from APPROACH, this carries the latest fire
    // bearing so we can resume APPROACH straight from AVOID without a
    // SEARCH detour. NaN means "no fire in progress".
    float _resume_bearing = NAN;
    bool  _resume_to_approach = false;
};


#endif // AVOID_H

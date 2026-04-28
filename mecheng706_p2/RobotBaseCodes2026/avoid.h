#ifndef AVOID_H
#define AVOID_H

#include "state.h"
#include "pid.h"

// AVOID: steer/strafe around an obstacle or wall using IR + ultrasonic cues.
// Returns to SEARCH (or APPROACH if fire bearing is still hot) when clear.
class Avoid : public State {
  public:
    Avoid(FireFighter* firefighter) : State(State::AVOID, firefighter) {};
    ~Avoid() {};

    void begin() override;
    void end() override;
    void poll() override;
};


#endif // AVOID_H

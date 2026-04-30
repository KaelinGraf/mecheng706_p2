#ifndef APPROACH_H
#define APPROACH_H

#include "state.h"
#include "pid.h"

// APPROACH: head toward a detected fire bearing until the robot's centre is
// within 20 cm of the fire's centre, then transition to EXTINGUISH.
class Approach : public State {
  public:
    Approach(FireFighter* firefighter) : State(State::APPROACH, firefighter) {};
    ~Approach() {};

    void begin() override;
    void begin(StateData data) override;
    void end() override;
    void poll() override;

  private:
    float _bearing = 0.0;            // bearing to fire (rad), set via begin(StateData)
    unsigned long _last_seen_ms = 0; // millis() of last valid fire detection (for lost-signal grace period)
};


#endif // APPROACH_H

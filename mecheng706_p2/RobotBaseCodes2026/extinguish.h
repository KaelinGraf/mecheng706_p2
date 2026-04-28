#ifndef EXTINGUISH_H
#define EXTINGUISH_H

#include "state.h"

// EXTINGUISH: run the fan (via MOSFET) for up to 10 s. The thermistor in the
// fire deactivates the LED once put out; if the LED goes out earlier the robot
// may proceed immediately. Increments fires_out_; transitions to SEARCH after
// the first fire and to STOPPED after the second.
class Extinguish : public State {
  public:
    Extinguish(FireFighter* firefighter) : State(State::EXTINGUISH, firefighter), _start_millis(0) {};
    ~Extinguish() {};

    void begin() override;
    void end() override;
    void poll() override;

    static constexpr unsigned long FAN_MAX_MS = 10000;

  private:
    unsigned long _start_millis;
    int _fires_out = 0;
};


#endif // EXTINGUISH_H

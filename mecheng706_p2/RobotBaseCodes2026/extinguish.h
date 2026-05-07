#ifndef EXTINGUISH_H
#define EXTINGUISH_H

#include "state.h"
#include "mappings.h"

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

    // Brief: fan runs for up to 10 s. Centralised in mappings.h.
    static constexpr unsigned long FAN_HARD_TIMEOUT_MS = FAN_MAX_MS;

  private:
    unsigned long _start_millis;
    // Counter of consecutive polls where the fire bank reads below FIRE_OUT_V.
    // Acts as a debounce: the LED can flicker as the thermistor cools, so we
    // require a short run of "out" readings before declaring victory.
    int _below_count = 0;
};


#endif // EXTINGUISH_H

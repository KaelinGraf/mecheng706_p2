#ifndef HAL_SIM_CLOCK_H
#define HAL_SIM_CLOCK_H

#include "iclock.h"

// Time source driven by the sim. Doesn't tick by itself - the sim host
// advances `now_ms` and `now_us` explicitly via `advance(...)` or `set(...)`.
// This makes the controller's behaviour deterministic at any tick rate.
class SimClock : public IClock {
public:
    SimClock() : _now_us(0) {}

    unsigned long now_ms() override { return (unsigned long)(_now_us / 1000UL); }
    unsigned long now_us() override { return _now_us; }

    void advance_us(unsigned long delta_us) { _now_us += delta_us; }
    void advance_ms(unsigned long delta_ms) { _now_us += (delta_ms * 1000UL); }

    void set_us(unsigned long t_us) { _now_us = t_us; }

private:
    unsigned long _now_us;
};

#endif

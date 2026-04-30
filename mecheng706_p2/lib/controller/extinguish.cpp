#include "extinguish.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// EXTINGUISH state
// ---------------------------------------------------------------------------
// Robot is parked within ~EXTINGUISH_RANGE_CM of the fire and aimed at it.
// Hold position, soft-start the fan, watch for the LED to drop. Increment the
// FireFighter's fire counter on completion; transition to SEARCH (1st fire)
// or STOPPED (2nd fire).
// ---------------------------------------------------------------------------

static const int FIRE_OUT_DEBOUNCE = 10;

static inline unsigned long _now_ms(FireFighter* ff) {
    return ff->clock() ? ff->clock()->now_ms() : 0UL;
}

void Extinguish::begin() {
    FireFighter* ff = firefighter_;
    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    ff->_fan->on();
    _start_millis = _now_ms(ff);
    _below_count = 0;
    ff->println("EXTINGUISH: fan ON");
}

void Extinguish::end() {
    firefighter_->_fan->off();
    firefighter_->println("EXTINGUISH: fan OFF");
}

void Extinguish::poll() {
    FireFighter* ff = firefighter_;

    bool timed_out = (_now_ms(ff) - _start_millis) >= FAN_HARD_TIMEOUT_MS;

    if (ff->_fire_bank->allBelow(FIRE_OUT_V)) {
        _below_count++;
    } else {
        _below_count = 0;
    }
    bool fire_out = _below_count >= FIRE_OUT_DEBOUNCE;

    if (fire_out || timed_out) {
        ff->_fan->off();
        ff->noteFireExtinguished();
        ff->print("EXTINGUISH: done (");
        ff->print(fire_out ? "led-out" : "timeout");
        ff->print(") fires=");
        ff->println(ff->firesExtinguished());

        if (ff->firesExtinguished() >= 2) {
            ff->switchState(State::STOPPED);
        } else {
            ff->_fire_bank->reset();
            ff->switchState(State::SEARCH);
        }
    }
}

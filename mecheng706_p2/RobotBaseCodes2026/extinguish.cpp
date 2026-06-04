#include "Arduino.h"
#include "extinguish.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// EXTINGUISH state
// ---------------------------------------------------------------------------
// Robot is parked within ~EXTINGUISH_RANGE_CM of the fire and aimed at it.
// We:
//   1. Stop all wheel motion (we must NOT drift while the fan blows; the
//      brief specifies the fire is extinguished while inside a 20 cm radius
//      of its centre).
//   2. Soft-start the fan (Fan::on() ramps the MOSFET gate to avoid an
//      inrush brown-out on the 5V regulator).
//   3. Poll every loop:
//        a. If FireBank::allBelow(FIRE_OUT_V) for FIRE_OUT_DEBOUNCE polls in
//           a row, the LED has gone dark - thermistor has done its job.
//           Cut the fan and exit early.
//        b. If we hit FAN_MAX_MS without the LED dropping, the brief allows
//           the demonstrator to manually switch off the LED at 10 s; we
//           still cut the fan and count it as extinguished (brief says
//           "the light will be turned off manually at 10 sec").
//   4. Increment the FireFighter's fire counter:
//        - 1 fire out  -> back to SEARCH for the second one.
//        - 2 fires out -> STOPPED (brief: "robot must cease all movement
//                          immediately after extinguishing the second fire").
// ---------------------------------------------------------------------------

// Number of consecutive "all below threshold" polls before we declare the
// fire out. With a 10 ms loop period this is ~100 ms of darkness.
static const int FIRE_OUT_DEBOUNCE = 5;

void Extinguish::begin() {
    FireFighter* ff = firefighter_;

    // Hold position - critical that the robot does NOT drift outside the
    // 20 cm extinguish radius mid-blow.
    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);

    // Soft-start the fan. on() blocks for ~250 ms during the ramp; that's
    // acceptable because no other state machine work needs to happen while
    // we're parked.
    ff->_fan->on();

    _start_millis = millis();
    _below_count  = 0;
    ff->println("EXTINGUISH: fan ON");
}

void Extinguish::end() {
    // Idempotent: safe even if begin() was bypassed for some reason.
    firefighter_->_fan->off();
    firefighter_->println("EXTINGUISH: fan OFF and moving back");

    // pushing in a few PT updates for MA
    firefighter_->_fire_bank->update();
    firefighter_->_fire_bank->update();

    if (firefighter_->firesExtinguished() < 2) {
        firefighter_->_motors->writeAllMotors(50, 0, 0);
        delay(300);
    }
    firefighter_->_motors->writeAllMotors(0, 0, 0);
}

void Extinguish::poll() {
    FireFighter* ff = firefighter_;

    // -------------------------------------------------------------------
    // 1. Hard timeout: 10 s per the brief. Whether or not the LED actually
    //    dropped, we move on (the demonstrator turns the LED off if not).
    // -------------------------------------------------------------------
    bool timed_out = (millis() - _start_millis) >= FAN_HARD_TIMEOUT_MS;

    // -------------------------------------------------------------------
    // 2. LED-out detection. The thermistor on the fire cylinder cuts the
    //    LED once the temperature drops; once the LED is off, all four
    //    phototransistors should read at or below the ambient floor.
    // -------------------------------------------------------------------
    if (ff->_fire_bank->allBelow(FIRE_OUT_V)) {
        _below_count++;
    } else {
        _below_count = 0;
    }
    bool fire_out = _below_count >= FIRE_OUT_DEBOUNCE;

    // -------------------------------------------------------------------
    // 3. Exit. Either condition counts as "this fire dealt with".
    // -------------------------------------------------------------------
    if (fire_out || timed_out) {
        ff->_fan->off();
        ff->noteFireExtinguished();
        ff->print("EXTINGUISH: done (");
        ff->print(fire_out ? "led-out" : "timeout");
        ff->print(") fires=");
        ff->print(ff->firesExtinguished());
        ff->print(" time=");
        ff->println(millis() - _start_millis);

        if (ff->firesExtinguished() >= 2) {
            ff->println("Complete");
            ff->switchState(State::STOPPED);
        } else {
            // Reset the EWMA on the photo bank so the lingering glow from
            // the just-extinguished fire doesn't immediately re-trigger
            // APPROACH against itself.
            ff->println("Next Fire");
            ff->_fire_bank->reset();
            // Behaviour 2: re-localise the next fire with a 360 spin-scan
            // before resuming the normal drive + turret pan-scan search.
            ff->switchState(State::SPIN_SCAN);
        }
    }
}

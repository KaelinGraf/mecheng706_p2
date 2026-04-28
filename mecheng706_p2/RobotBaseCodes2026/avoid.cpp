#include "Arduino.h"
#include <math.h>
#include "avoid.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// AVOID state
// ---------------------------------------------------------------------------
// Strategy:
//   1. On entry, decide which side has more clearance using the front-left
//      and front-right long-range IRs. Strafe in that direction (mecanum
//      vx) while creeping forward at a reduced speed - this slides past the
//      cylindrical obstacle rather than turning around it, which is faster.
//   2. After at least AVOID_STRAFE_MS have elapsed, start checking whether
//      the forward arc is clear (ultrasonic + both front IRs above
//      OBSTACLE_CLEAR_CM). When it is, exit back to SEARCH (or APPROACH if a
//      fire bearing was being chased before AVOID).
//   3. If still not clear after a generous timeout, rotate in place toward
//      the more-clear side instead. This handles corners where pure strafing
//      would just slam into the wall.
//
// Movement detection: between two consecutive AVOID entries, the same
// obstacle may also be the dynamic one mentioned in the brief. We don't try
// to dodge it cleverly here - the strafe-then-resume loop is robust to
// movement because if the obstacle keeps following, we re-enter AVOID with
// fresh clearance data each time.
// ---------------------------------------------------------------------------

// Total time we'll spend in AVOID before giving up the lateral strategy and
// falling back to rotation-in-place.
static const unsigned long AVOID_TIMEOUT_MS = 3000;

// Forward bias while strafing: we want to slide past the obstacle, not just
// move sideways. Small forward speed is plenty for a 10 cm dia. cylinder.
static const float AVOID_FORWARD_SPEED = 50.0f;

void Avoid::begin() {
    // No upstream context (e.g. entered from SEARCH). Resume to SEARCH when
    // clear.
    _resume_bearing = NAN;
    _resume_to_approach = false;

    FireFighter* ff = firefighter_;

    // Decide strafe direction up-front: prefer the side reading further
    // away (i.e. more clearance). If both are blocked, pick whichever has a
    // *valid* range; if neither, default to right (+1).
    float lf = ff->_front_left_ir->getAvg();
    float rf = ff->_front_right_ir->getAvg();
    bool lf_valid = (lf > 0.0f);
    bool rf_valid = (rf > 0.0f);

    if (lf_valid && rf_valid) {
        _strafe_sign = (lf > rf) ? -1 : 1;      // -1 strafe LEFT, +1 strafe RIGHT
    } else if (lf_valid) {
        _strafe_sign = -1;                       // right is "out of range" = far
    } else if (rf_valid) {
        _strafe_sign = +1;
    } else {
        _strafe_sign = +1;                       // default
    }

    _start_ms = millis();
    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    ff->print("AVOID: strafe ");
    ff->println(_strafe_sign > 0 ? "RIGHT" : "LEFT");
}

void Avoid::begin(StateData data) {
    // Entered from APPROACH: carry the bearing so we can resume the chase.
    begin();
    if (!isnan(data.param)) {
        _resume_bearing = data.param;
        _resume_to_approach = true;
    }
}

void Avoid::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Avoid::poll() {
    FireFighter* ff = firefighter_;
    unsigned long elapsed = millis() - _start_ms;

    // -------------------------------------------------------------------
    // Higher-priority interrupt: if a fire is now visible and we weren't
    // already chasing one, jump straight to APPROACH. Important because
    // AVOID can slide us past an obstacle and reveal a fire in the next
    // cell.
    // -------------------------------------------------------------------
    if (!_resume_to_approach && ff->_fire_bank->anyDetection(FIRE_DETECT_V)) {
        bool valid = false;
        float bearing = ff->_fire_bank->estimateBearing(&valid, FIRE_DETECT_V);
        if (valid) {
            ff->println("AVOID -> APPROACH (fire detected mid-avoid)");
            StateData payload = { bearing, false, false };
            ff->switchState(State::APPROACH, payload);
            return;
        }
    }

    // -------------------------------------------------------------------
    // Re-check forward arc once we've strafed long enough to mean it.
    // Use the wider OBSTACLE_CLEAR_CM threshold for hysteresis vs. the
    // OBSTACLE_TRIGGER_CM that put us into AVOID.
    // -------------------------------------------------------------------
    if (elapsed > AVOID_STRAFE_MS) {
        float us = ff->_ultrasonic->getAvg();
        float lf = ff->_front_left_ir->getAvg();
        float rf = ff->_front_right_ir->getAvg();

        auto clear = [](float v, float thr) {
            // -1 (out of range) = clear; positive readings must exceed thr.
            return (v < 0.0f) || (v >= thr);
        };

        if (clear(us, OBSTACLE_CLEAR_CM) &&
            clear(lf, OBSTACLE_CLEAR_CM) &&
            clear(rf, OBSTACLE_CLEAR_CM)) {
            if (_resume_to_approach) {
                ff->println("AVOID -> APPROACH (clear, resuming bearing)");
                StateData payload = { _resume_bearing, false, false };
                ff->switchState(State::APPROACH, payload);
            } else {
                ff->println("AVOID -> SEARCH (clear)");
                ff->switchState(State::SEARCH);
            }
            return;
        }
    }

    // -------------------------------------------------------------------
    // Fallback: too long in AVOID? The straight strafe isn't getting us
    // free (likely a corner). Rotate in place toward the strafe side.
    // -------------------------------------------------------------------
    if (elapsed > AVOID_TIMEOUT_MS) {
        // vtheta sign convention used elsewhere: +ve = turn LEFT.
        // _strafe_sign = +1 means we'd been strafing right -> rotate right
        // (vtheta < 0). _strafe_sign = -1 -> rotate left (vtheta > 0).
        float vtheta = -SEARCH_TURN_SPEED * _strafe_sign;
        ff->_motors->writeAllMotors(0.0f, 0.0f, vtheta);
        return;
    }

    // -------------------------------------------------------------------
    // Default: strafe + creep forward.
    // -------------------------------------------------------------------
    float vx = AVOID_STRAFE_SPEED * _strafe_sign;
    float vy = AVOID_FORWARD_SPEED;
    ff->_motors->writeAllMotors(vx, vy, 0.0f);
}

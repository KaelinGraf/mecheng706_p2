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
// ---------------------------------------------------------------------------

static const unsigned long AVOID_TIMEOUT_MS = 3000;
static const float AVOID_FORWARD_SPEED = 50.0f;

static inline unsigned long _now_ms(FireFighter* ff) {
    return ff->clock() ? ff->clock()->now_ms() : 0UL;
}

void Avoid::begin() {
    _resume_bearing = NAN;
    _resume_to_approach = false;

    FireFighter* ff = firefighter_;

    float lf = ff->_front_left_ir->getAvg();
    float rf = ff->_front_right_ir->getAvg();
    bool lf_valid = (lf > 0.0f);
    bool rf_valid = (rf > 0.0f);

    if (lf_valid && rf_valid) {
        _strafe_sign = (lf > rf) ? -1 : 1;
    } else if (lf_valid) {
        _strafe_sign = -1;
    } else if (rf_valid) {
        _strafe_sign = +1;
    } else {
        _strafe_sign = +1;
    }

    _start_ms = _now_ms(ff);
    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    ff->print("AVOID: strafe ");
    ff->println(_strafe_sign > 0 ? "RIGHT" : "LEFT");
}

void Avoid::begin(StateData data) {
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
    unsigned long elapsed = _now_ms(ff) - _start_ms;

    // Higher-priority interrupt: if a fire is now visible and we weren't
    // already chasing one, jump straight to APPROACH.
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

    // Re-check forward arc once we've strafed long enough to mean it.
    if (elapsed > AVOID_STRAFE_MS) {
        float us = ff->_ultrasonic->getAvg();
        float lf = ff->_front_left_ir->getAvg();
        float rf = ff->_front_right_ir->getAvg();

        auto clear = [](float v, float thr) {
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

    // Fallback: rotate in place if we can't strafe past.
    if (elapsed > AVOID_TIMEOUT_MS) {
        float vtheta = -SEARCH_TURN_SPEED * _strafe_sign;
        ff->_motors->writeAllMotors(0.0f, 0.0f, vtheta);
        return;
    }

    // Default: strafe + creep forward.
    float vx = AVOID_STRAFE_SPEED * _strafe_sign;
    float vy = AVOID_FORWARD_SPEED;
    ff->_motors->writeAllMotors(vx, vy, 0.0f);
}

#include <math.h>
#include "approach.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// APPROACH state
// ---------------------------------------------------------------------------
// Goal: drive the robot's centre to within EXTINGUISH_RANGE_CM of the fire,
// using phototransistor bearing as feedback.
// ---------------------------------------------------------------------------

static const float APPROACH_KP_TURN = 120.0f;
static const float APPROACH_FWD = 90.0f;
static const float BEARING_PIVOT_THRESH_RAD = 0.6f;
static const unsigned long LOST_FIRE_MS = 1500;

static inline unsigned long _now_ms(FireFighter* ff) {
    return ff->clock() ? ff->clock()->now_ms() : 0UL;
}

void Approach::begin() {
    _bearing = 0.0f;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    _last_seen_ms = _now_ms(firefighter_);
}

void Approach::begin(StateData data) {
    _bearing = data.param;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    _last_seen_ms = _now_ms(firefighter_);
}

void Approach::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Approach::poll() {
    FireFighter* ff = firefighter_;

    bool valid = false;
    float new_bearing = ff->_fire_bank->estimateBearing(&valid, FIRE_DETECT_V);
    if (valid) {
        _bearing = new_bearing;
        _last_seen_ms = _now_ms(ff);
    } else if (_now_ms(ff) - _last_seen_ms > LOST_FIRE_MS) {
        ff->println("APPROACH -> SEARCH (fire lost)");
        ff->switchState(State::SEARCH);
        return;
    }

    float us = ff->_ultrasonic->getAvg();
    float lf = ff->_front_left_ir->getAvg();
    float rf = ff->_front_right_ir->getAvg();

    auto inside = [](float v, float thr) {
        return (v > 0.0f) && (v <= thr);
    };

    bool close_front = inside(us, EXTINGUISH_RANGE_CM) ||
                       inside(lf, EXTINGUISH_RANGE_CM) ||
                       inside(rf, EXTINGUISH_RANGE_CM);
    bool aimed = fabsf(_bearing) < 0.35f;

    if (close_front && aimed) {
        ff->println("APPROACH -> EXTINGUISH (within range, aimed)");
        ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
        ff->switchState(State::EXTINGUISH);
        return;
    }

    if ((inside(us, OBSTACLE_TRIGGER_CM) ||
         inside(lf, OBSTACLE_TRIGGER_CM) ||
         inside(rf, OBSTACLE_TRIGGER_CM)) && !close_front) {
        ff->println("APPROACH -> AVOID (obstacle blocking line to fire)");
        StateData payload = { _bearing, false, false };
        ff->switchState(State::AVOID, payload);
        return;
    }

    float vtheta = APPROACH_KP_TURN * _bearing;

    float fwd_scale;
    if (fabsf(_bearing) > BEARING_PIVOT_THRESH_RAD) {
        fwd_scale = 0.0f;
    } else {
        fwd_scale = cosf(_bearing);
    }
    float vy = APPROACH_FWD * fwd_scale;

    ff->_motors->writeAllMotors(0.0f, vy, vtheta);
}

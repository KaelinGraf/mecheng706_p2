#include "Arduino.h"
#include <math.h>
#include "approach.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// APPROACH state
// ---------------------------------------------------------------------------
// Goal: drive the robot's centre to within EXTINGUISH_RANGE_CM of the fire,
// using phototransistor bearing as feedback.
//
// Control: at every poll we re-estimate the bearing-to-fire from the four
// phototransistors. We treat that bearing as the heading error and apply a
// proportional turn rate (vtheta), while driving forward at a speed that
// scales with cos(error) so the robot doesn't try to drive into a fire that
// is currently to its side - it pivots first.
//
// Transitions:
//   APPROACH -> EXTINGUISH : ultrasonic OR both front IRs read inside
//                            EXTINGUISH_RANGE_CM AND the bearing error is
//                            small (we're really facing the fire, not a
//                            piece of obstacle in front of it).
//   APPROACH -> AVOID      : a non-fire obstacle blocks us closer than the
//                            extinguish range. The fire bearing is handed
//                            off via StateData so AVOID can resume APPROACH
//                            once the obstacle is cleared.
//   APPROACH -> SEARCH     : we lose the fire detection for too long
//                            (LOST_FIRE_MS) - go back to wandering.
// ---------------------------------------------------------------------------

// Proportional steering gain. Bearing is in radians; multiplying by ~120
// puts a 30 deg error at vtheta ~63, which is gentle but visible.
static const float APPROACH_KP_TURN = 120.0f;

// Forward cruise during APPROACH. Slightly slower than SEARCH for finer
// closing-in.
static const float APPROACH_FWD = 90.0f;

// If the bearing error is bigger than this, slow forward speed to ~0 and
// let the rotation finish first. Avoids "scything" arcs around the fire.
static const float BEARING_PIVOT_THRESH_RAD = 0.6f;   // ~34 deg

// How long we'll keep approaching after the fire signal disappears before
// giving up and returning to SEARCH. The phototransistor signal can drop
// out briefly at close range when an obstacle obscures the LED, so we want
// some grace period.
static const unsigned long LOST_FIRE_MS = 1500;

void Approach::begin() {
    // No bearing supplied - default to "ahead" and re-estimate on first poll.
    _bearing = 0.0f;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    _last_seen_ms = millis();
}

void Approach::begin(StateData data) {
    _bearing = data.param;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    _last_seen_ms = millis();
}

void Approach::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Approach::poll() {
    FireFighter* ff = firefighter_;

    // -------------------------------------------------------------------
    // 1. Re-estimate bearing. If still detected, refresh _bearing and the
    //    "last seen" timestamp; otherwise keep coasting on the previous
    //    bearing for up to LOST_FIRE_MS.
    // -------------------------------------------------------------------
    bool valid = false;
    float new_bearing = ff->_fire_bank->estimateBearing(&valid, FIRE_DETECT_V);
    if (valid) {
        _bearing = new_bearing;
        _last_seen_ms = millis();
    } else if (millis() - _last_seen_ms > LOST_FIRE_MS) {
        ff->println("APPROACH -> SEARCH (fire lost)");
        ff->switchState(State::SEARCH);
        return;
    }

    // -------------------------------------------------------------------
    // 2. Pull range readings. We need both the ultrasonic (precise but
    //    narrow) and the front IRs (wider FOV, redundancy in case the
    //    ultrasonic mis-fires off the cylinder curvature).
    // -------------------------------------------------------------------
    float us = ff->_ultrasonic->getAvg();
    float lf = ff->_front_left_ir->getAvg();
    float rf = ff->_front_right_ir->getAvg();

    auto inside = [](float v, float thr) {
        return (v > 0.0f) && (v <= thr);
    };

    // -------------------------------------------------------------------
    // 3. EXTINGUISH gate. We're "at" the fire when:
    //    a) something is very close in front (ultrasonic OR a front IR
    //       reads inside EXTINGUISH_RANGE_CM), AND
    //    b) the photo bearing is small (we're actually facing the fire,
    //       not a different obstacle).
    // -------------------------------------------------------------------
    bool close_front = inside(us, EXTINGUISH_RANGE_CM) ||
                       inside(lf, EXTINGUISH_RANGE_CM) ||
                       inside(rf, EXTINGUISH_RANGE_CM);
    bool aimed       = fabsf(_bearing) < 0.35f;        // ~20 deg

    if (close_front && aimed) {
        ff->println("APPROACH -> EXTINGUISH (within range, aimed)");
        ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
        ff->switchState(State::EXTINGUISH);
        return;
    }

    // -------------------------------------------------------------------
    // 4. Obstacle check (non-fire). If a front IR or ultrasonic is firing
    //    inside OBSTACLE_TRIGGER_CM but we're NOT aimed at the fire, an
    //    obstacle is in our way. Hand off to AVOID, preserving the bearing
    //    so we can resume on the other side.
    // -------------------------------------------------------------------
    if ((inside(us, OBSTACLE_TRIGGER_CM) ||
         inside(lf, OBSTACLE_TRIGGER_CM) ||
         inside(rf, OBSTACLE_TRIGGER_CM)) && !close_front) {
        ff->println("APPROACH -> AVOID (obstacle blocking line to fire)");
        StateData payload = { _bearing, false, false };
        ff->switchState(State::AVOID, payload);
        return;
    }

    // -------------------------------------------------------------------
    // 5. Drive: proportional turn on bearing, forward speed gated by error.
    //    vtheta = +ve -> turn LEFT (toward +bearing) by convention.
    // -------------------------------------------------------------------
    float vtheta = APPROACH_KP_TURN * _bearing;

    float fwd_scale;
    if (fabsf(_bearing) > BEARING_PIVOT_THRESH_RAD) {
        fwd_scale = 0.0f;                              // pivot only
    } else {
        fwd_scale = cosf(_bearing);                    // 1 dead-ahead, 0 at 90 deg
    }
    float vy = APPROACH_FWD * fwd_scale;

    ff->_motors->writeAllMotors(0.0f, vy, vtheta);
}

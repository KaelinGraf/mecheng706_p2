#include <math.h>
#include "search.h"
#include "firefighter.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// SEARCH state
// ---------------------------------------------------------------------------
// Behaviour-based subsumption layers (top wins):
//
//   3. FIRE-DETECT  : any phototransistor reading >= FIRE_DETECT_V triggers a
//                     transition to APPROACH carrying the estimated bearing.
//   2. OBSTACLE-AVOID: if the front sensor cluster (HC-SR04 + 2x long-range
//                     IR + 2x short-range IR over the rear wheels) reports
//                     anything inside OBSTACLE_TRIGGER_CM, hand off to AVOID.
//   1. WALL-FOLLOW  : the rear short-range IRs at +/-40 deg also catch side
//                     walls; if either fires inside WALL_FOLLOW_CM, bias the
//                     turn rate away from that side.
//   0. CRUISE       : drive straight forward at SEARCH_FORWARD_SPEED.
// ---------------------------------------------------------------------------

static const unsigned long NUDGE_PERIOD_MS = 4000;
static const unsigned long NUDGE_LEN_MS    = 350;
static const float        NUDGE_TURN       = 60.0f;

static unsigned long _last_nudge = 0;
static int           _nudge_dir  = 1;
static bool          _nudge_active = false;
static unsigned long _nudge_start = 0;

static inline unsigned long _now_ms(FireFighter* ff) {
    return ff->clock() ? ff->clock()->now_ms() : 0UL;
}

void Search::begin() {
    firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
    _last_nudge = _now_ms(firefighter_);
    _nudge_active = false;
}

void Search::end() {
    // Nothing to do; transitions handle their own setup.
}

void Search::poll() {
    FireFighter* ff = firefighter_;

    // Layer 3: fire detection (highest priority)
    if (ff->_fire_bank->anyDetection(FIRE_DETECT_V)) {
        bool valid = false;
        float bearing = ff->_fire_bank->estimateBearing(&valid, FIRE_DETECT_V);
        if (valid) {
            ff->println("SEARCH -> APPROACH (fire detected)");
            StateData payload = { bearing, false, false };
            ff->switchState(State::APPROACH, payload);
            return;
        }
    }

    float us_cm  = ff->_ultrasonic->getAvg();
    float lf_cm  = ff->_front_left_ir->getAvg();
    float rf_cm  = ff->_front_right_ir->getAvg();
    float lr_cm  = ff->_rear_left_ir->getAvg();
    float rr_cm  = ff->_rear_right_ir->getAvg();

    auto blocked = [](float v, float thr) {
        return (v > 0.0f) && (v < thr);
    };

    // Layer 2: obstacle avoidance
    if (blocked(us_cm, OBSTACLE_TRIGGER_CM) ||
        blocked(lf_cm, OBSTACLE_TRIGGER_CM) ||
        blocked(rf_cm, OBSTACLE_TRIGGER_CM)) {
        ff->println("SEARCH -> AVOID (obstacle ahead)");
        ff->switchState(State::AVOID);
        return;
    }

    // Layer 1: wall-follow / side-clearance bias
    float vtheta = 0.0f;
    if (blocked(lr_cm, WALL_FOLLOW_CM)) {
        vtheta -= SEARCH_TURN_SPEED * (1.0f - lr_cm / WALL_FOLLOW_CM);
    }
    if (blocked(rr_cm, WALL_FOLLOW_CM)) {
        vtheta += SEARCH_TURN_SPEED * (1.0f - rr_cm / WALL_FOLLOW_CM);
    }

    // Periodic exploratory nudge
    unsigned long now = _now_ms(ff);
    if (!_nudge_active && (now - _last_nudge) > NUDGE_PERIOD_MS) {
        _nudge_active = true;
        _nudge_start  = now;
        _nudge_dir    = -_nudge_dir;
    }
    if (_nudge_active) {
        if ((now - _nudge_start) < NUDGE_LEN_MS) {
            vtheta += NUDGE_TURN * _nudge_dir;
        } else {
            _nudge_active = false;
            _last_nudge   = now;
        }
    }

    // Layer 0: cruise
    ff->_motors->writeAllMotors(0.0f, SEARCH_FORWARD_SPEED, vtheta);
}

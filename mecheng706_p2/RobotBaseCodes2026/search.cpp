#include "Arduino.h"
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
//
// The wander pattern is intentionally simple: drive forward, lean off walls,
// let AVOID handle obstacles. The 1200x2500 mm table is small enough that a
// straight-line cruise + obstacle reaction sweeps the area within a minute or
// two without needing a sophisticated coverage planner.
// ---------------------------------------------------------------------------

// Periodic random heading nudge so the robot doesn't lock into a hallway.
// Every NUDGE_PERIOD_MS we add a small left/right bias to vtheta for
// NUDGE_LEN_MS to break symmetry without slowing the search.
static const unsigned long NUDGE_PERIOD_MS = 4000;
static const unsigned long NUDGE_LEN_MS    = 350;
static const float        NUDGE_TURN       = 60.0f;

static unsigned long _last_nudge = 0;
static int           _nudge_dir  = 1;          // +1 = left, -1 = right
static bool          _nudge_active = false;
static unsigned long _nudge_start = 0;

void Search::begin() {
    firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
    _last_nudge = millis();
    _nudge_active = false;
}

void Search::end() {
    // Nothing to do; transitions handle their own setup.
}

void Search::poll() {
    FireFighter* ff = firefighter_;

    // -------------------------------------------------------------------
    // Layer 3: fire detection (highest priority)
    // -------------------------------------------------------------------
    // FireBank::update() is run from FireFighter::pollState(); just query.
    if (ff->_fire_bank->anyDetection(FIRE_DETECT_V)) {
        bool valid = false;
        float bearing = ff->_fire_bank->estimateBearing(&valid, FIRE_DETECT_V);
        if (valid) {
            ff->println("SEARCH -> APPROACH (fire detected)");
            // Pass bearing through StateData so APPROACH starts steering
            // toward the right hemisphere on its very first poll.
            StateData payload = { bearing, false, false };
            ff->switchState(State::APPROACH, payload);
            return;
        }
    }

    // -------------------------------------------------------------------
    // Pull median-filtered ranges. getAvg() returns -1 when no valid sample
    // is in the buffer, which we treat as "out of range" (i.e. clear).
    // -------------------------------------------------------------------
    float us_cm  = ff->_ultrasonic->getAvg();
    float lf_cm  = ff->_front_left_ir->getAvg();
    float rf_cm  = ff->_front_right_ir->getAvg();
    float lr_cm  = ff->_rear_left_ir->getAvg();
    float rr_cm  = ff->_rear_right_ir->getAvg();

    // Helper lambda: a reading "blocks" the path if it is positive (valid)
    // and below the threshold. The -1.0 sentinel returned for out-of-range
    // values must NOT count as "blocked".
    auto blocked = [](float v, float thr) {
        return (v > 0.0f) && (v < thr);
    };

    // -------------------------------------------------------------------
    // Layer 2: obstacle avoidance
    // -------------------------------------------------------------------
    // Anything inside OBSTACLE_TRIGGER_CM on the *forward* arc forces AVOID.
    // We only consider the front-facing sensors here - the rear-mounted SRs
    // are angled forward but are mostly useful for side-wall bias (Layer 1)
    // because they alarm on the body sides during turns.
    if (blocked(us_cm, OBSTACLE_TRIGGER_CM) ||
        blocked(lf_cm, OBSTACLE_TRIGGER_CM) ||
        blocked(rf_cm, OBSTACLE_TRIGGER_CM)) {
        ff->println("SEARCH -> AVOID (obstacle ahead)");
        ff->switchState(State::AVOID);
        return;
    }

    // -------------------------------------------------------------------
    // Layer 1: wall-follow / side-clearance bias
    // -------------------------------------------------------------------
    // The +/-40 deg rear-wheel IRs cover the lateral-forward zone. If one
    // sees something close, we're tracking parallel to a wall and need to
    // peel away. Use a soft proportional bias rather than a hard turn.
    float vtheta = 0.0f;
    if (blocked(lr_cm, WALL_FOLLOW_CM)) {
        // Wall on left rear quarter -> turn right (bearing convention:
        // negative vtheta is "right" if vtheta>0 turns left).
        vtheta -= SEARCH_TURN_SPEED * (1.0f - lr_cm / WALL_FOLLOW_CM);
    }
    if (blocked(rr_cm, WALL_FOLLOW_CM)) {
        // Wall on right rear quarter -> turn left.
        vtheta += SEARCH_TURN_SPEED * (1.0f - rr_cm / WALL_FOLLOW_CM);
    }

    // -------------------------------------------------------------------
    // Periodic exploratory nudge
    // -------------------------------------------------------------------
    // Without this, the robot would keep driving in a straight line until
    // something blocks it. A small alternating heading nudge breaks that.
    unsigned long now = millis();
    if (!_nudge_active && (now - _last_nudge) > NUDGE_PERIOD_MS) {
        _nudge_active = true;
        _nudge_start  = now;
        _nudge_dir    = -_nudge_dir;            // alternate L/R
    }
    if (_nudge_active) {
        if ((now - _nudge_start) < NUDGE_LEN_MS) {
            vtheta += NUDGE_TURN * _nudge_dir;
        } else {
            _nudge_active = false;
            _last_nudge   = now;
        }
    }

    // -------------------------------------------------------------------
    // Layer 0: cruise
    // -------------------------------------------------------------------
    // Mecanum: writeAllMotors(vx, vy, vtheta). vy>0 = forward.
    ff->_motors->writeAllMotors(0.0f, SEARCH_FORWARD_SPEED, vtheta);
}

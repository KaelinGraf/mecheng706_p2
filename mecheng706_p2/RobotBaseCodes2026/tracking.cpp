#include "Arduino.h"
#include <math.h>
#include "tracking.h"
#include "firefighter.h"
#include "mappings.h"

namespace {
static const unsigned long LOST_FIRE_MS = 1500;
static const unsigned long AVOID_STRAFE_MS = 900;
static const unsigned long AVOID_TIMEOUT_MS = 3000;
static const unsigned long NUDGE_PERIOD_MS = 4000;
static const unsigned long NUDGE_LEN_MS = 350;
static const float NUDGE_TURN = 60.0f;
static const float BEARING_PIVOT_THRESH_RAD = 0.6f;
}

Tracking::Tracking(FireFighter *firefighter)
    : firefighter_(firefighter),
      active_behavior_(BehaviorNS::SearchBehaviour::FIND_FIRE),
      bearing_(0.0f),
      resume_bearing_(0.0f),
      resume_to_move_(false),
      strafe_sign_(1),
      behavior_start_ms_(0),
      last_seen_ms_(0),
      last_nudge_ms_(0),
      nudge_start_ms_(0),
      nudge_active_(false),
      nudge_dir_(1) {}

void Tracking::enterFindFire() {
    firefighter_->println("Find Fire");
    active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
    behavior_start_ms_ = millis();
    resume_to_move_ = false;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    last_nudge_ms_ = behavior_start_ms_;
    nudge_active_ = false;
}

void Tracking::enterAvoid(bool resume_to_move) {
    firefighter_->println("Avoiding");
    active_behavior_ = BehaviorNS::SearchBehaviour::AVOID;
    behavior_start_ms_ = millis();
    resume_to_move_ = resume_to_move;

    float lf_cm = firefighter_->_front_left_ir->getAvg();
    float rf_cm = firefighter_->_front_right_ir->getAvg();
    bool lf_valid = (lf_cm > 0.0f);
    bool rf_valid = (rf_cm > 0.0f);

    firefighter_->_motors->writeAllMotors(-REVERSE_SPEED, 0.0f, 0.0f);

    if((lf_cm > 10.0) && (rf_cm)){
        enterFindFire();
    }
}

void Tracking::enterMoveToFire(float bearing) {
    active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
    behavior_start_ms_ = millis();
    bearing_ = bearing;
    last_seen_ms_ = behavior_start_ms_;
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Tracking::begin() {
    bearing_ = 0.0f;
    resume_bearing_ = 0.0f;
    resume_to_move_ = false;
    enterFindFire();
}

void Tracking::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Tracking::poll() {
    FireFighter *ff = firefighter_;
    unsigned long now = millis();

    float us_cm = ff->_ultrasonic->getAvg();
    float lf_cm = ff->_front_left_ir->getAvg();
    float rf_cm = ff->_front_right_ir->getAvg();
    float lr_cm = ff->_rear_left_ir->getAvg();
    float rr_cm = ff->_rear_right_ir->getAvg();

    auto blocked = [](float v, float thr) {
        return (v > 0.0f) && (v < thr);
    };

    bool fire_detected = ff->_fire_bank->anyDetection(FIRE_DETECT_V);
    bool fire_valid = false;
    float fire_bearing = bearing_;
    if (fire_detected) {
        fire_bearing = ff->_fire_bank->estimateBearing(&fire_valid, FIRE_DETECT_V);
    }

    bool obstacle_ahead = blocked(us_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(lf_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(rf_cm, OBSTACLE_TRIGGER_CM);

    bool close_front = blocked(us_cm, EXTINGUISH_RANGE_CM) ||
                       blocked(lf_cm, EXTINGUISH_RANGE_CM) ||
                       blocked(rf_cm, EXTINGUISH_RANGE_CM);

    bool aimed = fabsf(bearing_) < 0.35f;

<<<<<<< Updated upstream
=======
    firefighter_->print("Obsatcle: ");
    firefighter_->println(obstacle_ahead);
    firefighter_->print("In Front of Fire: ");
    firefighter_->println(close_front);
    firefighter_->print("Aimed: ");
    firefighter_->println(aimed);
    if(obstacle_ahead){
        enterAvoid(false);
    }


>>>>>>> Stashed changes
    if (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
        if (fire_valid) {
            bearing_ = fire_bearing;
            last_seen_ms_ = now;
        } else if ((now - last_seen_ms_) > LOST_FIRE_MS) {
            ff->println("SEARCH: fire lost -> FIND_FIRE");
            enterFindFire();
            return;
        }

        if (close_front && aimed) {
            ff->println("SEARCH: fire within extinguish range");
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->switchState(State::EXTINGUISH);
            return;
        }

        if (obstacle_ahead && !close_front) {
            ff->println("SEARCH: obstacle blocks fire -> AVOID");
            resume_bearing_ = bearing_;
            enterAvoid(true);
            return;
        }

        float vtheta = APPROACH_TURN_GAIN * bearing_;
        if (vtheta > APPROACH_MAX_TURN) vtheta = APPROACH_MAX_TURN;
        if (vtheta < -APPROACH_MAX_TURN) vtheta = -APPROACH_MAX_TURN;

        float vy = (fabsf(bearing_) > BEARING_PIVOT_THRESH_RAD)
                       ? 0.0f
                       : (APPROACH_FORWARD_SPEED * cosf(bearing_));
        ff->_motors->writeAllMotors(0.0f, vy, vtheta);
        return;
    }

    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID) {
        if (fire_valid) {
            resume_bearing_ = fire_bearing;
            resume_to_move_ = true;
            bearing_ = fire_bearing;
            last_seen_ms_ = now;
        }

        bool clear = !obstacle_ahead &&
                     ((us_cm < 0.0f) || (us_cm >= OBSTACLE_CLEAR_CM)) &&
                     ((lf_cm < 0.0f) || (lf_cm >= OBSTACLE_CLEAR_CM)) &&
                     ((rf_cm < 0.0f) || (rf_cm >= OBSTACLE_CLEAR_CM));

        unsigned long elapsed = now - behavior_start_ms_;
        if (clear && elapsed > AVOID_STRAFE_MS) {
            ff->println("SEARCH: obstacle cleared");
            if (resume_to_move_) {
                enterMoveToFire(resume_bearing_);
            } else {
                enterFindFire();
            }
            return;
        }

        if (elapsed > AVOID_TIMEOUT_MS) {
            float vtheta = -SEARCH_TURN_SPEED * strafe_sign_;
            ff->_motors->writeAllMotors(0.0f, 0.0f, vtheta);
            return;
        }

        ff->_motors->writeAllMotors(AVOID_STRAFE_SPEED * strafe_sign_,
                                    AVOID_FORWARD_SPEED,
                                    0.0f);
        return;
    }

    // FIND_FIRE behaviour
    if (fire_valid) {
        ff->println("SEARCH: fire detected -> MOVE_TO_FIRE");
        enterMoveToFire(fire_bearing);
        return;
    }

    if (obstacle_ahead) {
        ff->println("SEARCH: obstacle ahead -> AVOID");
        enterAvoid(false);
        return;
    }

    float vtheta = 0.0f;
    if (blocked(lr_cm, WALL_FOLLOW_CM)) {
        vtheta -= SEARCH_TURN_SPEED * (1.0f - lr_cm / WALL_FOLLOW_CM);
    }
    if (blocked(rr_cm, WALL_FOLLOW_CM)) {
        vtheta += SEARCH_TURN_SPEED * (1.0f - rr_cm / WALL_FOLLOW_CM);
    }

    if (!nudge_active_ && (now - last_nudge_ms_) > NUDGE_PERIOD_MS) {
        nudge_active_ = true;
        nudge_start_ms_ = now;
        nudge_dir_ = -nudge_dir_;
    }
    if (nudge_active_) {
        if ((now - nudge_start_ms_) < NUDGE_LEN_MS) {
            vtheta += NUDGE_TURN * nudge_dir_;
        } else {
            nudge_active_ = false;
            last_nudge_ms_ = now;
        }
    }

    ff->_motors->writeAllMotors(0.0f, SEARCH_FORWARD_SPEED, vtheta);
}
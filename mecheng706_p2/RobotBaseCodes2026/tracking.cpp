#include "Arduino.h"
#include <math.h>
#include "tracking.h"
#include "firefighter.h"
#include "mappings.h"

namespace {
static const unsigned long LOST_FIRE_MS = 1500;
static const unsigned long AVOID_TIMEOUT_MS = 3000;
static const unsigned long NUDGE_PERIOD_MS = 4000;
static const unsigned long NUDGE_LEN_MS = 350;
static const float NUDGE_TURN = 60.0f;
static const float BEARING_PIVOT_THRESH_RAD = 0.6f;
static const float RAD_PER_DEG = 0.017453292519943295f;
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
    active_behavior_ = BehaviorNS::SearchBehaviour::AVOID;
    behavior_start_ms_ = millis();
    resume_to_move_ = resume_to_move;

    float lf_cm = firefighter_->_front_left_ir->getAvg();
    float rf_cm = firefighter_->_front_right_ir->getAvg();
    bool lf_valid = (lf_cm > 0.0f);
    bool rf_valid = (rf_cm > 0.0f);
    firefighter_->println("Avoiding");
    firefighter_->_motors->writeAllMotors(100.0f, 0.0f, 0.0f);

    if((lf_cm > 10.0) && (rf_cm > 10.0)){
        firefighter_->println("Safe");
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
    firefighter_->println("Tracking");
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

    // Turret angle arrives from the main sketch as a 0-180 degree target.
    // Convert it to the same robot-frame bearing convention used by the fire
    // estimator: 0 rad = straight ahead, positive = left.
    float target_bearing = ff->bearing_;
    if (target_bearing > 3.1415926f || target_bearing < -3.1415926f) {
        target_bearing = (target_bearing - 90.0f) * RAD_PER_DEG;
    }

    float bearing_error = fire_bearing - target_bearing;

    bool obstacle_ahead = blocked(us_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(lf_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(rf_cm, OBSTACLE_TRIGGER_CM);
    

    bool close_front = blocked(us_cm, EXTINGUISH_RANGE_CM);

    bool aimed = fabsf(bearing_error) < 0.35f;

    ff->print("[TRACK] beh=");
    ff->print((int)active_behavior_);
    ff->print(" fire=");
    ff->print(fire_detected);
    ff->print(" valid=");
    ff->print(fire_valid);
    ff->print(" fireBear=");
    ff->print(fire_bearing, 3);
    ff->print(" target=");
    ff->print(target_bearing, 3);
    ff->print(" err=");
    ff->print(bearing_error, 3);
    ff->print(" aimed=");
    ff->print(aimed);
    ff->print(" obs=");
    ff->print(obstacle_ahead);
    ff->print(" close=");
    ff->println(close_front);

    /*
    firefighter_->print("Obsatcle: ");
    firefighter_->println(obstacle_ahead);
    firefighter_->print("In Front of Fire: ");
    firefighter_->println(close_front);
    firefighter_->print("Aimed: ");
    firefighter_->println(aimed);
    */
    
    if(obstacle_ahead){
        ff->println("[TRACK] obstacle ahead -> AVOID");
        enterAvoid(true);
        return;
    }


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

        float vtheta = APPROACH_TURN_GAIN * bearing_error;
        if (vtheta > APPROACH_MAX_TURN) vtheta = APPROACH_MAX_TURN;
        if (vtheta < -APPROACH_MAX_TURN) vtheta = -APPROACH_MAX_TURN;

        float vy = (fabsf(bearing_error) > BEARING_PIVOT_THRESH_RAD)
                       ? 0.0f
                       : (APPROACH_FORWARD_SPEED * cosf(bearing_error));
        vy = 100.0f;
        ff->print("[TRACK] MOVE_TO_FIRE vtheta=");
        ff->print(vtheta, 2);
        ff->print(" vy=");
        ff->println(vy, 2);
        ff->_motors->writeAllMotors(0.0f, 0.0f, vtheta);
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
        if (clear && (elapsed > AVOID_STRAFE_MS)) {
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
            ff->print("[TRACK] AVOID timeout vtheta=");
            ff->println(vtheta, 2);
            //ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            return;
        }

        //ff->_motors->writeAllMotors(AVOID_STRAFE_SPEED * strafe_sign_,
        //                            AVOID_FORWARD_SPEED,
        //                            0.0f);
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
        ff->println("[TRACK] nudge start");
    }
    if (nudge_active_) {
        if ((now - nudge_start_ms_) < NUDGE_LEN_MS) {
            vtheta += NUDGE_TURN * nudge_dir_;
        } else {
            nudge_active_ = false;
            last_nudge_ms_ = now;
            ff->println("[TRACK] nudge end");
        }
    }
    ff->print("[TRACK] FIND_FIRE vtheta=");
    ff->println(vtheta, 2);
    ff->_motors->writeAllMotors(0.0f, 0.0f, vtheta);
}
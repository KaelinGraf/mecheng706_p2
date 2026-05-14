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

void Tracking::begin() {
    firefighter_->println("Tracking");
    bearing_ = 0.0f;
    resume_bearing_ = 0.0f;
    resume_to_move_ = false;
    active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
    behavior_start_ms_ = millis();
    last_nudge_ms_ = behavior_start_ms_;
    nudge_active_ = false;
}

void Tracking::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Tracking::poll() {
    FireFighter *ff = firefighter_;
    unsigned long now = millis();

    // Read all sensors
    float us_cm = ff->_ultrasonic->getAvg();
    float lf_cm = ff->_front_left_ir->getAvg();
    float rf_cm = ff->_front_right_ir->getAvg();
    float lr_cm = ff->_rear_left_ir->getAvg();
    float rr_cm = ff->_rear_right_ir->getAvg();

    auto blocked = [](float v, float thr) {
        return (v > 0.0f) && (v < thr);
    };

    // Detect and estimate fire bearing
    bool fire_detected = ff->_fire_bank->anyDetection(FIRE_DETECT_V);
    bool fire_valid = false;
    float fire_bearing = bearing_;
    if (fire_detected) {
        fire_bearing = ff->_fire_bank->estimateBearing(&fire_valid, FIRE_DETECT_V);
    }


    
    // Convert turret angle to robot-frame bearing
    float target_bearing = ff->bearing_;
    /*
    if (target_bearing > 3.1415926f || target_bearing < -3.1415926f) {
        target_bearing = (target_bearing - 90.0f);
    }
    */

    float bearing_error = (90 - target_bearing) / 10;

    // Obstacle detection
    bool obstacle_ahead = blocked(us_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(lf_cm, OBSTACLE_TRIGGER_CM) ||
                          blocked(rf_cm, OBSTACLE_TRIGGER_CM);

    bool close_front = blocked(us_cm, EXTINGUISH_RANGE_CM);
    bool aimed = fabsf(bearing_error) < 0.35f;

    // Debug output
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

    // Motor command variables
    float motor_vx = 0.0f;
    float motor_vy = 0.0f;
    float motor_vtheta = 0.0f;

    // PRIORITY 1: Obstacle ahead triggers AVOID
    if (obstacle_ahead && !close_front) {
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            ff->println("[TRACK] obstacle ahead -> AVOID");
            active_behavior_ = BehaviorNS::SearchBehaviour::AVOID;
            behavior_start_ms_ = now;
        }
    }

    // Handle AVOID behavior
    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID) {
        unsigned long elapsed = now - behavior_start_ms_;

        // Update fire info if detected while avoiding
        if (fire_valid) {
            resume_bearing_ = fire_bearing;
            resume_to_move_ = true;
            bearing_ = fire_bearing;
            last_seen_ms_ = now;
        }

        // Check if obstacle is now clear
        bool clear = !obstacle_ahead &&
                     ((us_cm < 0.0f) || (us_cm >= OBSTACLE_CLEAR_CM)) &&
                     ((lf_cm < 0.0f) || (lf_cm >= OBSTACLE_CLEAR_CM)) &&
                     ((rf_cm < 0.0f) || (rf_cm >= OBSTACLE_CLEAR_CM));

        if (clear && (elapsed > AVOID_STRAFE_MS)) {
            ff->println("[TRACK] obstacle cleared");
            if (resume_to_move_) {
                ff->println("[TRACK] resuming MOVE_TO_FIRE");
                active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
                bearing_ = resume_bearing_;
                last_seen_ms_ = now;
            } else {
                ff->println("[TRACK] returning to FIND_FIRE");
                active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
                resume_to_move_ = false;
                last_nudge_ms_ = now;
                nudge_active_ = false;
            }
        } else if (elapsed > AVOID_TIMEOUT_MS) {
            // Timeout: rotate in place
            motor_vtheta = -SEARCH_TURN_SPEED * strafe_sign_;
            ff->print("[TRACK] AVOID timeout vtheta=");
            ff->println(motor_vtheta, 2);
        } else {
            // Continue strafe motion
            motor_vx = AVOID_STRAFE_SPEED * strafe_sign_;
            motor_vy = AVOID_FORWARD_SPEED;
        }
    }
    // PRIORITY 2: Fire detected triggers MOVE_TO_FIRE
    else if (fire_valid && active_behavior_ != BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
        ff->println("[TRACK] fire detected -> MOVE_TO_FIRE");
        active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
        behavior_start_ms_ = now;
        bearing_ = fire_bearing;
        last_seen_ms_ = now;
    }

    // hard code for testing
    active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
    if (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
        // Update fire bearing if still detected
        if (fire_valid || 1) {
            bearing_ = fire_bearing;
            last_seen_ms_ = now;
        } else if ((now - last_seen_ms_) > LOST_FIRE_MS) {
            ff->println("[TRACK] fire lost -> FIND_FIRE");
            active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
            resume_to_move_ = false;
            last_nudge_ms_ = now;
            nudge_active_ = false;
        }

        // Check if close enough to extinguish
        if (close_front && aimed) {
            ff->println("[TRACK] fire within extinguish range");
            motor_vx = 0.0f;
            motor_vy = 0.0f;
            motor_vtheta = 0.0f;
            ff->_motors->writeAllMotors(motor_vx, motor_vy, motor_vtheta);
            ff->switchState(State::EXTINGUISH);
            return;
        }

        // Calculate approach motion
        float vtheta = APPROACH_TURN_GAIN * bearing_error;
        if (vtheta > APPROACH_MAX_TURN) vtheta = APPROACH_MAX_TURN;
        if (vtheta < -APPROACH_MAX_TURN) vtheta = -APPROACH_MAX_TURN;

        float vy = (fabsf(bearing_error) > BEARING_PIVOT_THRESH_RAD)
                       ? 0.0f
                       : (APPROACH_FORWARD_SPEED * cosf(bearing_error));
        vy = 100.0f;

        motor_vtheta = vtheta;
        motor_vy = vy;

        ff->print("[TRACK] MOVE_TO_FIRE vtheta=");
        ff->print(vtheta, 2);
        ff->print(" vy=");
        ff->println(vy, 2);
    }
    // PRIORITY 3: Search (FIND_FIRE)
    else if (active_behavior_ == BehaviorNS::SearchBehaviour::FIND_FIRE) {
        // Spin servo until fire detected
        motor_vtheta = 0.0f;
        
        /*
        // Wall follow with rear IR sensors
        if (blocked(lr_cm, WALL_FOLLOW_CM)) {
            motor_vtheta -= SEARCH_TURN_SPEED * (1.0f - lr_cm / WALL_FOLLOW_CM);
        }
        if (blocked(rr_cm, WALL_FOLLOW_CM)) {
            motor_vtheta += SEARCH_TURN_SPEED * (1.0f - rr_cm / WALL_FOLLOW_CM);
        }
        */

        // Nudge logic
        if (!nudge_active_ && (now - last_nudge_ms_) > NUDGE_PERIOD_MS) {
            nudge_active_ = true;
            nudge_start_ms_ = now;
            nudge_dir_ = -nudge_dir_;
            ff->println("[TRACK] nudge start");
        }
        if (nudge_active_) {
            if ((now - nudge_start_ms_) < NUDGE_LEN_MS) {
                motor_vtheta += NUDGE_TURN * nudge_dir_;
            } else {
                nudge_active_ = false;
                last_nudge_ms_ = now;
                ff->println("[TRACK] nudge end");
            }
        }

        motor_vtheta = -motor_vtheta;
        ff->print("[TRACK] FIND_FIRE vtheta=");
        ff->println(motor_vtheta, 2);
    }

    // Write motors once at the end
    ff->_motors->writeAllMotors(0.0f, 0.0f, motor_vtheta);
}
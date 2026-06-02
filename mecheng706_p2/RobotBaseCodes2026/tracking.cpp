#include "Arduino.h"
#include <math.h>
#include "tracking.h"
#include "firefighter.h"
#include "mappings.h"

// Global turret instance (defined in RobotBaseCodes2026.ino)
extern Turret *turret;

namespace {
static const unsigned long LOST_FIRE_MS = 1500;
static const unsigned long AVOID_TIMEOUT_MS = 3000;
static const unsigned long NUDGE_PERIOD_MS = 4000;
static const unsigned long NUDGE_LEN_MS = 350;
static const float NUDGE_TURN = 60.0f;
static const float BEARING_PIVOT_THRESH = 5.0f;
static const float RAD_PER_DEG = 0.017453292519943295f;

static float wrap_angle_error(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}
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
    float us_cm = ff->_ultrasonic->readBlocking();
    float lf_cm = ff->_front_left_ir->getAvg();
    float rf_cm = ff->_front_right_ir->getAvg();
    float lr_cm = ff->_rear_left_ir->getAvg();
    float rr_cm = ff->_rear_right_ir->getAvg();

    auto blocked = [](float v, float thr) {
        return (v > 0.0f) && (v < thr);
    };

    auto in_band = [](float v, float min_cm, float max_cm) {
        return (v > 0.0f) && (v >= min_cm) && (v <= max_cm);
    };

    // Detect and estimate fire bearing
    bool fire_detected = turret->locked_on_;
    
    // Convert turret angle to robot-frame bearing
    float target_bearing = ff->bearing_;

    //float bearing_error = (90 - target_bearing) / 10; // Ideal
    float bearing_error = (SERVO_CENTER - target_bearing) / 10; // Realsisitc

    bool obstacle_ahead = 0;
    bool obstacle_right = 0;
    bool obstacle_left = 0;
    bool obstacle_side = 0;

    // Obstacle detection
    obstacle_ahead = blocked(us_cm, OBSTACLE_TRIGGER_CM_F);
    obstacle_left = blocked(lf_cm, OBSTACLE_TRIGGER_CM_F);
    obstacle_right = blocked(rf_cm, OBSTACLE_TRIGGER_CM_F); 
    obstacle_side = blocked(lr_cm, OBSTACLE_TRIGGER_CM_R) || blocked(rr_cm, OBSTACLE_TRIGGER_CM_R);

    bool close_front = blocked(us_cm, EXTINGUISH_RANGE_CM);
    bool aimed = (fabsf(bearing_error) < 3.0f) && (turret->locked_on_);

    // Motor command variables
    float motor_vx = 0.0f;
    float motor_vy = 0.0f;
    float motor_vtheta = 0.0f;
    float angle = turret->angle_;
    float center_deadzone = 15.0;

    int curr_turret =1;
    if (angle > (SERVO_CENTER + center_deadzone)){
        curr_turret = 0;
    }else if(angle<(SERVO_CENTER - center_deadzone)){
        curr_turret = 2;
    }
    bool close_to_fire = turret->atFire();
    ff->print("Close to fire: ");
    ff->print(close_to_fire);
    ff->print(" || Max Val: ");
    ff->println(ff->_fire_bank->maxVMid());

    if (((obstacle_ahead || close_front) && ((curr_turret==1) ^ close_front))
        || (obstacle_left && ((curr_turret==0) ^ close_front))
        || (obstacle_right &&  ((curr_turret==2) ^ close_front))
        || obstacle_side) {
        // PRIORITY 1: Obstacle ahead triggers AVOID
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            resume_bearing_ = ff->_gyro->getAngle();
            resume_to_move_ = (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE 
            || active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING);
            ff->println(resume_to_move_);
            active_behavior_ = BehaviorNS::SearchBehaviour::AVOID;
            behavior_start_ms_ = now;
        }
    }
    // recent fire
    if (ff->recentExtinguish() && (close_front || obstacle_ahead)){
        ff->println("[TRACK] RECENT FIRE");
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            resume_bearing_ = ff->_gyro->getAngle();
            resume_to_move_ = (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE 
            || active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING);
            active_behavior_ = BehaviorNS::SearchBehaviour::AVOID;
            behavior_start_ms_ = now;
        }
    }
    if(close_to_fire){
        ff->println("[TRACK] AT_FIRE");
        active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
        behavior_start_ms_ = now;
    }

    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID && ((now-behavior_start_ms_) > 1000) )
    {
        active_behavior_ = BehaviorNS::SearchBehaviour::RETURN_TO_HEADING;
        behavior_start_ms_ = now;
    }
    
    // Handle AVOID behavior
    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID) {
        unsigned long elapsed = now - behavior_start_ms_;
        ff->println("WE ARE IN AVOID");

        // Check if obstacle is now clear
        bool clear = !obstacle_ahead &&
                     ((us_cm < 0.0f) || (us_cm >= OBSTACLE_CLEAR_CM_F)) &&
                     ((lf_cm < 0.0f) || (lf_cm >= OBSTACLE_CLEAR_CM_F)) &&
                     ((rf_cm < 0.0f) || (rf_cm >= OBSTACLE_CLEAR_CM_F)) && 
                     ((lr_cm < 0.0f) || (lr_cm >= OBSTACLE_CLEAR_CM_R)) && 
                     ((rr_cm < 0.0f) || (rr_cm >= OBSTACLE_CLEAR_CM_R));
                     
        if (clear && (elapsed > AVOID_STRAFE_MS)) {
            ff->println("[TRACK] leaving AVOID -> RETURN_TO_HEADING");
            active_behavior_ = BehaviorNS::SearchBehaviour::RETURN_TO_HEADING;
            behavior_start_ms_ = now;
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            return;
        } else if (elapsed > AVOID_TIMEOUT_MS) {
            // Timeout: rotate in place
            motor_vtheta = -SEARCH_TURN_SPEED * strafe_sign_;
            ff->print("[TRACK] AVOID timeout");
        } else {
            if(obstacle_left) {
                ff->print("f: ");
                ff->print(lf_cm);
                ff->print(" r: ");
                ff->println(lr_cm);
                ff->println("[TRACK] AVOID Left");
                if (lf_cm <= AVOID_URGENT) {
                    // Reverse and turn
                    motor_vtheta = AVOID_ROTATE_SPEED*1.5;
                    motor_vx = -AVOID_SPEED;
                } else {
                    // Go forward and around
                    motor_vtheta = AVOID_ROTATE_SPEED;
                    motor_vx = AVOID_SPEED;
                }
            } else if (obstacle_right){
                ff->print("f: ");
                ff->print(rf_cm);
                ff->print(" r: ");
                ff->println(rr_cm);
                ff->println("[TRACK] AVOID Right");
                if (rf_cm <= AVOID_URGENT) {
                    // Reverse and turn
                    motor_vtheta = -AVOID_ROTATE_SPEED*1.5;
                    motor_vx = -AVOID_SPEED;
                } else {
                    // Go forward and around
                    motor_vtheta = -AVOID_ROTATE_SPEED;
                    motor_vx = AVOID_SPEED;
                }
            } else if (obstacle_side) {
                ff->print(" r: ");
                ff->println(rr_cm);
                ff->print(" l: ");
                ff->println(lr_cm);
                ff->println("[TRACK] AVOID Side");
                
                motor_vtheta = 0.0f;
                motor_vx = AVOID_SPEED;

            } else { 
                ff->print(" Bearing Error: ");
                ff->print(bearing_error);
                ff->print(" Ahead: ");
                ff->print(obstacle_ahead);
                ff->print(" Close: ");
                ff->print(close_front);
                ff->print(" aimed=");
                ff->println(aimed);

                if (obstacle_ahead && aimed){
                bool light_thresh = turret->atFire();
                if (light_thresh) {
                    active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;   
                    ff->println("[AVOID] -> APPROACH FIRE");
                    behavior_start_ms_ = now;  
                } else {
                    ff->println("[AVOID] -> FIRE_BEHIND");
                    // fire behind obstacle
                    motor_vtheta = AVOID_ROTATE_SPEED*1.5;
                    motor_vx = -AVOID_SPEED;
                    
                    // being overwritten and stuck
                    ff->_motors->writeAllMotors((-motor_vx/2), motor_vy, motor_vtheta);
                    return;
                }

            } else {
                // Obstacle ahead and not aimed, just go left
                ff->println("[AVOID] -> AHEAD NOT AIMED");
                motor_vtheta = -AVOID_ROTATE_SPEED;
                motor_vx = -AVOID_SPEED;
                }
            }
        }
    }
    if (active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING) {
        float heading_error = wrap_angle_error(resume_bearing_ - ff->_gyro->getAngle());

        if (fabsf(heading_error) > 0.12f) {
            ff->print("[TRACK] RETURN_TO_HEADING: ");
            ff->print(heading_error);
            motor_vx = AVOID_SPEED;
            motor_vtheta = 2*((heading_error > 0.0f) ? -AVOID_ROTATE_SPEED : AVOID_ROTATE_SPEED);
            ff->print("[vTheta: ");
            ff->println(motor_vtheta);
        } else {
            ff->println("[TRACK] heading restored");
            active_behavior_ = resume_to_move_
                ? BehaviorNS::SearchBehaviour::MOVE_TO_FIRE
                : BehaviorNS::SearchBehaviour::FIND_FIRE;
            behavior_start_ms_ = now;
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            return;
        }
        ff->_motors->writeAllMotors((-motor_vx/2), motor_vy, motor_vtheta);
        return;
    }
    
    // PRIORITY 2: Use turret lock to switch between FIND_FIRE and MOVE_TO_FIRE
    else {
        if (turret && fire_detected) {
            if (active_behavior_ != BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
                ff->println("[TRACK] turret locked -> MOVE_TO_FIRE");
                active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
                behavior_start_ms_ = now;
            }
        } else {
            if (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
                ff->println("[TRACK] turret lost lock -> FIND_FIRE");
                active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
                behavior_start_ms_ = now;
            }
        }
    }
    if (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {      
        // Check if close enough to extinguish
        if (close_front && aimed && !ff->recentExtinguish() && turret->atFire()) {
            ff->println("[TRACK] fire within extinguish range");
            motor_vx = 0.0f;
            motor_vy = 0.0f;
            motor_vtheta = 0.0f;
            ff->_motors->writeAllMotors(motor_vx, motor_vy, motor_vtheta);

            // If phototransistors read strong enough

            ff->print("Moving to Extinguish, time: ");
            ff->println(millis());
            
            ff->switchState(State::EXTINGUISH);
            return;
        }

        // Calculate approach motion
        float vtheta = APPROACH_TURN_GAIN * bearing_error;
        if (vtheta > APPROACH_MAX_TURN) vtheta = APPROACH_MAX_TURN;
        if (vtheta < -APPROACH_MAX_TURN) vtheta = -APPROACH_MAX_TURN;

        float vx = (fabsf(bearing_error) > BEARING_PIVOT_THRESH)
                       ? 0.0f
                       : (APPROACH_FORWARD_SPEED * cosf(bearing_error));
        
                       vx = 100.0f;  // For testing
        motor_vtheta = vtheta;

        // If close then slow down
        motor_vx = obstacle_ahead ? (vx*0.6) : vx;
        motor_vx = close_front ? (0.0) : vx;

        //ff->println("[TRACK] MTF");
    }
    // PRIORITY 3: Search (FIND_FIRE)
    else if (active_behavior_ == BehaviorNS::SearchBehaviour::FIND_FIRE) {
        motor_vtheta = 0.0f;

        motor_vx = SEARCH_SPEED;

        if (now - behavior_start_ms_ > 5000) {  // If 5 seconds have passed
            //sweep
            ff->println("Reset, spin scan");
            ff->switchState(State::SPIN_SCAN);
        }

        /*
        // Wall follow with rear IR sensors
        if (blocked(lr_cm, WALL_FOLLOW_CM)) {
            motor_vtheta -= SEARCH_TURN_SPEED * (1.0f - lr_cm / WALL_FOLLOW_CM);
        }
        if (blocked(rr_cm, WALL_FOLLOW_CM)) {
            motor_vtheta += SEARCH_TURN_SPEED * (1.0f - rr_cm / WALL_FOLLOW_CM);
        }

        motor_vtheta = motor_vtheta;
        */
        ff->println("[TRACK] FF");
    }

    // Write motors once at the end
    // ff->print("vx= ");
    // ff->print(-motor_vx, 2); // Moving backwards, fix
    // ff->print(", vy= ");
    // ff->print(motor_vy, 2);
    // ff->print(", vtheta= ");
    // ff->println(motor_vtheta, 2);
    ff->_motors->writeAllMotors((-motor_vx/2), motor_vy, motor_vtheta);
}
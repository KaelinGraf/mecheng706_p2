#include "Arduino.h"
#include <math.h>
#include "tracking.h"
#include "firefighter.h"
#include "mappings.h"

// Global turret instance (defined in RobotBaseCodes2026.ino)
extern Turret *turret;

namespace {
static const unsigned long LOST_FIRE_MS   = 1500;
static const unsigned long AVOID_TIMEOUT_MS = 3000;
static const unsigned long NUDGE_PERIOD_MS  = 4000;
static const unsigned long NUDGE_LEN_MS     = 350;
static const float NUDGE_TURN              = 60.0f;
static const float BEARING_PIVOT_THRESH    = 50.0f;
static const float RAD_PER_DEG             = 0.017453292519943295f;
volatile bool still_wall = 0;
}

static float wrap_angle_error(float angle) {
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle + PI, TWO_PI);
    if (angle < 0.0f) angle += TWO_PI;
    return angle - PI;
}

// ---------------------------------------------------------------------------
// Avoid sub-behaviour helpers
// Each writes into the motor_vx / motor_vy / motor_vtheta references passed
// in and returns true if the caller should immediately call writeAllMotors
// and return (i.e. early-exit path), false to fall through to the normal
// writeAllMotors call at the bottom of poll().
// ---------------------------------------------------------------------------

static void avoidLeft(FireFighter *ff,
                      float lf_cm, float lr_cm,
                      float &motor_vx, float &motor_vy, float &motor_vtheta)
{
    ff->print("f: "); ff->print(lf_cm);
    ff->print(" r: "); ff->println(lr_cm);

    if (lf_cm <= AVOID_URGENT) {
        ff->println("[TRACK] AVOID Left Urgent");
        motor_vtheta = AVOID_ROTATE_SPEED;
        motor_vx     = -AVOID_SPEED;
        motor_vy     = 50;
    } else {
        ff->println("[TRACK] AVOID Left");
        motor_vtheta = AVOID_ROTATE_SPEED * 1.25f;
        motor_vx     = AVOID_SPEED * 0.75f;
    }
}

static void avoidRight(FireFighter *ff,
                       float rf_cm, float rr_cm,
                       float &motor_vx, float &motor_vy, float &motor_vtheta)
{
    ff->print("f: "); ff->print(rf_cm);
    ff->print(" r: "); ff->print(rr_cm);

    if (rf_cm <= AVOID_URGENT) {
        ff->println(" [TRACK] AVOID Right Urgent");
        motor_vtheta = -AVOID_ROTATE_SPEED;
        motor_vx     = -AVOID_SPEED;
        motor_vy     = -50;
    } else {
        ff->println(" [TRACK] AVOID Right");
        motor_vtheta = -AVOID_ROTATE_SPEED * 1.25f;
        motor_vx     = -AVOID_SPEED * 0.75f;
    }
}

static void avoidSide(FireFighter *ff,
                      float lr_cm, float rr_cm,
                      float &motor_vx, float &motor_vy, float &motor_vtheta)
{
    ff->print(" r: "); ff->print(rr_cm);
    ff->print(" l: "); ff->print(lr_cm);
    ff->println(" [TRACK] AVOID Side");

    auto blocked = [](float v, float thr) { return (v > 0.0f) && (v < thr); };

    if (blocked(lr_cm, OBSTACLE_TRIGGER_CM_R)){
        motor_vy += 40;
        motor_vtheta -= 30;
    }
    if (blocked(rr_cm, OBSTACLE_TRIGGER_CM_R)){
        motor_vy -= 40;
        motor_vtheta += 30;
    }
    motor_vx     = AVOID_SPEED;
}

// Returns true if the caller should early-return after writing motors.
static bool avoidAhead(FireFighter *ff, float fr, float fl, 
                       bool aimed, bool close_to_fire,
                       unsigned long now,
                       BehaviorNS::SearchBehaviour &active_behavior_,
                       unsigned long &behavior_start_ms_,
                       float &motor_vx, float &motor_vy, float &motor_vtheta)
{
    if (aimed) {
        if (close_to_fire) {
            active_behavior_    = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
            behavior_start_ms_  = now;
            ff->println("[AVOID] -> APPROACH FIRE");
        } else {
            ff->println("[AVOID] -> FIRE_BEHIND");
            motor_vtheta = AVOID_ROTATE_SPEED * 1.5f;
            motor_vx     = -AVOID_SPEED;
            motor_vy     = 50;
            ff->_motors->writeAllMotors((-motor_vx / 2), motor_vy, motor_vtheta);
            return true;   // caller must return immediately
        }
    } else {
        ff->println("[AVOID] -> AHEAD NOT AIMED");
        motor_vtheta = (fr < fl) ? -AVOID_ROTATE_SPEED * 0.75f : AVOID_ROTATE_SPEED * 0.75f;
        motor_vx     = -AVOID_SPEED;
    }
    return false;
}

static void avoidTimeout(float &motor_vtheta, int strafe_sign_)
{
    motor_vtheta = -SEARCH_TURN_SPEED * strafe_sign_;
}

static void continueLastAvoid(FireFighter *ff,
                              Tracking::AvoidMode mode,
                              float lf_cm, float rf_cm, float lr_cm, float rr_cm,
                              float &motor_vx, float &motor_vy, float &motor_vtheta)
{
    switch (mode) {
        case Tracking::AvoidMode::LEFT:
            avoidLeft(ff, lf_cm, lr_cm, motor_vx, motor_vy, motor_vtheta);
            break;
        case Tracking::AvoidMode::RIGHT:
            avoidRight(ff, rf_cm, rr_cm, motor_vx, motor_vy, motor_vtheta);
            break;
        case Tracking::AvoidMode::SIDE:
            avoidSide(ff, lr_cm, rr_cm, motor_vx, motor_vy, motor_vtheta);
            break;
        case Tracking::AvoidMode::AHEAD:
            ff->println("[TRACK] AVOID continue ahead");
            motor_vtheta = AVOID_ROTATE_SPEED * 0.75f;
            motor_vx     = -AVOID_SPEED;
            break;
        case Tracking::AvoidMode::NONE:
        default:
            break;
    }
}

// ---------------------------------------------------------------------------

Tracking::Tracking(FireFighter *firefighter)
    : firefighter_(firefighter),
      active_behavior_(BehaviorNS::SearchBehaviour::FIND_FIRE),
      bearing_(0.0f),
      resume_bearing_(0.0f),
      resume_to_move_(false),
      strafe_sign_(1),
    last_avoid_mode_(AvoidMode::NONE),
      behavior_start_ms_(0),
      last_seen_ms_(0),
      last_nudge_ms_(0),
      nudge_start_ms_(0),
      nudge_active_(false),
      nudge_dir_(1) {}

void Tracking::begin() {
    firefighter_->println("Tracking");
    bearing_         = 0.0f;
    resume_bearing_  = 0.0f;
    resume_to_move_  = false;
    active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
    last_avoid_mode_ = AvoidMode::NONE;
    behavior_start_ms_ = millis();
    last_nudge_ms_     = behavior_start_ms_;
    nudge_active_      = false;
}

void Tracking::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void Tracking::poll() {
    FireFighter *ff  = firefighter_;
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

    bool fire_detected   = turret->locked_on_;
    float target_bearing = ff->bearing_;
    float bearing_error  = (SERVO_CENTER - target_bearing) / 10;

    bool obstacle_ahead = blocked(us_cm, OBSTACLE_TRIGGER_CM_US);
    bool obstacle_left  = blocked(lf_cm, OBSTACLE_TRIGGER_CM_F);
    bool obstacle_right = blocked(rf_cm, OBSTACLE_TRIGGER_CM_F);
    bool obstacle_side  = blocked(lr_cm, OBSTACLE_TRIGGER_CM_R) || blocked(rr_cm, OBSTACLE_TRIGGER_CM_R);

    bool close_front  = blocked(us_cm, EXTINGUISH_RANGE_CM);
    bool aimed        = (fabsf(bearing_error) < 3.0f) && (turret->locked_on_);

    float motor_vx     = 0.0f;
    float motor_vy     = 0.0f;
    float motor_vtheta = 0.0f;

    float angle           = turret->angle_;
    float center_deadzone = 15.0f;

    int curr_turret = 1;
    if      (angle > (SERVO_CENTER + center_deadzone)) curr_turret = 0;
    else if (angle < (SERVO_CENTER - center_deadzone)) curr_turret = 2;

    bool close_to_fire = turret->atFire();

    // -----------------------------------------------------------------------
    // Priority 1: obstacle → AVOID
    // -----------------------------------------------------------------------
    if ((obstacle_ahead || close_front)
        || (obstacle_left)
        || (obstacle_right)
        || obstacle_side)
    {
        ff->println("[TRACK] OBSTACLE");
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            if (active_behavior_ != BehaviorNS::SearchBehaviour::RETURN_TO_HEADING)
                resume_bearing_ = ff->_gyro->getAngle();
            resume_to_move_ = (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE
                             || active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING);
            active_behavior_   = BehaviorNS::SearchBehaviour::AVOID;
            behavior_start_ms_ = now;
        }
    }

    // Recent extinguish + still close
    if (ff->recentExtinguish() && (close_front || obstacle_ahead)) {
        ff->println("[TRACK] RECENT FIRE");
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            resume_bearing_    = ff->_gyro->getAngle();
            resume_to_move_    = (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE
                                || active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING);
            active_behavior_   = BehaviorNS::SearchBehaviour::AVOID;
            behavior_start_ms_ = now;
        }
    }

    if (close_to_fire) {
        ff->println("[TRACK] AT_FIRE");
        active_behavior_   = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
        behavior_start_ms_ = now;
    }

    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID
        && ((now - behavior_start_ms_) > 1500))
    {
        ff->println("Avoid Timeout -> FF");
        active_behavior_   = BehaviorNS::SearchBehaviour::FIND_FIRE;
        behavior_start_ms_ = now;
    } else if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID && active_behavior_ != BehaviorNS::SearchBehaviour::RETURN_TO_HEADING) {
        // -----------------------------------------------------------------------
        // Priority 2: turret lock → MOVE_TO_FIRE / FIND_FIRE
        // -----------------------------------------------------------------------
        if (turret && fire_detected) {
            if (active_behavior_ != BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
                ff->println("[TRACK] turret locked -> MOVE_TO_FIRE");
                active_behavior_   = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
                behavior_start_ms_ = now;
            }
        } else {
            if (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE) {
                ff->println("[TRACK] turret lost lock -> FIND_FIRE");
                active_behavior_   = BehaviorNS::SearchBehaviour::FIND_FIRE;
                behavior_start_ms_ = now;
                if (!turret) ff->println("Turret Issue - HOW DID WE GET HERE");
            }
        }
    }

    // -----------------------------------------------------------------------
    // Behaviour dispatch
    // -----------------------------------------------------------------------
    switch (active_behavior_) {

        case BehaviorNS::SearchBehaviour::AVOID: {
            unsigned long elapsed = now - behavior_start_ms_;

            bool clear = !obstacle_ahead
                      && ((us_cm < 0.0f) || (us_cm >= OBSTACLE_CLEAR_CM_F))
                      && ((lf_cm < 0.0f) || (lf_cm >= OBSTACLE_CLEAR_CM_F))
                      && ((rf_cm < 0.0f) || (rf_cm >= OBSTACLE_CLEAR_CM_F))
                      && ((lr_cm < 0.0f) || (lr_cm >= OBSTACLE_CLEAR_CM_R))
                      && ((rr_cm < 0.0f) || (rr_cm >= OBSTACLE_CLEAR_CM_R));

            if (clear) {
                ff->println("[TRACK] AVOID clear");
                active_behavior_ = BehaviorNS::SearchBehaviour::FIND_FIRE;
                behavior_start_ms_ = now;
            } else if (elapsed > AVOID_TIMEOUT_MS) {
                ff->print("[TRACK] AVOID timeout");
                avoidTimeout(motor_vtheta, strafe_sign_);
            } else if (obstacle_left) {
                last_avoid_mode_ = AvoidMode::LEFT;
                avoidLeft(ff, lf_cm, lr_cm, motor_vx, motor_vy, motor_vtheta);
            } else if (obstacle_right) {
                last_avoid_mode_ = AvoidMode::RIGHT;
                avoidRight(ff, rf_cm, rr_cm, motor_vx, motor_vy, motor_vtheta);
            } else if (obstacle_side) {
                last_avoid_mode_ = AvoidMode::SIDE;
                avoidSide(ff, lr_cm, rr_cm, motor_vx, motor_vy, motor_vtheta);
            } else if (obstacle_ahead || close_front){
                last_avoid_mode_ = AvoidMode::AHEAD;
                if (avoidAhead(ff, rf_cm, lf_cm, aimed, close_to_fire, now,
                               active_behavior_, behavior_start_ms_,
                               motor_vx, motor_vy, motor_vtheta)) {
                    return;
                }
            } else {
                ff->println("[TRACK] AVOID no fresh obstacle, continue last obstacle");
                continueLastAvoid(ff, last_avoid_mode_, lf_cm, rf_cm, lr_cm, rr_cm,
                                  motor_vx, motor_vy, motor_vtheta);
            }
            break;
        }

        case BehaviorNS::SearchBehaviour::RETURN_TO_HEADING: {
            float heading_error = wrap_angle_error(resume_bearing_ - ff->_gyro->getAngle());

            if (fabsf(heading_error) > 0.12f) {
                still_wall = (blocked(lr_cm, 20.0f) || blocked(rr_cm, 20.0f)) ? 1 : 0;
                if (still_wall) {
                    ff->println("Still Wall");
                    motor_vx     = AVOID_SPEED;
                    motor_vtheta = 0;
                } else {
                    ff->println("No Wall");
                    motor_vx     = 0;
                    motor_vtheta = 2.0f * ((heading_error > 0.0f) ? -AVOID_ROTATE_SPEED : AVOID_ROTATE_SPEED);
                }
            } else {
                ff->println("[TRACK] heading restored");
                still_wall         = 0;
                active_behavior_   = resume_to_move_
                    ? BehaviorNS::SearchBehaviour::MOVE_TO_FIRE
                    : BehaviorNS::SearchBehaviour::FIND_FIRE;
                behavior_start_ms_ = now;
                ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
                return;
            }
            ff->_motors->writeAllMotors((-motor_vx / 2), motor_vy, motor_vtheta);
            return;
        }

        case BehaviorNS::SearchBehaviour::MOVE_TO_FIRE: {
            if (close_front && aimed && !ff->recentExtinguish() && close_to_fire) {
                ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
                ff->print("Moving to Extinguish, time: ");
                ff->println(millis());
                ff->switchState(State::EXTINGUISH);
                return;
            }
            float vtheta = APPROACH_TURN_GAIN * bearing_error;
            if (vtheta >  APPROACH_MAX_TURN) vtheta =  APPROACH_MAX_TURN;
            if (vtheta < -APPROACH_MAX_TURN) vtheta = -APPROACH_MAX_TURN;

            float vx = (fabsf(bearing_error) > BEARING_PIVOT_THRESH)
                       ? 0.0f
                       : (APPROACH_FORWARD_SPEED * cosf(bearing_error*DEG_TO_RAD));
            ff->print(vx);
            ff->print("   ");
            ff->println(close_front);
            motor_vtheta = vtheta;
            motor_vx     = close_front   ? 0.0f  :
                           obstacle_ahead ? vx * 0.6f : vx;
            break;
        }

        case BehaviorNS::SearchBehaviour::FIND_FIRE: {
            ff->print("time in ff ");
            ff->println(now - behavior_start_ms_);
            motor_vtheta = 0.0f;
            motor_vx     = SEARCH_SPEED;

            if (now - behavior_start_ms_ > 3500) {
                ff->println("Reset, spin scan");
                ff->switchState(State::SPIN_SCAN);
                return;
            }
            break;
        }

        default: {
            ff->println("   ----------");
            ff->println("--------------------");
            ff->println("\t\tNO BEHAVIOUR");
            ff->println("--------------------");
            ff->println("   ----------");
        }
    }
    
    ff->_motors->writeAllMotors((-motor_vx / 2), motor_vy, motor_vtheta);
}
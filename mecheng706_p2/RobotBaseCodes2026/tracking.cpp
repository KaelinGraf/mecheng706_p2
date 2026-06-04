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
volatile bool still_wall = 0;

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

    // Read all sensors. Ultrasonic is the NON-BLOCKING cache now (FireFighter::pollState() pumps
    // _ultrasonic->service() every loop); getAvg() returns the median of recent pings without stalling.
    float us_cm = ff->_ultrasonic->getAvg();
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

    // === STRAFE-CENTRIC CLEARANCE ============================================================
    // Closest obstacle on each side (cm; large = open). Front IR are short-range, rear IR long-range.
    auto side_min = [&](float a, float b) {
        float m = 1.0e6f;
        if (a > 0.0f && a < m) m = a;
        if (b > 0.0f && b < m) m = b;
        return m;
    };
    const float left_clear  = side_min(lf_cm, lr_cm);
    const float right_clear = side_min(rf_cm, rr_cm);

    // Centering / clearance strafe (+vy = RIGHT -- confirmed vs the mecanum mix + mapping). When a side
    // is within the care band, steer toward the roomier side: this CENTRES the robot in a gap and edges
    // it away from a near side object WITHOUT rotating, so a barely-fit gap stays threadable and side
    // objects are never clipped. This term is ADDED to the normal drive (MOVE_TO_FIRE / FIND_FIRE)
    // below; the heavy reverse/rotate AVOID is reserved for a true frontal wall (boxed-in fallback).
    // STRAFE_DIR_SIGN is the one-line flip if the bench shows the strafe goes the wrong way.
    float vy_clear = 0.0f;
    const bool side_near = (left_clear < WALL_CARE_CM) || (right_clear < WALL_CARE_CM);
    if (side_near) {
        vy_clear = WALL_CENTER_GAIN * (right_clear - left_clear);   // >0 => more room right => go right
        if (vy_clear >  WALL_STRAFE_SPEED) vy_clear =  WALL_STRAFE_SPEED;
        if (vy_clear < -WALL_STRAFE_SPEED) vy_clear = -WALL_STRAFE_SPEED;
    }
    // Hard imminent-collision override: a side this close -> decisive full strafe away.
    if (left_clear  < SIDE_HARD_MIN_CM) vy_clear =  WALL_STRAFE_SPEED;
    if (right_clear < SIDE_HARD_MIN_CM) vy_clear = -WALL_STRAFE_SPEED;
    vy_clear *= STRAFE_DIR_SIGN;
    // Creep the forward speed when a side is tight, so the strafe has time to clear it before we pass.
    float creep = 1.0f;
    if (side_near)                                                    creep = 0.6f;
    if (left_clear < SIDE_HARD_MIN_CM || right_clear < SIDE_HARD_MIN_CM) creep = 0.35f;

    // FRONTAL block ONLY: the ULTRASONIC (centre) is the gap/wall discriminator -- a gap's centre is
    // OPEN at close range (the posts fall outside the cone) while a wall's centre is CLOSED. SIDE
    // objects are handled by the clearance strafe above, NOT by this heavy reverse/rotate avoid (that
    // was what made gaps un-threadable and twitched against side walls). `aimed`/`close_to_fire`
    // suppress avoiding the FIRE itself on final approach. Reverse/rotate AVOID is now only the
    // boxed-in fallback for a real wall ahead.
    if (obstacle_ahead && !aimed && !close_to_fire) {
        if (active_behavior_ != BehaviorNS::SearchBehaviour::AVOID) {
            resume_bearing_ = ff->_gyro->getAngle();
            resume_to_move_ = (active_behavior_ == BehaviorNS::SearchBehaviour::MOVE_TO_FIRE
            || active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING);
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

    // (Removed the blind ">1000 ms -> RETURN_TO_HEADING" cap: it cut AVOID off after 1 s regardless of
    // whether the path had cleared -- and made the AVOID_TIMEOUT_MS branch below dead code -- so any
    // obstacle needing >1 s to clear produced the back-away/twitch cycle. AVOID now exits on the actual
    // `clear` check below, with AVOID_TIMEOUT_MS as the rotate-in-place backstop.)

    // Handle AVOID behavior
    if (active_behavior_ == BehaviorNS::SearchBehaviour::AVOID) {
        unsigned long elapsed = now - behavior_start_ms_;

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
                    motor_vy = 50;
                } else {
                    // Go forward and around
                    motor_vtheta = AVOID_ROTATE_SPEED;
                    motor_vx = AVOID_SPEED;
                }
            } else if (obstacle_right){
                ff->print("f: ");
                ff->print(rf_cm);
                ff->print(" r: ");
                ff->print(rr_cm);
                ff->println(" [TRACK] AVOID Right");
                if (rf_cm <= AVOID_URGENT) {
                    // Reverse and turn
                    motor_vtheta = -AVOID_ROTATE_SPEED*1.5;
                    motor_vx = -AVOID_SPEED;
                    motor_vy = -50;
                } else {
                    // Go FORWARD and around (matches the obstacle_left branch; the old
                    // motor_vx = -AVOID_SPEED drove BACKWARD here -- asymmetric with left and with
                    // this branch's own "forward" comment). +motor_vx = forward via the -motor_vx/2 write.
                    motor_vtheta = -AVOID_ROTATE_SPEED;
                    motor_vx = AVOID_SPEED;
                }
            } else if (obstacle_side) {
                ff->print(" r: ");
                ff->print(rr_cm);
                ff->print(" l: ");
                ff->print(lr_cm);
                ff->println(" [TRACK] AVOID Side");
                
                motor_vtheta = 0.0f;
                if (blocked(lr_cm, OBSTACLE_TRIGGER_CM_R)) {
                    motor_vy += 40;
                }
                if (blocked(rr_cm, OBSTACLE_TRIGGER_CM_R)) {
                    motor_vy -= 40;
                }
                motor_vx = AVOID_SPEED;

                        } else {
                if (obstacle_ahead && aimed) {

                    if (close_to_fire) {
                        active_behavior_ = BehaviorNS::SearchBehaviour::MOVE_TO_FIRE;
                        ff->println("[AVOID] -> APPROACH FIRE");
                        behavior_start_ms_ = now;
                    } else {
                        ff->println("[AVOID] -> FIRE_BEHIND");

                        motor_vtheta = (rf_cm < lf_cm) ? AVOID_ROTATE_SPEED : -AVOID_ROTATE_SPEED;
                        motor_vx = -AVOID_SPEED;

                        ff->_motors->writeAllMotors((-motor_vx / 2), motor_vy, motor_vtheta);
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

    else if (active_behavior_ == BehaviorNS::SearchBehaviour::RETURN_TO_HEADING) {
        float heading_error = wrap_angle_error(resume_bearing_ - ff->_gyro->getAngle());

        if (fabsf(heading_error) > 0.12f) {

            still_wall = (blocked(lr_cm, 20.0) || blocked(rr_cm, 20.0)) ? 1 : 0;
            if (still_wall){
                ff->println("Still Wall");
                motor_vx = AVOID_SPEED/2;
                motor_vtheta = 0;
            } else {
                ff->println("No Wall");
                motor_vx = 0;
                motor_vtheta = 2*((heading_error > 0.0f) ? -AVOID_ROTATE_SPEED : AVOID_ROTATE_SPEED);
            }
        } else {
            ff->println("[TRACK] heading restored");
            still_wall = 0;
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
        if (close_front && aimed && !ff->recentExtinguish() && close_to_fire) {
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

        // Pivot-then-go: only translate once the chassis is actually aimed at the fire (turret near
        // centre); otherwise hold vx = 0 and let vtheta yaw us onto it. cosf() needs RADIANS, but
        // bearing_error is a deg/10 pseudo-unit -- build the real off-boresight angle from the turret
        // offset directly. (The previous code computed this correctly, then threw it away with a
        // hard-coded `vx = 100.0f; // For testing` that drove full speed regardless of aim -- the main
        // reason the robot charged forward without keeping heading on the fire.)
        const float err_deg = SERVO_CENTER - target_bearing;     // real degrees off boresight
        float vx = (fabsf(err_deg) > BEARING_PIVOT_THRESH)
                       ? 0.0f                                     // not aimed -> pivot in place
                       : (APPROACH_FORWARD_SPEED * cosf(err_deg * RAD_PER_DEG));
        motor_vtheta = vtheta;

        // Slow for an obstacle ahead, then stop hard at extinguish range. CHAINED: the old second
        // line re-read `vx` and silently clobbered the obstacle slow-down on the line above.
        motor_vx = obstacle_ahead ? (vx * 0.6f) : vx;
        if (close_front) motor_vx = 0.0f;
        motor_vx *= creep;        // creep when a side is tight, so the strafe can clear it
        motor_vy += vy_clear;     // centre in gaps / edge away from side objects while approaching

        //ff->println("[TRACK] MTF");
    }
    // PRIORITY 3: Search (FIND_FIRE)
    else if (active_behavior_ == BehaviorNS::SearchBehaviour::FIND_FIRE) {
        motor_vtheta = 0.0f;

        motor_vx = SEARCH_SPEED * creep;   // creep past tight sides
        motor_vy += vy_clear;              // centre between / edge away from side obstacles while cruising

        if (now - behavior_start_ms_ > 5000) {  // If 2 seconds have passed
            //sweep
            ff->println("Reset, spin scan");
            ff->switchState(State::SPIN_SCAN);
            return;
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
        // ff->println("[TRACK] FF");
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
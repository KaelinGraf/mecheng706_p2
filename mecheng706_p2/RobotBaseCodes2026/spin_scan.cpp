#include "Arduino.h"
#include "spin_scan.h"
#include "firefighter.h"

// Global turret instance (defined in RobotBaseCodes2026.ino)
extern Turret *turret;

void SpinScan::begin() {
    FireFighter* ff = firefighter_;
    ff->println("SPIN_SCAN: begin");

    // Stop, drop any stale lock, and hold the turret dead-centre for the whole
    // spin so the phototransistor readings reflect the chassis heading only.
    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    turret->lockOn(false);
    turret->center();

    // Zero the integrated heading so we can detect a full revolution.
    ff->_gyro->resetAngle();

    // Initialize tracking variables for the sweep
    max_intensity_ = 0.0f;
    best_angle_ = 0.0f;
    returning_to_best_ = false;
    prev_angle_ = ff->_gyro->getAngle();
    total_swept_angle_ = 0.0f;
    last_status_print_ms_ = millis();
}

void SpinScan::end() {
    // Keep this function strictly for cleanup.
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    firefighter_->println("SPIN_SCAN: end");
}

void SpinScan::poll() {
    FireFighter* ff = firefighter_;
    ff->_fire_bank->update();

    float current_angle = ff->_gyro->getAngle();
    unsigned long now = millis();

    if (now - last_status_print_ms_ >= 1000UL) {
        ff->print("SPIN_SCAN: angle=");
        ff->print(current_angle);
        ff->print(" max=");
        ff->println(max_intensity_);
        last_status_print_ms_ = now;
    }

    // PHASE 1: Do a full sweep to find the maximum light intensity
    if (!returning_to_best_) {
        float sr = ff->_fire_bank->_sr->getFilteredV();
        float sl = ff->_fire_bank->_sl->getFilteredV();
        float current_intensity = sr < sl ? sr : sl; // takes minimum of side PT

        float early_exit_intensity = ff->firesExtinguished() == 0 ? 1.8f : 0.6f;
        if (current_intensity > early_exit_intensity) {
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->println("SPIN_SCAN: High intensity found early -> SEARCH");
            ff->switchState(State::SEARCH);
            return;
        }

        if (current_intensity > max_intensity_) {
            max_intensity_ = current_intensity;
            best_angle_ = current_angle;
        }
        
        // Calculate the difference since the last loop
        float delta = current_angle - prev_angle_;
        
        // Handle the -PI to PI wrap-around
        while (delta > PI) delta -= TWO_PI;
        while (delta < -PI) delta += TWO_PI;
        
        // Accumulate the absolute rotation and update prev_angle
        total_swept_angle_ += fabs(delta);
        prev_angle_ = current_angle;

        // Exit condition: Has the accumulated rotation reached a full circle?
        if (total_swept_angle_ >= TWO_PI) {
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->print("best angle: ");
            ff->println(best_angle_);
            ff->println("SPIN_SCAN: 360 complete. Returning to best angle.");
            returning_to_best_ = true;
            return;
        }

        ff->_motors->writeAllMotors(0.0f, 0.0f, SPIN_SCAN_SPEED);
    } 
    // PHASE 2: Rotate back to roughly face the best angle
    else {
        float error = best_angle_ - current_angle;

        while (error > PI) error -= TWO_PI;
        while (error < -PI) error += TWO_PI;

        if (fabs(error) < 0.2f) {
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->println("SPIN_SCAN: Facing best angle -> SEARCH");
            ff->switchState(State::SEARCH);
        } else {
            float rot_speed = (error > 0) ? -SPIN_SCAN_SPEED : SPIN_SCAN_SPEED;
            ff->_motors->writeAllMotors(0.0f, 0.0f, rot_speed);
        }
    }
}
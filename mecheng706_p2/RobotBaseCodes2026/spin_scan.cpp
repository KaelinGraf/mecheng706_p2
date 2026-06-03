#include "Arduino.h"
#include "spin_scan.h"
#include "firefighter.h"

// Global turret instance (defined in RobotBaseCodes2026.ino)
extern Turret *turret;

// How long to sample the peak intensity before arming the fallback (ms)
static constexpr unsigned long FALLBACK_SAMPLE_MS = 11000UL;

// How close to the recorded peak we need to be to trigger the fallback exit.
// A small margin handles slight sensor noise.
static constexpr float FALLBACK_THRESHOLD_RATIO = 0.95f;  // 95 % of peak

void SpinScan::begin() {
    FireFighter* ff = firefighter_;
    ff->println("SPIN_SCAN: begin");

    ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    turret->lockOn(false);
    turret->center();

    ff->_gyro->resetAngle();

    // Core tracking
    max_intensity_        = 0.0f;
    best_angle_           = 0.0f;
    returning_to_best_    = false;
    prev_angle_           = ff->_gyro->getAngle();
    total_swept_angle_    = 0.0f;
    last_status_print_ms_ = millis();

    // Gyro-fallback tracking
    scan_start_ms_         = millis();
    fallback_max_intensity_ = 0.0f;
    fallback_armed_        = false;
    fallback_triggered_    = false;
}

void SpinScan::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    firefighter_->println("SPIN_SCAN: end");
}

void SpinScan::poll() {
    FireFighter* ff = firefighter_;
    ff->_fire_bank->update();

    float current_angle = ff->_gyro->getAngle();
    unsigned long now   = millis();

    if (now - last_status_print_ms_ >= 1000UL) {
        ff->print("SPIN_SCAN: angle=");
        ff->print(current_angle);
        ff->print(" swept=");
        ff->print(total_swept_angle_);
        ff->print(" max=");
        ff->println(max_intensity_);
        last_status_print_ms_ = now;
    }

    // -----------------------------------------------------------------------
    // PHASE 1: full sweep
    // -----------------------------------------------------------------------
    if (!returning_to_best_) {
        float sr = ff->_fire_bank->_sr->getFilteredV();
        float sl = ff->_fire_bank->_sl->getFilteredV();
        float current_intensity = sr < sl ? sr : sl;

        // --- Early-exit on bright signal ---
        float early_exit_intensity = ff->firesExtinguished() == 0 ? 1.8f : 0.6f;
        if (current_intensity > early_exit_intensity) {
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->println("SPIN_SCAN: High intensity found early -> SEARCH");
            ff->switchState(State::SEARCH);
            return;
        }

        // --- Track best ---
        if (current_intensity > max_intensity_) {
            max_intensity_ = current_intensity;
            best_angle_    = current_angle;
        }

        // --- Gyro fallback: sample window ---
        if (!fallback_armed_) {
            if (current_intensity > fallback_max_intensity_) {
                fallback_max_intensity_ = current_intensity;
            }
            if (now - scan_start_ms_ >= FALLBACK_SAMPLE_MS) {
                fallback_armed_ = true;
                ff->print("SPIN_SCAN: fallback armed, peak=");
                ff->println(fallback_max_intensity_);
            }
        }

        // --- Gyro fallback: exit trigger ---
        // After the 5 s window, if the sensor returns to ≥95 % of the sampled
        // peak, the gyro is most likely stuck and we've done roughly one full
        // revolution.  Treat it the same as completing 360 °.
        if (fallback_armed_ && !fallback_triggered_) {
            float threshold = fallback_max_intensity_ * FALLBACK_THRESHOLD_RATIO;
            if (current_intensity >= threshold) {
                fallback_triggered_ = true;
                ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
                ff->print("SPIN_SCAN: fallback exit triggered at intensity=");
                ff->print(current_intensity);
                ff->print(" (threshold=");
                ff->print(threshold);
                ff->println(") -> returning to best");
                returning_to_best_ = true;
                return;
            }
        }

        // --- Gyro accumulation ---
        float delta = current_angle - prev_angle_;
        while (delta >  PI)  delta -= TWO_PI;
        while (delta < -PI)  delta += TWO_PI;
        total_swept_angle_ += fabs(delta);
        prev_angle_         = current_angle;

        // --- Normal 360 ° exit ---
        if (total_swept_angle_ >= TWO_PI) {
            ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
            ff->print("SPIN_SCAN: 360 complete. best_angle=");
            ff->println(best_angle_);
            returning_to_best_ = true;
            return;
        }

        ff->_motors->writeAllMotors(0.0f, 0.0f, SPIN_SCAN_SPEED);
    }
    // -----------------------------------------------------------------------
    // PHASE 2: return to best angle
    // -----------------------------------------------------------------------
    else {
        float error = best_angle_ - current_angle;
        while (error >  PI)  error -= TWO_PI;
        while (error < -PI)  error += TWO_PI;

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
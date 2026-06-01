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
}

void SpinScan::end() {
    firefighter_->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
}

void SpinScan::poll() {
    FireFighter* ff = firefighter_;

    // updateTurret() is suppressed while in SPIN_SCAN, so refresh the fire bank
    // here (cheap: 4 ADC reads).
    ff->_fire_bank->update();

    // Found the fire: use the SAME gate as the turret lock so the handoff to
    // SEARCH locks on within the same loop.
    if (ff->_fire_bank->bothOuterAbove(FIRE_LOCK_OUTER_V)) {
        ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
        ff->println("SPIN_SCAN: fire found -> SEARCH");
        ff->switchState(State::SEARCH);
        return;
    }

    // Completed a full revolution with no detection: fall back to the normal
    // drive + turret pan-scan search.
    if (fabs(ff->_gyro->getAngle()) >= TWO_PI) {
        ff->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
        ff->println("SPIN_SCAN: no fire in 360 -> SEARCH");
        ff->switchState(State::SEARCH);
        return;
    }

    // Keep rotating in place.
    ff->_motors->writeAllMotors(0.0f, 0.0f, SPIN_SCAN_SPEED);
}

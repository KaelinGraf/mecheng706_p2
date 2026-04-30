#include "initialising.h"
#include "firefighter.h"

// ---------------------------------------------------------------------------
// Initialising
// ---------------------------------------------------------------------------
// The FireFighter constructor wires up sensors and motors in setup(), but it
// can't immediately run a behavioural state because:
//   * the BNO08x gyro takes a few hundred milliseconds to begin emitting good
//     samples after the I2C report is enabled,
//   * the IR/ultrasonic ring buffers need a couple of valid samples to make
//     median filtering meaningful,
//   * the phototransistor EWMA needs to settle to ambient so the FIRE_DETECT
//     threshold is meaningful at start.
//
// This state stalls in place for a short time, taking sensor readings to warm
// the filters, then transitions to SEARCH.
// ---------------------------------------------------------------------------

static const unsigned long WARMUP_MS = 1000;

void Initialising::begin() {
    firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
    firefighter_->_fan->off();
    firefighter_->_fire_bank->reset();
    firefighter_->_front_left_ir->resetBuffer();
    firefighter_->_front_right_ir->resetBuffer();
    firefighter_->_rear_left_ir->resetBuffer();
    firefighter_->_rear_right_ir->resetBuffer();
    firefighter_->_ultrasonic->resetBuffer();

    last_millis_ = firefighter_->clock() ? firefighter_->clock()->now_ms() : 0;
    count_ = 0;
}

void Initialising::end() {
    // Nothing to tear down - the warm-up is purely passive.
}

void Initialising::poll() {
    firefighter_->_front_left_ir->getAvg();
    firefighter_->_front_right_ir->getAvg();
    firefighter_->_rear_left_ir->getAvg();
    firefighter_->_rear_right_ir->getAvg();
    firefighter_->_ultrasonic->getAvg();
    count_++;

    unsigned long now = firefighter_->clock() ? firefighter_->clock()->now_ms() : 0;
    if (now - last_millis_ >= WARMUP_MS) {
        firefighter_->println("Initialising: warm-up done, switching to SEARCH");
        firefighter_->switchState(State::SEARCH);
    }
}

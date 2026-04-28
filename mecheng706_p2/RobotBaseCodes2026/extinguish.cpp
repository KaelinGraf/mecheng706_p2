#include "Arduino.h"
#include "extinguish.h"
#include "firefighter.h"

void Extinguish::begin() {
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
  _start_millis = millis();
  // TODO: drive fan_pin HIGH (MOSFET gate) to turn fan on
}

void Extinguish::end() {
  // TODO: drive fan_pin LOW
}

void Extinguish::poll() {
  // TODO: poll phototransistors to detect when the fire LED goes out (early exit).
  // After up to 10 s or once the LED is out, increment _fires_out and switch
  // back to SEARCH (or STOPPED after the second fire).
  if (millis() - _start_millis >= FAN_MAX_MS) {
    _fires_out++;
    if (_fires_out >= 2) {
      firefighter_->switchState(State::STOPPED);
    } else {
      firefighter_->switchState(State::SEARCH);
    }
  }
}

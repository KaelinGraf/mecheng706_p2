#include "Arduino.h"
#include "approach.h"
#include "firefighter.h"

void Approach::begin() {
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
}

void Approach::begin(StateData data) {
  _bearing = data.param;
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
}

void Approach::end() {
}

void Approach::poll() {
  // TODO: drive toward the fire bearing while watching IRs for obstacles.
  // Switch to AVOID if blocked, EXTINGUISH once within 20 cm of the fire centre.
}

#include "Arduino.h"
#include "search.h"
#include "firefighter.h"

void Search::begin() {
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
  // TODO: initialise wander pattern
}

void Search::end() {
}

void Search::poll() {
  // TODO: drive a wander pattern; check phototransistors for fire bearing;
  // check IR/ultrasonic for obstacles; switch to APPROACH or AVOID accordingly.
}

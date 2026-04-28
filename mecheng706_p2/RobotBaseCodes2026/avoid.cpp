#include "Arduino.h"
#include "avoid.h"
#include "firefighter.h"

void Avoid::begin() {
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
  // TODO: snapshot which sensor triggered avoidance
}

void Avoid::end() {
}

void Avoid::poll() {
  // TODO: strafe / rotate to clear the obstacle; bear in mind 1-2 obstacles
  // may be moving toward the robot. Return to SEARCH when path is clear.
}

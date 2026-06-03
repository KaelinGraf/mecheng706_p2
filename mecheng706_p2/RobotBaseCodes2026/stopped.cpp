#include "Arduino.h"
#include "stopped.h"
#include "firefighter.h"

void Stopped::begin() {
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
  Serial.println("STOPPED: Completed Fire Figthing!!");
}

void Stopped::end() {
}

void Stopped::poll() {
  // Stay stopped. Project 2 requires the robot to cease all movement
  // immediately after extinguishing the second fire.
  firefighter_->_motors->writeAllMotors(0.0, 0.0, 0.0);
  delay(100);
}

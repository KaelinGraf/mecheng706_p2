#include "servo_control.h"

void Motor::writeMotor(uint16_t microseconds){
  _motor.writeMicroseconds(int(clip<uint16_t>(microseconds,min_duty_motor,max_duty_motor)));
}

void Motor::writeMotor(float vx, float vy, float vtheta){
  float control_effort_sum;
  control_effort_sum = vx * _control_multipliers[0] + vy * _control_multipliers[1] + vtheta * _control_multipliers[2] ;

  _motor.writeMicroseconds(scaleMotor(control_effort_sum, -300.0, 300.0));
}

uint16_t Motor::scaleMotor(float control_effort_sum, float output_min, float output_max){
  if (output_max <= output_min) {
    return neutral;
  }
  if (control_effort_sum > output_max) control_effort_sum = output_max;
  if (control_effort_sum < output_min) control_effort_sum = output_min;
  float proportion = (control_effort_sum - output_min) / (output_max - output_min);
  float pwm_float = (proportion * (max_duty_motor - min_duty_motor)) + min_duty_motor;
  return static_cast<uint16_t>(clip<uint16_t>(pwm_float, min_duty_motor, max_duty_motor));
}



void Turret::attach(){
  attachMotor();
}

void Turret::detach(){
  _motor.detach();
  _is_attatched = false;
}

void Turret::pollState(){
  if (!locked_on_) {
    pan_scan(millis());
  }
}

bool Turret::atFire(){
  //return (middle_left > 3) && (middle_right > 3)
  return false;
}

void Turret::writeAngle(int angle){
  if (!_is_attatched) attachMotor();
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  this->angle_ = angle;
  _motor.write(angle);
}

void Turret::writeUS(uint16_t microseconds){
  if (!_is_attatched) attachMotor();
  _motor.writeMicroseconds(int(clip<uint16_t>(microseconds,min_duty_turret,max_duty_turret)));
}

void Turret::center(){
  // Center using neutral duty for small turret
  writeAngle(90);
}

void Turret::pan_scan(unsigned long current_time_ms){
  // Simple pan-scan behavior when not locked on
  // Sweeps between 0 and 180 degrees at a fixed speed
  const unsigned long scan_period_ms = 8000; // Time for a full sweep (back and forth)
  const int scan_range_deg = 360; // Total range of motion
  const int scan_speed_deg_per_sec = (scan_range_deg * 2) / (scan_period_ms / 1000.0); // Degrees per second

  // Calculate the angle based on the current time
  float angle = (current_time_ms % scan_period_ms) / static_cast<float>(scan_period_ms) * scan_range_deg;
  if ((current_time_ms / (scan_period_ms / 2)) % 2 == 1) {
    angle = scan_range_deg - angle; // Reverse direction on the second half of the period
  }
  
  writeAngle(static_cast<int>(angle));
}

void driveMotors::writeAllMotors(float vx, float vy, float vtheta){
  _left_front_motor.writeMotor(vx,vy,vtheta);
  _left_rear_motor.writeMotor(vx,vy,vtheta);
  _right_front_motor.writeMotor(vx,vy,vtheta);
  _right_rear_motor.writeMotor(vx,vy,vtheta);
}

void driveMotors::attatchAll(){
  _left_front_motor.attachMotor();
  _left_rear_motor.attachMotor();
  _right_front_motor.attachMotor();
  _right_rear_motor.attachMotor();
}


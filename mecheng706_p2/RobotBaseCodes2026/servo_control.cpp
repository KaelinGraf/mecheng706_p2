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
  _motor.attach(_motor_pin);
  _is_attatched = true;
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
  return (((_fb->_sl->getFilteredV() > 4.8) || (_fb->_sr->getFilteredV() > 4.8)) && (_fb->_l->getFilteredV() + _fb->_r->getFilteredV() >= 1.0));
}

void Turret::writeAngle(int angle){
  if (!_is_attatched) attachMotor();
  if (angle < (SERVO_CENTER - 60)) angle = SERVO_CENTER - 60;
  if (angle > (SERVO_CENTER + 60)) angle = SERVO_CENTER + 60;
  this->angle_ = angle;
  _motor.write(angle);
}

void Turret::center(){
  // Center using neutral duty for small turret
  writeAngle(SERVO_CENTER);
}

void Turret::pan_scan(unsigned long current_time_ms){
  // Time for a full back-and-forth sweep
  const unsigned long scan_period_ms = 5000; 
  const unsigned long half_period_ms = scan_period_ms / 2;
  
  // Set boundaries
  const int min_angle = 30;
  const int max_angle = 150;
  const int scan_range_deg = max_angle - min_angle; // 160 degrees

  // Calculate our exact position within the current half-period (0.0 to 1.0)
  unsigned long time_in_half = current_time_ms % half_period_ms;
  float proportion = static_cast<float>(time_in_half) / half_period_ms;

  float angle;
  // Determine if we are on the forward or backward stroke
  if ((current_time_ms / half_period_ms) % 2 == 0) {
    // Forward sweep: 10 -> 170
    angle = min_angle + (proportion * scan_range_deg);
  } else {
    // Reverse sweep: 170 -> 10
    angle = max_angle - (proportion * scan_range_deg);
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


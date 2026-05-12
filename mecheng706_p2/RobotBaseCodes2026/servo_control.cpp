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

#include "servo_control.h"


void Motor::writeMotor(uint16_t microseconds) {
    if (!_hw) return;
    _hw->write_us(_channel,
                  clip<uint16_t>(microseconds, min_duty_motor, max_duty_motor));
}

void Motor::writeMotor(float vx, float vy, float vtheta) {
    if (!_hw || !_control_multipliers) return;
    float control_effort_sum = vx * _control_multipliers[0]
                             + vy * _control_multipliers[1]
                             + vtheta * _control_multipliers[2];
    _hw->write_us(_channel, scaleMotor(control_effort_sum, -300.0f, 300.0f));
}

void Motor::stopMotor() {
    if (_hw) _hw->write_us(_channel, neutral);
}

uint16_t Motor::scaleMotor(float control_effort_sum, float output_min, float output_max) {
    if (output_max <= output_min) {
        return neutral;
    }
    if (control_effort_sum > output_max) control_effort_sum = output_max;
    if (control_effort_sum < output_min) control_effort_sum = output_min;
    float proportion = (control_effort_sum - output_min) / (output_max - output_min);
    float pwm_float = (proportion * (max_duty_motor - min_duty_motor)) + min_duty_motor;
    return static_cast<uint16_t>(clip<uint16_t>((uint16_t)pwm_float, min_duty_motor, max_duty_motor));
}


void turret::writeMotor(int angle) {
    if (!_hw) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // Standard hobby-servo mapping: 0..180 deg -> min_duty_turret..max_duty_turret.
    uint16_t pw = (uint16_t)(min_duty_turret + (uint32_t)(max_duty_turret - min_duty_turret) * (uint32_t)angle / 180UL);
    _hw->write_us(_channel, pw);
}


void driveMotors::writeAllMotors(float vx, float vy, float vtheta) {
    _left_front_motor.writeMotor(vx, vy, vtheta);
    _left_rear_motor.writeMotor(vx, vy, vtheta);
    _right_front_motor.writeMotor(vx, vy, vtheta);
    _right_rear_motor.writeMotor(vx, vy, vtheta);
}

void driveMotors::attatchAll() {
    _left_front_motor.attachMotor();
    _left_rear_motor.attachMotor();
    _right_front_motor.attachMotor();
    _right_rear_motor.attachMotor();
}

void driveMotors::writeMotor(motorName target_motor, uint16_t speed) {
    switch (target_motor) {
        case left_font:    _left_front_motor.writeMotor(speed); break;
        case left_rear:    _left_rear_motor.writeMotor(speed); break;
        case right_rear:   _right_rear_motor.writeMotor(speed); break;
        case right_front:  _right_front_motor.writeMotor(speed); break;
    }
}

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <Servo.h>
#include "mappings.h"
#include "utils.h"
#include "Arduino.h"

const int chassis_scale = (L_len+l_len);


const int left_front_multis[3] = {-1, 1, 1};

const int right_front_multis[3] = {1, 1, 1};

const int left_rear_multis[3] = {-1, -1,1};

const int right_rear_multis[3] = {1, -1,1};


enum motorName{
  left_font,
  left_rear,
  right_rear,
  right_front,
};
class Motor{
  protected:
    Servo _motor;
    const int* _control_multipliers; //array of i.e [1,-1,1] for front right motor (multipliers for control effort Vx Vy Vtheta)
    uint8_t _motor_pin;
    bool _is_attatched = false;


  public:
    Motor(const int* control_multipliers,uint8_t motor_pin):_control_multipliers(control_multipliers),_motor_pin(motor_pin){
    }
    Motor(uint8_t motor_pin){
      _motor_pin = motor_pin;
    }
    void attachMotor(){
      _motor.attach(_motor_pin);
      _is_attatched = true;
    }
    ~Motor(){
      _motor.detach();
    }
    virtual void writeMotor(uint16_t microseconds);
    void writeMotor(float vx, float vy, float vtheta);
    void stopMotor();
    uint16_t scaleMotor(float control_effor_sum, float output_min, float output_max); //output min and output max refer to the ranges of the summed control effort.
};

class driveMotors{
  private:
    Motor _left_front_motor;
    Motor _left_rear_motor;
    Motor _right_front_motor;
    Motor _right_rear_motor;

  public:
    driveMotors():_left_front_motor(left_front_multis, left_front_pin),_left_rear_motor(left_rear_multis, left_rear_pin),_right_front_motor(right_front_multis, right_front_pin),_right_rear_motor(right_rear_multis, right_rear_pin){

    }
    void writeMotor(motorName target_motor, uint16_t speed);
    void writeAllMotors(float vx, float vy, float vtheta);
    void attatchAll();

};




// Turret (SG90) wrapper class providing a simple, safe API.
class Turret : public Motor {
  public:
    int angle_ = 90; // Current angle in degrees (0-180)
    bool locked_on_ = true; // Whether the turret is facing fire

    Turret(uint8_t motor_pin = turret_pin) : Motor(motor_pin) {};
    void pollState();
    void attach();
    void detach();
    void writeAngle(int angle); // degrees 0-180
    void writeUS(uint16_t microseconds); // write in microseconds (clipped to turret range)
    void center();
    void lockOn(bool locked_on) {
        locked_on_ = locked_on;
    }
    void pan_scan(unsigned long current_time_ms); // simple pan-scan behavior when not locked on
};





#endif

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <Servo.h>
#include "mappings.h"
#include "utils.h"

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




class turret : public Motor{
  public:
    turret(uint8_t motor_pin):Motor(motor_pin){};
    void writeMotor(int angle);

};





#endif

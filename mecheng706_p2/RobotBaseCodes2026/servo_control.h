#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <Servo.h>
#include "mappings.h"
#include "utils.h"
#include "Arduino.h"
#include "fire.h"

const int chassis_scale = (L_len+l_len);


// Mecanum mix multipliers [vx, vy, vtheta] per wheel. THIS IS THE SINGLE SOURCE OF
// TRUTH for drive direction. The vx column (first entry of each row) is set so that
// +vx = FORWARD. (The committed values had +vx = backward, so every caller used to
// undo it with a -vx; those compensations were removed in the SAME pass — see
// tracking.cpp, extinguish.cpp, and the dead-reckoner in pose.cpp.) Convention is
// now uniform everywhere: +vx = forward, +vy = strafe right.
const int left_front_multis[3] = {1, 1, 1};

const int right_front_multis[3] = {-1, 1, 1};

const int left_rear_multis[3] = {1, -1,1};

const int right_rear_multis[3] = {-1, -1,1};


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

    struct last_motor_commands{
      float vx = 0.0f;
      float vy = 0.0f;
      float vtheta = 0.0f;
    } last_commands;


  public:
    driveMotors():_left_front_motor(left_front_multis, left_front_pin),_left_rear_motor(left_rear_multis, left_rear_pin),_right_front_motor(right_front_multis, right_front_pin),_right_rear_motor(right_rear_multis, right_rear_pin){

    }
    void writeMotor(motorName target_motor, uint16_t speed);
    void writeAllMotors(float vx, float vy, float vtheta);
    void attatchAll();

    // Last commanded body-frame velocities, latched on every writeAllMotors().
    // This is the single source of truth for dead-reckoning's motion model.
    const last_motor_commands& getLastCommand() const { return last_commands; }

};






// Turret (SG90) wrapper class providing a simple, safe API.
class Turret {
  private:
    FireBank *_fb;
    Servo _motor;
    int _motor_pin;
    bool _is_attatched;
  public:
    int angle_ = 90; // Current angle in degrees (0-180)
    bool locked_on_ = false; // Whether the turret is facing fire

    Turret(FireBank *fb, uint8_t motor_pin = turret_pin) :_fb(fb), _motor_pin(motor_pin) {};
    void pollState();
    void attach();
    void detach();
    void writeAngle(int angle); // degrees 0-180
    void writeUS(uint16_t microseconds); // write in microseconds (clipped to turret range)
    void center();
    void lockOn(bool locked_on) {
        locked_on_ = locked_on;
    }
    bool atFire();
    void pan_scan(unsigned long current_time_ms); // simple pan-scan behavior when not locked on
};





#endif

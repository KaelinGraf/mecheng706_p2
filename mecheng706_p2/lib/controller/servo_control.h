#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include "mappings.h"
#include "utils.h"
#include "imotor_output.h"

const int chassis_scale = (L_len + l_len);

const int left_front_multis[3] = {-1, 1, 1};
const int right_front_multis[3] = {1, 1, 1};
const int left_rear_multis[3]   = {-1, -1, 1};
const int right_rear_multis[3]  = {1, -1, 1};

enum motorName {
    left_font,    // (sic) historical typo - preserved as part of public API
    left_rear,
    right_rear,
    right_front,
};

// ---------------------------------------------------------------------------
// Motor: owns the per-wheel control mixing math (multiplier dot-product on
// vx/vy/vtheta) and emits a microsecond pulse to its assigned channel via
// the IMotorOutput backend.
// ---------------------------------------------------------------------------
class Motor {
protected:
    IMotorOutput* _hw;
    const int* _control_multipliers;  // i.e. {1,-1,1} for vx,vy,vtheta scaling
    uint8_t _motor_pin;
    uint8_t _channel;
    bool _is_attached = false;

public:
    Motor(IMotorOutput* hw, const int* control_multipliers, uint8_t channel, uint8_t motor_pin)
        : _hw(hw), _control_multipliers(control_multipliers),
          _motor_pin(motor_pin), _channel(channel) {}

    Motor(IMotorOutput* hw, uint8_t channel, uint8_t motor_pin)
        : _hw(hw), _control_multipliers(nullptr),
          _motor_pin(motor_pin), _channel(channel) {}

    void attachMotor() {
        if (_hw) _hw->attach(_channel, _motor_pin);
        _is_attached = true;
    }

    virtual void writeMotor(uint16_t microseconds);
    void writeMotor(float vx, float vy, float vtheta);
    void stopMotor();
    uint16_t scaleMotor(float control_effort_sum, float output_min, float output_max);
};


// ---------------------------------------------------------------------------
// driveMotors: composition of the four mecanum-wheel motors. Public API
// (writeAllMotors, attatchAll) is unchanged; the only change is that the
// constructor needs an IMotorOutput backend to wire each Motor to.
// ---------------------------------------------------------------------------
class driveMotors {
private:
    Motor _left_front_motor;
    Motor _left_rear_motor;
    Motor _right_front_motor;
    Motor _right_rear_motor;

public:
    driveMotors(IMotorOutput* hw)
        : _left_front_motor(hw, left_front_multis, left_font,    left_front_pin),
          _left_rear_motor (hw, left_rear_multis,  left_rear,    left_rear_pin),
          _right_front_motor(hw, right_front_multis, right_front, right_front_pin),
          _right_rear_motor(hw, right_rear_multis,  right_rear,   right_rear_pin) {}

    void writeMotor(motorName target_motor, uint16_t speed);
    void writeAllMotors(float vx, float vy, float vtheta);
    void attatchAll();
};


// ---------------------------------------------------------------------------
// turret: small servo. Currently defined for completeness; not instantiated
// by the FireFighter today. Converts an angle (0..180) into a pulse width.
// ---------------------------------------------------------------------------
class turret : public Motor {
public:
    using Motor::writeMotor;  // unhide the inherited (uint16_t) and (vx,vy,vtheta) overloads

    turret(IMotorOutput* hw, uint8_t channel, uint8_t motor_pin)
        : Motor(hw, channel, motor_pin) {}

    void writeMotor(int angle);
};


#endif

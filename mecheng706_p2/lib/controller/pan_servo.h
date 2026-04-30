#ifndef PAN_SERVO_H
#define PAN_SERVO_H

#include <stdint.h>
#include "imotor_output.h"

// Hobby-servo wrapper for the turret pan axis. Wraps the same IMotorOutput
// HAL channel pool used by the drive motors (channel reserved for the
// turret), exposing a radian-based angle interface clamped to a configurable
// mechanical range.
//
// Conventions: positive angle rotates the turret CCW relative to the body
// forward axis (matches the IMU yaw sign convention used elsewhere). A
// non-zero mount_offset_rad accounts for the servo not being installed
// pointed straight ahead at its mechanical centre.
class PanServo {
public:
    PanServo(IMotorOutput* hw, uint8_t channel, uint8_t pin,
             float min_rad = -1.5707963f,    // -90 deg
             float max_rad =  1.5707963f,    // +90 deg
             float mount_offset_rad = 0.0f,
             uint16_t pulse_min_us = 900,
             uint16_t pulse_max_us = 2100);

    // One-time servo attach. Safe to call multiple times.
    void attach();

    // Drive the servo to the given body-frame angle (rad). Clamps to the
    // configured [min_rad, max_rad] range; the mount offset is applied
    // internally so the caller always works in the body frame.
    void write_rad(float angle_rad);

    // Last commanded body-frame angle (rad).
    inline float current_rad() const { return _current_rad; }

    inline float min_rad() const { return _min_rad; }
    inline float max_rad() const { return _max_rad; }

private:
    IMotorOutput* _hw;
    uint8_t  _channel;
    uint8_t  _pin;
    float    _min_rad;
    float    _max_rad;
    float    _mount_offset_rad;
    uint16_t _pulse_min_us;
    uint16_t _pulse_max_us;
    float    _current_rad;
    bool     _attached;
};

#endif

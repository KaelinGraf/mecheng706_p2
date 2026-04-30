#ifndef HAL_IMOTOR_OUTPUT_H
#define HAL_IMOTOR_OUTPUT_H

#include <stdint.h>

// Per-channel motor PWM output. The driveMotors / Motor classes still own
// the kinematics math (mecanum mixing, scaling); this interface just emits
// a pulse-width to a logical motor channel. On Arduino it wraps Servo's
// writeMicroseconds(); on host it captures the value for the sim to read.
class IMotorOutput {
public:
    virtual ~IMotorOutput() = default;

    // Attach a channel (one-time setup before any writes). On Arduino this
    // calls Servo::attach(pin); on host it just records the pin so the sim
    // can address it.
    virtual void attach(uint8_t channel, uint8_t pin) = 0;

    // Send a pulse-width (microseconds) to a channel. Range typically
    // [min_duty_motor, max_duty_motor] (700..2300us, neutral 1500us).
    virtual void write_us(uint8_t channel, uint16_t microseconds) = 0;

    // Read back the most recent pulse-width that was written to a channel.
    // Used primarily by sim consumers to observe the controller's commands.
    virtual uint16_t last_us(uint8_t channel) const = 0;
};

#endif

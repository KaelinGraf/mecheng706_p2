#ifndef HAL_ARDUINO_MOTOR_OUTPUT_H
#define HAL_ARDUINO_MOTOR_OUTPUT_H

#include <Servo.h>
#include "imotor_output.h"

// IMotorOutput backed by Arduino's Servo library (writeMicroseconds).
// Supports the four mecanum drive channels plus an optional fifth turret
// channel; size MAX_CHANNELS up if more are needed.
class ArduinoMotorOutput : public IMotorOutput {
public:
    static const uint8_t MAX_CHANNELS = 8;

    void attach(uint8_t channel, uint8_t pin) override;
    void write_us(uint8_t channel, uint16_t microseconds) override;
    uint16_t last_us(uint8_t channel) const override;

private:
    Servo _servos[MAX_CHANNELS];
    bool _attached[MAX_CHANNELS] = {false, false, false, false, false, false, false, false};
    uint16_t _last_us[MAX_CHANNELS] = {0,0,0,0,0,0,0,0};
};

#endif

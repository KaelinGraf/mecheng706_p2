#include "arduino_motor_output.h"

void ArduinoMotorOutput::attach(uint8_t channel, uint8_t pin) {
    if (channel >= MAX_CHANNELS) return;
    if (!_attached[channel]) {
        _servos[channel].attach(pin);
        _attached[channel] = true;
    }
}

void ArduinoMotorOutput::write_us(uint8_t channel, uint16_t microseconds) {
    if (channel >= MAX_CHANNELS) return;
    if (!_attached[channel]) return;
    _servos[channel].writeMicroseconds(microseconds);
    _last_us[channel] = microseconds;
}

uint16_t ArduinoMotorOutput::last_us(uint8_t channel) const {
    if (channel >= MAX_CHANNELS) return 0;
    return _last_us[channel];
}

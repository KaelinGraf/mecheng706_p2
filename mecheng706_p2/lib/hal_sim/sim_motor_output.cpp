#include "sim_motor_output.h"

SimMotorOutput::SimMotorOutput() {
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        _pin[i] = 0;
        _last_us[i] = 0;
        _attached[i] = false;
    }
}

void SimMotorOutput::attach(uint8_t channel, uint8_t pin) {
    if (channel >= MAX_CHANNELS) return;
    _pin[channel] = pin;
    _attached[channel] = true;
}

void SimMotorOutput::write_us(uint8_t channel, uint16_t microseconds) {
    if (channel >= MAX_CHANNELS) return;
    _last_us[channel] = microseconds;
}

uint16_t SimMotorOutput::last_us(uint8_t channel) const {
    if (channel >= MAX_CHANNELS) return 0;
    return _last_us[channel];
}

uint8_t SimMotorOutput::pin_for(uint8_t channel) const {
    if (channel >= MAX_CHANNELS) return 0;
    return _pin[channel];
}

#include "sim_analog_input.h"

SimAnalogInput::SimAnalogInput() {
    for (int i = 0; i < N_PINS; ++i) _voltages[i] = 0.0f;
}

float SimAnalogInput::read_voltage(uint8_t pin) {
    return _voltages[pin];
}

void SimAnalogInput::set_voltage(uint8_t pin, float volts) {
    _voltages[pin] = volts;
}

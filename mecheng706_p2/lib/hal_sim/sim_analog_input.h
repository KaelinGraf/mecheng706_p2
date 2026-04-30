#ifndef HAL_SIM_ANALOG_INPUT_H
#define HAL_SIM_ANALOG_INPUT_H

#include "ianalog_input.h"

// IAnalogInput backed by a flat per-pin voltage table the sim writes into.
// Indexed by raw Arduino pin number (0..255). Sim consumers (Python via the
// pybind11 wrapper, or a C++ test harness) call set_voltage(pin, v) before
// each controller tick.
class SimAnalogInput : public IAnalogInput {
public:
    SimAnalogInput();

    float read_voltage(uint8_t pin) override;
    void set_voltage(uint8_t pin, float volts);

private:
    static const int N_PINS = 256;
    float _voltages[N_PINS];
};

#endif

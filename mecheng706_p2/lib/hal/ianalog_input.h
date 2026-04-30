#ifndef HAL_IANALOG_INPUT_H
#define HAL_IANALOG_INPUT_H

#include <stdint.h>

// Single ADC channel read. The controller's sensor classes call into this
// instead of analogRead() so the same calibration / filtering code runs
// against either real ADC samples or sim-supplied voltages.
class IAnalogInput {
public:
    virtual ~IAnalogInput() = default;

    // Returns the voltage on `pin` in volts (0 .. ADC reference, typically 5.0).
    // Implementations are responsible for ADC scaling.
    virtual float read_voltage(uint8_t pin) = 0;
};

#endif

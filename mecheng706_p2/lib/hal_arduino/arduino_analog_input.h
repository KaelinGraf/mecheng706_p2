#ifndef HAL_ARDUINO_ANALOG_INPUT_H
#define HAL_ARDUINO_ANALOG_INPUT_H

#include "ianalog_input.h"

class ArduinoAnalogInput : public IAnalogInput {
public:
    // Reads an analog pin via analogRead() and scales the resulting count
    // (0..ADC_RANGE-1) to volts using the V_ADC reference.
    float read_voltage(uint8_t pin) override;
};

#endif

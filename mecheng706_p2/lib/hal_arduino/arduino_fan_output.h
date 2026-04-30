#ifndef HAL_ARDUINO_FAN_OUTPUT_H
#define HAL_ARDUINO_FAN_OUTPUT_H

#include <stdint.h>
#include "ifan_output.h"

// MOSFET-gate fan driver with a soft-start ramp. Implements the original
// Fan::on() behaviour: ~250ms PWM ramp before pinning the gate HIGH so the
// inrush current doesn't brown out the 5V regulator.
class ArduinoFanOutput : public IFanOutput {
public:
    explicit ArduinoFanOutput(uint8_t pin);

    void begin() override;
    void on() override;
    void off() override;

private:
    uint8_t _pin;
};

#endif

#ifndef HAL_ARDUINO_ULTRASONIC_SOURCE_H
#define HAL_ARDUINO_ULTRASONIC_SOURCE_H

#include <stdint.h>
#include "iultrasonic_source.h"

// HC-SR04 source backed by a free-running INT4 ISR (ultrasonic echo line is
// wired to digital pin 2 / INT4 on the Mega). The trigger pin is held high
// for ~10us by trigger(); the ISR captures rising/falling timestamps and the
// pulse-width is computed from them.
//
// The class registers itself in a global pointer so the ISR (which has to be
// a C-style free function tied to the AVR vector) can dispatch into it.
class ArduinoUltrasonicSource : public IUltrasonicSource {
public:
    ArduinoUltrasonicSource(uint8_t trigger_pin, uint8_t echo_pin);

    // One-time setup: configures the trigger pin as output, enables INT4 for
    // both edges, and arms the global ISR pointer. Idempotent.
    void begin();

    void trigger() override;
    unsigned long last_pulse_width_us() override;

    // Called by the ISR to update timestamps. Public so the free-function
    // ISR can reach it without making the ISR a friend.
    void on_echo_isr();

private:
    uint8_t _trigger_pin;
    uint8_t _echo_pin;

    // Captured timestamps - written by the ISR, read by last_pulse_width_us.
    volatile unsigned long _t_rise;
    volatile unsigned long _t_fall;
    volatile unsigned long _t_last_rise;
};

#endif

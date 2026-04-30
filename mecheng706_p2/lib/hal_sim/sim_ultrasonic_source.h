#ifndef HAL_SIM_ULTRASONIC_SOURCE_H
#define HAL_SIM_ULTRASONIC_SOURCE_H

#include "iultrasonic_source.h"

// Sim ultrasonic source. The host pushes the simulated echo pulse-width
// (microseconds) corresponding to the simulated ray distance. The sensor
// class converts this to cm via the same /58.0 formula used on Arduino.
class SimUltrasonicSource : public IUltrasonicSource {
public:
    SimUltrasonicSource() : _pulse_us(0) {}

    void trigger() override {}                 // sim is always up to date
    unsigned long last_pulse_width_us() override { return _pulse_us; }

    // Push a new echo pulse-width. Convenience helper:
    //   push_distance_cm(d) sets pulse_us = d * 58.0.
    void push_pulse_us(unsigned long pulse_us) { _pulse_us = pulse_us; }
    void push_distance_cm(float distance_cm)   { _pulse_us = (unsigned long)(distance_cm * 58.0f); }

private:
    unsigned long _pulse_us;
};

#endif

#ifndef HAL_IULTRASONIC_SOURCE_H
#define HAL_IULTRASONIC_SOURCE_H

#include <stdint.h>

// HC-SR04 ultrasonic ranging source. The Ultrasonic class consumes
// echo-pulse timestamps and converts them to distance; the source is
// responsible for triggering pings and measuring echo width. On the Arduino
// target this is driven by the INT4 ISR plus a periodic trigger; on the
// host target the sim pushes pulse widths corresponding to the simulated
// world geometry.
class IUltrasonicSource {
public:
    virtual ~IUltrasonicSource() = default;

    // Issue a fresh trigger pulse (10us high on the trigger line on Arduino;
    // a no-op on sim, since the sim updates pulse widths synchronously).
    virtual void trigger() = 0;

    // Most recent echo pulse-width in microseconds, derived from the rising
    // and falling edges of the echo line. Implementations cache the latest
    // valid value; out-of-range / no-echo conditions are signalled by
    // returning >= max_dist (which the controller already treats as OOR).
    virtual unsigned long last_pulse_width_us() = 0;
};

#endif

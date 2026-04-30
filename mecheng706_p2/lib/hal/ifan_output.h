#ifndef HAL_IFAN_OUTPUT_H
#define HAL_IFAN_OUTPUT_H

// Fan / extinguisher driver. The Fan class still owns the high-level
// "is the fan currently running and for how long" state; this interface
// just toggles the underlying actuator. On Arduino this drives a MOSFET
// gate with a soft-start ramp; on host it just flips a boolean the sim
// can query.
class IFanOutput {
public:
    virtual ~IFanOutput() = default;

    // Configure the underlying pin / channel and ensure the fan starts off.
    // Idempotent - safe to call repeatedly.
    virtual void begin() = 0;

    // Engage the fan. Implementations may soft-start (Arduino) or apply
    // immediately (sim); blocking time is implementation-specific.
    virtual void on() = 0;

    // Disengage the fan. Idempotent.
    virtual void off() = 0;
};

#endif

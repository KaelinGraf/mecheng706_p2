#ifndef HAL_SIM_MOTOR_OUTPUT_H
#define HAL_SIM_MOTOR_OUTPUT_H

#include "imotor_output.h"

// Sim motor output. Captures the last microsecond pulse-width per channel
// so the sim can pull commands out via last_us(channel).
class SimMotorOutput : public IMotorOutput {
public:
    static const uint8_t MAX_CHANNELS = 8;

    SimMotorOutput();

    void attach(uint8_t channel, uint8_t pin) override;
    void write_us(uint8_t channel, uint16_t microseconds) override;
    uint16_t last_us(uint8_t channel) const override;

    // Sim convenience: which Arduino pin was attached to a channel (0 if
    // never attached).
    uint8_t pin_for(uint8_t channel) const;

private:
    uint8_t _pin[MAX_CHANNELS];
    uint16_t _last_us[MAX_CHANNELS];
    bool _attached[MAX_CHANNELS];
};

#endif

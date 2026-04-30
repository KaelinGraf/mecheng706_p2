#ifndef HAL_SIM_FAN_OUTPUT_H
#define HAL_SIM_FAN_OUTPUT_H

#include "ifan_output.h"

// Sim fan: just a boolean. The sim consumer queries is_on() to decide
// whether to apply the extinguishing effect to a fire in range.
class SimFanOutput : public IFanOutput {
public:
    SimFanOutput() : _on(false) {}

    void begin() override { _on = false; }
    void on() override    { _on = true; }
    void off() override   { _on = false; }

    bool is_on() const { return _on; }

private:
    bool _on;
};

#endif

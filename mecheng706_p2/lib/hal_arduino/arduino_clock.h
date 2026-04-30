#ifndef HAL_ARDUINO_CLOCK_H
#define HAL_ARDUINO_CLOCK_H

#include "iclock.h"

class ArduinoClock : public IClock {
public:
    unsigned long now_ms() override;
    unsigned long now_us() override;
};

#endif

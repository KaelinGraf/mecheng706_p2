#ifndef HAL_ICLOCK_H
#define HAL_ICLOCK_H

#include <stdint.h>

// Time source. Replaces direct calls to millis() / micros() so the controller
// can be driven by a sim-controlled clock during host builds.
class IClock {
public:
    virtual ~IClock() = default;
    virtual unsigned long now_ms() = 0;
    virtual unsigned long now_us() = 0;
};

#endif

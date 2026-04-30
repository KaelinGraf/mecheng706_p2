#ifndef SFH300FA_H
#define SFH300FA_H

#include "fire.h"

// OSRAM SFH 300 FA — IR-filtered NPN silicon phototransistor in a 5 mm
// radial T-1 3/4 epoxy package (datasheet 2014-01-14 v1.1).
//
// Spectral range: 730 - 1120 nm, peak sensitivity ~880 nm. The integrated
// daylight blocking filter rejects most visible light, so this part is
// well-suited to picking out an IR-emitting fire LED against ambient
// fluorescent/LED lab lighting.
//
// Wiring matches the existing Phototransistor base: emitter to GND through
// a pull-down resistor (default 10 kOhm gives ~1 V at 100 uA collector
// current, comfortably inside the Mega's 0-5 V ADC range), collector to
// V+, collector-pin tap into an analog input.
//
// The slower default EWMA (alpha = 0.25 vs the visible Phototransistor's
// 0.40) reflects the SFH 300 FA's better noise floor when its IR signal
// is the dominant component — we can afford a longer time-constant for
// cleaner bearing estimates from the turret photo array.
class SFH300FA : public Phototransistor {
public:
    SFH300FA(IAnalogInput* adc, uint8_t read_pin,
             float pull_down_kohm = 10.0f,
             float alpha = 0.25f)
        : Phototransistor(adc, read_pin, alpha),
          _pull_down_kohm(pull_down_kohm) {}

    inline float pullDownKOhm() const { return _pull_down_kohm; }

private:
    float _pull_down_kohm;
};

#endif

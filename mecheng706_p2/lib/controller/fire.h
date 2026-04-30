#ifndef FIRE_H
#define FIRE_H

#include <stdint.h>
#include "sensors.h"
#include "mappings.h"
#include "utils.h"
#include "ifan_output.h"
#include "iclock.h"

// ---------------------------------------------------------------------------
// Phototransistor: simple analog light sensor used to detect a fire LED.
//
// Wiring assumption (the standard pull-down circuit):
//   5V --- Phototransistor (collector) ---+--- Analog pin
//                                         |
//                                         R (e.g. 10k) -- GND
// ADC voltage rises as more light hits the device.
// ---------------------------------------------------------------------------
class Phototransistor : public Sensor {
protected:
    float _filtered_v = 0.0f;
    float _alpha = 0.4f;
    bool _initialised = false;

public:
    Phototransistor(IAnalogInput* adc, uint8_t read_pin, float alpha = 0.4f)
        : Sensor(adc, read_pin) { _alpha = alpha; (void)0; }
    // ^ explicit body assignment (instead of init list) so the in-class
    //   `_alpha = 0.4f` default still serves as documentation; the ctor
    //   parameter overrides it for callers that want a different smoothing.

    float readSensor() override;
    inline float applyCalibration(float adc_voltage) override { return adc_voltage; }
    inline float getFilteredV() const { return _filtered_v; }
    inline bool detected(float threshold = FIRE_DETECT_V) const {
        return _initialised && _filtered_v >= threshold;
    }
    inline void reset() {
        _filtered_v = 0.0f;
        _initialised = false;
        mapping_reading_ = 0.0f;
    }
};


// The body-cardinal FireBank that the legacy FSM used has been retired:
// the panning turret's PhotoArray (lib/controller/photo_array.h) now owns
// the "is there a fire and in which direction" question, with the boom-
// mounted SFH 300 FA photodiodes giving a much sharper bearing.

// ---------------------------------------------------------------------------
// Fan: extinguisher actuator wrapper. Owns the "on/off" state plus timing;
// the actual MOSFET-gate driving (or sim toggling) is delegated to an
// IFanOutput backend and an IClock.
// ---------------------------------------------------------------------------
class Fan {
private:
    IFanOutput* _hw;
    IClock* _clock;
    bool _on = false;
    unsigned long _on_started_ms = 0;

public:
    Fan(IFanOutput* hw, IClock* clock) : _hw(hw), _clock(clock) {}

    void begin();
    void on();
    void off();

    inline bool isOn() const { return _on; }
    inline unsigned long msSinceOn() const {
        return (_on && _clock) ? (_clock->now_ms() - _on_started_ms) : 0;
    }
};

#endif

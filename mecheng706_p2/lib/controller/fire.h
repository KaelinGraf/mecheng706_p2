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
private:
    float _filtered_v = 0.0f;
    float _alpha = 0.4f;
    bool _initialised = false;

public:
    Phototransistor(IAnalogInput* adc, uint8_t read_pin) : Sensor(adc, read_pin) {}

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


// ---------------------------------------------------------------------------
// FireBank: aggregates the four cardinal phototransistors into a single
// object answering "is there a fire?" and "in roughly which direction?".
//
// Bearing convention:
//   bearing = 0 rad  -> straight ahead (+Y, FORWARD)
//   bearing > 0      -> CCW (toward the body LEFT)
//   bearing < 0      -> CW  (toward the body RIGHT)
// ---------------------------------------------------------------------------
class FireBank {
public:
    Phototransistor* front;
    Phototransistor* right;
    Phototransistor* rear;
    Phototransistor* left;

    FireBank(Phototransistor* f, Phototransistor* r, Phototransistor* b, Phototransistor* l)
        : front(f), right(r), rear(b), left(l) {}

    void update();
    float maxV() const;

    inline bool anyDetection(float threshold = FIRE_DETECT_V) const {
        return maxV() >= threshold;
    }

    bool allBelow(float threshold = FIRE_OUT_V) const;
    float estimateBearing(bool* valid = nullptr, float threshold = FIRE_DETECT_V) const;
    void reset();
};


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

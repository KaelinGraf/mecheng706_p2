#ifndef FIRE_H
#define FIRE_H

#include <stdint.h>
#include <Arduino.h>
#include "sensors.h"
#include "mappings.h"
#include "utils.h"

// ---------------------------------------------------------------------------
// Phototransistor: simple analog light sensor used to detect a fire LED.
//
// Wiring assumption (the standard pull-down circuit):
//   5V --- Phototransistor (collector) ---+--- Analog pin
//                                         |
//                                         R (e.g. 10k) -- GND
// ADC voltage rises as more light hits the device. The "fire" lamp puts out
// far more (and bluer) energy than ambient room light, so a fixed voltage
// threshold is enough to flag detection. We expose the raw voltage so the
// search/approach states can build a bearing from the four-cell array rather
// than using a binary flag per cell.
// ---------------------------------------------------------------------------
class Phototransistor : public Sensor {
  private:
    // EWMA filter on raw voltage. The fire LED + ambient noise can be jittery
    // at the ADC level; smoothing prevents single-sample spikes from causing
    // spurious state changes.
    float _filtered_v = 0.0f;
    float _alpha      = 0.4f;
    bool  _initialised = false;

  public:
    Phototransistor(uint8_t read_pin) : Sensor(read_pin) {}

    // Reads the ADC, updates the filter, and stores the smoothed value in
    // mapping_reading_ for the data-logger.
    float readSensor() override;

    // Pass-through (no calibration curve - we work in volts).
    inline float applyCalibration(float adc_voltage) override { return adc_voltage; }

    // Latest filtered ADC voltage (cached - does not re-read the pin).
    inline float getFilteredV() const { return _filtered_v; }

    // Convenience: did we see fire on the last read?
    inline bool detected(float threshold = FIRE_DETECT_V) const {
        return _initialised && _filtered_v >= threshold;
    }

    // Reset filter state (use when entering APPROACH/EXTINGUISH so a stale
    // baseline doesn't bias the new decision).
    inline void reset() {
        _filtered_v = 0.0f;
        _initialised = false;
        mapping_reading_ = 0.0f;
    }
};


// ---------------------------------------------------------------------------
// FireBank: aggregates the four cardinal phototransistors (front / right /
// rear / left) into a single object that can answer "is there a fire?" and
// "in roughly which direction?".
//
// Bearing model: photo intensity I_i at sensor i is loosely proportional to
// cos(theta_fire - theta_i) once the LED is in line-of-sight. Treating each
// reading as a vector of magnitude V_i pointing along its sensor axis and
// summing gives a coarse bearing that's good enough to steer toward.
//
// Convention used everywhere in this code:
//   bearing = 0 rad  -> straight ahead (+Y, FORWARD)
//   bearing > 0      -> CCW (toward the body LEFT)
//   bearing < 0      -> CW  (toward the body RIGHT)
// ---------------------------------------------------------------------------
class FireBank {
  private: 
    bool _angleValid;
  public:
    Phototransistor* _sl;
    Phototransistor* _l;
    Phototransistor* _r;
    Phototransistor* _sr;

    FireBank(Phototransistor* sl, Phototransistor* l, Phototransistor* r, Phototransistor* sr)
      : _sl(sl), _l(l), _r(r), _sr(sr) {}

    // Read all four cells. Call once per loop; getters below are non-blocking.
    void update();

    // Strongest single-cell voltage seen on the last update.
    float maxV() const;

    // True if the strongest cell exceeds the detect threshold.
    inline bool anyDetection(float threshold = FIRE_DETECT_V) const {
        return maxV() >= threshold;
    }

    // True if ALL cells are below the "out" threshold (used by EXTINGUISH to
    // declare the fire put out and bail early).
    bool allBelow(float threshold = FIRE_OUT_V) const;

    // Bearing-to-fire estimate (radians, robot frame). Returns 0.0 and sets
    // *valid=false if no cell is above threshold.
    float estimateBearing(float threshold = FIRE_DETECT_V);

    // Reset the EWMA filter on each cell.
    void reset();
    inline bool isValid() {return _angleValid;}
};


// ---------------------------------------------------------------------------
// Fan: low-side N-channel MOSFET (FQP30N06) gate driver.
//
// Behaviour:
//   * digitalWrite(fan_pin, HIGH/LOW) is sufficient - the gate is logic-level.
//   * We expose a "soft start" that ramps PWM from 0 to full over ~250 ms to
//     avoid slamming the 5V regulator with the fan inrush current. The fan
//     itself is run from the LiPo, but the MOSFET's input capacitance and the
//     spike in motor current at startup can brown out the rest of the rail
//     without it.
// ---------------------------------------------------------------------------
class Fan {
  private:
    uint8_t _pin;
    bool    _on = false;
    unsigned long _on_started_ms = 0;

  public:
    Fan(uint8_t pin) : _pin(pin) {}

    // Configure the pin. Call once in setup AFTER the constructor.
    void begin();

    // Soft-start the fan and remember when it was turned on.
    void on();

    // Hard-off (gate low). Safe to call repeatedly.
    void off();

    // Idempotent state queries.
    inline bool isOn() const { return _on; }
    inline unsigned long msSinceOn() const {
        return _on ? (millis() - _on_started_ms) : 0;
    }
};

#endif // FIRE_H

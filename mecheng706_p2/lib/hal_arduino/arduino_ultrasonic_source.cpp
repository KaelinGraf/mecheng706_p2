#include <Arduino.h>
#include "arduino_ultrasonic_source.h"

// The ISR has to be a free function bound to the AVR INT4 vector. It
// dispatches into whichever ArduinoUltrasonicSource registered itself last.
static volatile ArduinoUltrasonicSource* s_active = nullptr;

ISR(INT4_vect) {
    if (!s_active) return;
    const_cast<ArduinoUltrasonicSource*>(s_active)->on_echo_isr();
}

ArduinoUltrasonicSource::ArduinoUltrasonicSource(uint8_t trigger_pin, uint8_t echo_pin)
    : _trigger_pin(trigger_pin), _echo_pin(echo_pin),
      _t_rise(0), _t_fall(0), _t_last_rise(0) {}

void ArduinoUltrasonicSource::begin() {
    pinMode(_trigger_pin, OUTPUT);
    digitalWrite(_trigger_pin, LOW);

    // Configure INT4 for any-edge interrupts (matches original setup).
    EICRB |= (1 << ISC40);
    EICRB &= ~(1 << ISC41);
    EIMSK |= (1 << INT4);

    s_active = this;
}

void ArduinoUltrasonicSource::trigger() {
    digitalWrite(_trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger_pin, LOW);
}

unsigned long ArduinoUltrasonicSource::last_pulse_width_us() {
    // The ISR writes _t_rise on the rising edge and _t_fall on the falling
    // edge. If they're out of order (reading mid-pulse, etc.) fall back to
    // the previous rising edge to avoid an underflow.
    unsigned long t1, t2;
    noInterrupts();
    t1 = _t_rise;
    t2 = _t_fall;
    if (t2 < t1) t1 = _t_last_rise;
    interrupts();
    return (t2 >= t1) ? (t2 - t1) : 0UL;
}

void ArduinoUltrasonicSource::on_echo_isr() {
    if (digitalRead(_echo_pin)) {
        // Rising edge: rotate the timestamp window forward.
        _t_last_rise = _t_rise;
        _t_rise = micros();
    } else {
        _t_fall = micros();
    }
}

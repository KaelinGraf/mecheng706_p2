#include <Arduino.h>
#include "arduino_fan_output.h"

ArduinoFanOutput::ArduinoFanOutput(uint8_t pin) : _pin(pin) {}

void ArduinoFanOutput::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void ArduinoFanOutput::on() {
    // Soft-start: ramp PWM 0 -> 255 over ~240ms before pinning HIGH. The end
    // state is a hard digitalWrite HIGH so we don't keep the timer running.
    for (int duty = 0; duty <= 255; duty += 32) {
        analogWrite(_pin, duty);
        delay(30);
    }
    digitalWrite(_pin, HIGH);
}

void ArduinoFanOutput::off() {
    digitalWrite(_pin, LOW);
}

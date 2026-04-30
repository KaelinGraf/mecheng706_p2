#include <Arduino.h>
#include "arduino_analog_input.h"
#include "mappings.h"

float ArduinoAnalogInput::read_voltage(uint8_t pin) {
    float adc_value = (float)analogRead(pin);
    return (adc_value / (float)ADC_RANGE) * V_ADC;
}

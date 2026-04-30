#include <Arduino.h>
#include "arduino_clock.h"

unsigned long ArduinoClock::now_ms() { return millis(); }
unsigned long ArduinoClock::now_us() { return micros(); }

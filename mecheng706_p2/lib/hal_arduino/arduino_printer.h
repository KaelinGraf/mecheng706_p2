#ifndef HAL_ARDUINO_PRINTER_H
#define HAL_ARDUINO_PRINTER_H

#include <Print.h>
#include "iprinter.h"

// IPrinter wrapper around any Arduino Print* (HardwareSerial, SoftwareSerial,
// any other Stream that inherits from Print). One instance per output sink.
class ArduinoPrinter : public IPrinter {
public:
    explicit ArduinoPrinter(Print* sink) : _sink(sink) {}

    void print(const char* s)   override { if (_sink) _sink->print(s); }
    void print(char c)          override { if (_sink) _sink->print(c); }
    void print(int v)           override { if (_sink) _sink->print(v); }
    void print(unsigned int v)  override { if (_sink) _sink->print(v); }
    void print(long v)          override { if (_sink) _sink->print(v); }
    void print(unsigned long v) override { if (_sink) _sink->print(v); }
    void print(float v)         override { if (_sink) _sink->print(v); }
    void print(double v)        override { if (_sink) _sink->print(v); }
    void println()              override { if (_sink) _sink->println(); }

private:
    Print* _sink;
};

#endif

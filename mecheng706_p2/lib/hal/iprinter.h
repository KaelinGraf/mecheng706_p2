#ifndef HAL_IPRINTER_H
#define HAL_IPRINTER_H

// Logging sink. Wraps Arduino's HardwareSerial / SoftwareSerial in the
// embedded build; the sim build can route to stdout, a string buffer, or
// back into Python.
//
// Single-arg overloads cover every type the controller currently prints
// (mostly C-strings and numerics from the sensor mapping_reading_ float).
// Multi-arg variadics in FireFighter forward to these one-by-one.
class IPrinter {
public:
    virtual ~IPrinter() = default;

    virtual void print(const char* s) = 0;
    virtual void print(char c) = 0;
    virtual void print(int v) = 0;
    virtual void print(unsigned int v) = 0;
    virtual void print(long v) = 0;
    virtual void print(unsigned long v) = 0;
    virtual void print(float v) = 0;
    virtual void print(double v) = 0;

    virtual void println() = 0;

    void println(const char* s)   { print(s); println(); }
    void println(char c)          { print(c); println(); }
    void println(int v)           { print(v); println(); }
    void println(unsigned int v)  { print(v); println(); }
    void println(long v)          { print(v); println(); }
    void println(unsigned long v) { print(v); println(); }
    void println(float v)         { print(v); println(); }
    void println(double v)        { print(v); println(); }
};

#endif

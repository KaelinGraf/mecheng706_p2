#ifndef HAL_SIM_PRINTER_H
#define HAL_SIM_PRINTER_H

#include "iprinter.h"

// Sim printer. Default behaviour is to write to stdout via printf so a
// host smoke-test sees logs immediately. A constructor flag lets pybind11
// suppress stdout (Python can hook in via a callback later if needed).
class SimPrinter : public IPrinter {
public:
    SimPrinter(bool emit_to_stdout = true) : _emit(emit_to_stdout) {}

    void print(const char* s)   override;
    void print(char c)          override;
    void print(int v)           override;
    void print(unsigned int v)  override;
    void print(long v)          override;
    void print(unsigned long v) override;
    void print(float v)         override;
    void print(double v)        override;
    void println()              override;

private:
    bool _emit;
};

#endif

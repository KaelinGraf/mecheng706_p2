#include "sim_printer.h"
#include <cstdio>

void SimPrinter::print(const char* s)   { if (_emit && s) std::printf("%s", s); }
void SimPrinter::print(char c)          { if (_emit) std::printf("%c", c); }
void SimPrinter::print(int v)           { if (_emit) std::printf("%d", v); }
void SimPrinter::print(unsigned int v)  { if (_emit) std::printf("%u", v); }
void SimPrinter::print(long v)          { if (_emit) std::printf("%ld", v); }
void SimPrinter::print(unsigned long v) { if (_emit) std::printf("%lu", v); }
void SimPrinter::print(float v)         { if (_emit) std::printf("%g", (double)v); }
void SimPrinter::print(double v)        { if (_emit) std::printf("%g", v); }
void SimPrinter::println()              { if (_emit) std::printf("\n"); }

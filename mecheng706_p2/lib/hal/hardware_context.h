#ifndef HAL_HARDWARE_CONTEXT_H
#define HAL_HARDWARE_CONTEXT_H

#include "iclock.h"
#include "ianalog_input.h"
#include "igyro_source.h"
#include "iultrasonic_source.h"
#include "imotor_output.h"
#include "ifan_output.h"
#include "iprinter.h"

// Bundle of HAL pointers handed to the FireFighter at construction time.
//
// Every member is a non-owning raw pointer - the entry point owns the
// concrete instances and outlives the FireFighter (typical embedded
// pattern: instances live in setup() scope on Arduino, in the pybind11
// module / smoke-test main on the host).
//
// `printer_secondary` is optional; on Arduino it's wired to the Bluetooth
// SoftwareSerial so prints fan out to both USB and BT. May be null.
struct HardwareContext {
    IClock*            clock              = nullptr;
    IAnalogInput*      analog_input       = nullptr;
    IGyroSource*       gyro_source        = nullptr;
    IUltrasonicSource* ultrasonic_source  = nullptr;
    IMotorOutput*      motor_output       = nullptr;
    IFanOutput*        fan_output         = nullptr;
    IPrinter*          printer_primary    = nullptr;
    IPrinter*          printer_secondary  = nullptr;
};

#endif

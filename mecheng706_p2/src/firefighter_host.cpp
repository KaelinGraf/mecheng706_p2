// Host (native) smoke-test for the controller.
//
// Wires the Sim HAL backends into a HardwareContext, constructs the
// FireFighter, and runs a few sim ticks. The point of this entry is purely
// to prove the controller code compiles and links cleanly without any
// Arduino dependencies; the real Python sim talks to the same code through
// the pybind11 module under python_bindings/.

#include <cstdio>

#include "hardware_context.h"

#include "sim_clock.h"
#include "sim_analog_input.h"
#include "sim_gyro_source.h"
#include "sim_ultrasonic_source.h"
#include "sim_motor_output.h"
#include "sim_fan_output.h"
#include "sim_printer.h"

#include "firefighter.h"
#include "mappings.h"


int main() {
    // 1. Construct concrete Sim HAL backends.
    SimClock              clock;
    SimAnalogInput        adc;
    SimGyroSource         gyro;
    SimUltrasonicSource   us;
    SimMotorOutput        motors;
    SimFanOutput          fan;
    SimPrinter            printer(true);

    // Seed plausible voltages so the FSM has something to react to.
    adc.set_voltage(front_left_ir_pin,  1.5f);
    adc.set_voltage(front_right_ir_pin, 1.5f);
    adc.set_voltage(rear_left_ir_pin,   1.5f);
    adc.set_voltage(rear_right_ir_pin,  1.5f);
    adc.set_voltage(photo_front_pin,    0.2f);
    adc.set_voltage(photo_right_pin,    0.2f);
    adc.set_voltage(photo_rear_pin,     0.2f);
    adc.set_voltage(photo_left_pin,     0.2f);
    adc.set_voltage(battery_sense_pin,  3.85f);    // ~mid-charge LiPo
    us.push_distance_cm(150.0f);
    gyro.push_omega(0.0f);

    // 2. Build the HAL bundle.
    HardwareContext hw;
    hw.clock              = &clock;
    hw.analog_input       = &adc;
    hw.gyro_source        = &gyro;
    hw.ultrasonic_source  = &us;
    hw.motor_output       = &motors;
    hw.fan_output         = &fan;
    hw.printer_primary    = &printer;
    hw.printer_secondary  = nullptr;

    // 3. Construct the FireFighter and step the FSM a few times.
    FireFighter ff(hw);

    std::printf("\n[host] smoke test: stepping 50 ticks at 10ms each\n");
    for (int i = 0; i < 50; ++i) {
        ff.pollState();
        clock.advance_ms(10);
        gyro.push_omega(0.0f);
    }

    std::printf("[host] final state: %d, motor channel 0 last us = %u\n",
                (int)ff.getCurrentState(), motors.last_us(0));
    std::printf("[host] smoke test ok\n");
    return 0;
}

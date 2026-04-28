#ifndef MAPPINGS_H
#define MAPPINGS_H

#include <stdint.h>

// Diagnostic prints from sensor classes
#define DIAGNOSTICS true

// Ultrasonic ranging sensor pins (defined by the shield)
#define TRIG_PIN  48
#define ECHO_PIN  49

// Anything over ~400 cm (23200 us pulse) is "out of range".
#define MAX_DIST  23200

// Per-sensor minimum loop interval (ms)
#define LONGRANGE_LATENCY  50
#define SHORTRANGE_LATENCY 20

// ADC scaling
#define V_ADC      5.0
#define ADC_RANGE  1024

// Motor PWM pins (mecanum chassis)
#define left_front_pin  46
#define left_rear_pin   47
#define right_rear_pin  50
#define right_front_pin 51

// Drive motor PWM duty range (microseconds)
#define max_duty_motor 2300
#define min_duty_motor 700
#define neutral        1500

// Turret / small servo PWM duty range (microseconds)
#define max_duty_turret 2100
#define min_duty_turret 900
#define neutral_turret  1500

// IR sensor analog input pins
// Project 2 constraint: only 2 IR sensors of each type may be used.
#define front_left_ir_pin  A9
#define front_right_ir_pin A15
#define rear_right_ir_pin  A4
#define rear_left_ir_pin   A3

// ----- Project 2 specific pin assignments (TODO: assign as hardware is wired) -----
// Phototransistors for fire detection (4x available)
// #define photo_front_left_pin  Ax
// #define photo_front_right_pin Ax
// #define photo_rear_left_pin   Ax
// #define photo_rear_right_pin  Ax

// Fan / MOSFET gate pin for fire extinguishing
// #define fan_pin  Dx

// Mecanum geometry (cm) - half-length and half-width of wheel base
#define L_len 15
#define l_len 9

#endif // MAPPINGS_H

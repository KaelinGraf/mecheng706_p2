#ifndef MAPPINGS_H
#define MAPPINGS_H

#include <stdint.h>

// Diagnostic prints from sensor classes
#define DIAGNOSTICS true

// Ultrasonic ranging sensor pins (defined by the shield)
#define TRIG_PIN  48
#define ECHO_PIN  49
#define US_INT_PIN 2

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
// Small servo (turret) pin
#define turret_pin 9

// Drive motor PWM duty range (microseconds)
#define max_duty_motor 2300
#define min_duty_motor 700
#define neutral        1500

// Turret / small servo PWM duty range (microseconds)
#define max_duty_turret 2100
#define min_duty_turret 900
#define neutral_turret  1500

// ---------------------------------------------------------------------------
// IR sensor analog input pins
// Project 2 constraint: only 2 IR sensors of each type may be used.
// Layout finalised for Project 2 (see sensor_layout.html):
//   * 2x long-range 2Y0A21 on the front, splayed +/-30 deg from forward
//   * 2x short-range 2Y0A41SK over the rear wheels, facing forward but
//     splayed +/-40 deg outward to cover the rear quarters during strafes
// ---------------------------------------------------------------------------
#define front_left_ir_pin  A9    // long-range, body-left front (yaw +30 deg)
#define front_right_ir_pin A15   // long-range, body-right front (yaw -30 deg)
#define rear_left_ir_pin   A3    // short-range, over rear-left wheel (yaw +40 deg)
#define rear_right_ir_pin  A4    // short-range, over rear-right wheel (yaw -40 deg)

// ---------------------------------------------------------------------------
// Phototransistor analog input pins (4x for fire LED detection)
// Cardinal layout (front/right/rear/left). Comparing intensities across the
// front-vs-rear and left-vs-right pairs gives a coarse bearing to the fire
// in the robot frame.
// ---------------------------------------------------------------------------
#define photo_front_pin   A10
#define photo_right_pin   A11
#define photo_rear_pin    A12
#define photo_left_pin    A13

// ---------------------------------------------------------------------------
// Fan (extinguisher) - drives the gate of the FQP30N06 (FDPF55N06-D shown in
// kit) low-side N-channel MOSFET. Pin chosen on Mega Timer4 OC4C so the gate
// is PWM-capable if we want to soft-start the fan (current limit on the 5V
// regulator while the fan spins up). HIGH = fan ON.
// ---------------------------------------------------------------------------
#define fan_pin  8

// ---------------------------------------------------------------------------
// Mecanum geometry (cm) - half-length and half-width of wheel base
// ---------------------------------------------------------------------------
#define L_len 15
#define l_len 9

// ---------------------------------------------------------------------------
// Behaviour tuning constants. Centralised here so all FSM states pull from
// the same source of truth and the report can quote them directly.
// All distances in cm, speeds in the [-300, +300] control-effort scale used
// by driveMotors::writeAllMotors (see servo_control.cpp).
// ---------------------------------------------------------------------------

// SEARCH: forward cruise speed and turn speeds while wandering / wall-following.
#define SEARCH_FORWARD_SPEED   110.0f
#define SEARCH_TURN_SPEED       70.0f
#define REVERSE_SPEED           50.0f

// Distance below which an object on the front arc is treated as an obstacle
// (forces transition to AVOID). Sized so the robot can react before the ~10 cm
// dia. cylinder enters the dead-zone of the long-range IRs.
#define OBSTACLE_TRIGGER_CM     22.0f

// Hysteresis: must read this clear before AVOID is allowed to release back to
// SEARCH/APPROACH. Wider than OBSTACLE_TRIGGER_CM to avoid limit-cycling.
#define OBSTACLE_CLEAR_CM       35.0f

// Wall-follow band - if a side IR reads below this, nudge away from that side.
#define WALL_FOLLOW_CM          15.0f

// APPROACH: speed toward detected fire, turn gain for bearing correction
#define APPROACH_FORWARD_SPEED  90.0f
#define APPROACH_TURN_GAIN      30.0f
#define APPROACH_MAX_TURN       100.0f

// AVOID: strafe and rotation speeds
#define AVOID_STRAFE_SPEED      130.0f
#define AVOID_STRAFE_MS         1000.0f
#define AVOID_SPEED             60.0f
#define AVOID_ROTATE_SPEED      80.0f

// EXTINGUISH gate: robot's centre is within 20 cm of the fire's centre per the
// brief. The ultrasonic measures from the front face, so trigger when the
// front-of-robot reading is <= (20 cm - half chassis length) ~ 5 cm; a more
// forgiving value is used to allow for sensor noise / cylinder curvature.
#define EXTINGUISH_RANGE_CM     12.0f

// Phototransistor threshold (volts) above which we count a cell as "seeing"
// fire. Calibrated against ambient room light + LED at ~2 m line-of-sight.
// Tune on the bench: cover all 4, read the noise floor, then add ~3 sigma.
#define FIRE_DETECT_V           1.6f

// Phototransistor threshold for declaring a fire is OUT (LED has gone dark).
// Lower than detect threshold by a margin to provide hysteresis against
// flickering / partial obscuration.
#define FIRE_OUT_V              0.7f

// Maximum time the fan runs on a single fire (per brief: up to 10 s, after
// which the demonstrator turns the LED off manually if it hasn't already).
#define FAN_MAX_MS              10000UL

// AVOID strafe duration (ms). Sized so a full chassis width clears at the
// strafe speed below.
#define AVOID_STRAFE_SPEED      130.0f

#endif // MAPPINGS_H

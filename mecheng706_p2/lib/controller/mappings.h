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

// ---------------------------------------------------------------------------
// IR sensor analog input pins
// Project 2 constraint: only 2 IR sensors of each type may be used.
// Layout finalised for Project 2 (see sensor_layout.html):
//   * 2x long-range 2Y0A21 on the front, splayed +/-30 deg from forward
//   * 2x short-range 2Y0A41SK over the rear wheels, facing forward but
//     splayed +/-40 deg outward to cover the rear quarters during strafes
// ---------------------------------------------------------------------------
// Pin numbers below use the Arduino Mega 2560 raw indices for the analog
// channels (A0 == 54, A1 == 55, ..., A15 == 69) so this header stays free of
// any <Arduino.h> dependency and compiles unchanged in the host build.
#define front_left_ir_pin   63   // A9  - long-range, body-left front (yaw +30 deg)
#define front_right_ir_pin  69   // A15 - long-range, body-right front (yaw -30 deg)
#define rear_left_ir_pin    57   // A3  - short-range, over rear-left wheel (yaw +40 deg)
#define rear_right_ir_pin   58   // A4  - short-range, over rear-right wheel (yaw -40 deg)

// The four body-cardinal phototransistor pins (A10-A13) used by the
// retired FireBank are now free; fire bearing comes from the turret's
// boom-mounted PhotoArray on A1/A2 (see below).

// Battery sense (LiPo divider into a Mega ADC channel).
#define battery_sense_pin   54   // A0

// ---------------------------------------------------------------------------
// Turret subsystem (NEW — body-mounted panning boom carrying the fan and
// SFH 300 FA IR phototransistor array). Fan + photodiodes ride the boom
// so the fan auto-aims at the candle once the turret locks.
// ---------------------------------------------------------------------------
// Pan servo (the small hobby servo from the kit). Driven through the same
// IMotorOutput pool as the drive motors; channel 4 (drive uses 0..3).
#define turret_servo_pin     11
#define TURRET_SERVO_CHANNEL  4

// Boom-mounted photodiode pins. Two minimum (left/right of the boom
// centerline); add more by extending the PhotoArray in FireFighter.
#define turret_photo_left_pin    55   // A1
#define turret_photo_right_pin   56   // A2

// Soft-centroid mount angles for the two boom photodiodes (rad,
// right-positive). +/-30 deg fan-out gives a ~60 deg overlap window
// where both photodiodes contribute to the bearing centroid.
#define TURRET_PHOTO_LEFT_RAD   (-0.5235988f)   // -30 deg
#define TURRET_PHOTO_RIGHT_RAD  ( 0.5235988f)   // +30 deg

// Mechanical pan range relative to body forward (rad).
#define TURRET_PAN_MIN_RAD  (-1.5707963f)       // -90 deg
#define TURRET_PAN_MAX_RAD  ( 1.5707963f)       // +90 deg

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
// Behaviour tuning constants. Centralised here so all behaviors pull from
// the same source of truth and the report can quote them directly.
// All distances in cm, speeds in the [-300, +300] control-effort scale used
// by driveMotors::writeAllMotors (see servo_control.cpp).
// ---------------------------------------------------------------------------

// FindFire wall-bias / cruise turn speed (rear-IR side push).
#define SEARCH_FORWARD_SPEED   110.0f
#define SEARCH_TURN_SPEED       70.0f

// Distance below which an object on the front arc is treated as an obstacle
// by AvoidObstacleBehavior. Sized so the robot can react before the
// ~10 cm dia. cylinder enters the dead-zone of the long-range IRs.
#define OBSTACLE_TRIGGER_CM     22.0f

// Hysteresis: must read this clear before AvoidObstacle releases. Wider
// than OBSTACLE_TRIGGER_CM to avoid limit-cycling.
#define OBSTACLE_CLEAR_CM       35.0f

// Wall-follow band - if a side IR reads below this, nudge away from that side.
#define WALL_FOLLOW_CM          15.0f

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
#define AVOID_STRAFE_MS         900UL
#define AVOID_STRAFE_SPEED      130.0f

// ---------------------------------------------------------------------------
// BEHAVIOR ARBITER TUNING (placeholders - tune on the bench).
// All distances cm, speeds in [-300,+300] effort scale, angles in radians.
// ---------------------------------------------------------------------------

// FindFire: cruise speed and exploration nudge cadence while the turret
// hunts for a candle. The turret does its own SWEEP; the body just moves
// to expose new arena to the boom-mounted photo array.
#define FINDFIRE_FORWARD_SPEED       100.0f
#define FINDFIRE_NUDGE_PERIOD_MS    4500UL
#define FINDFIRE_NUDGE_LEN_MS        400UL
#define FINDFIRE_NUDGE_TURN           50.0f

// MoveToFire: open-loop yaw-rate gain on the turret's body-relative
// bearing. Step 6 swaps this for closed-loop yaw control on the IMU's
// quaternion-derived absolute yaw.
#define MOVETOFIRE_KP_TURN          120.0f
#define MOVETOFIRE_FORWARD_SPEED     90.0f
// Above this body-relative bearing magnitude the body pivots in place
// instead of driving forward (avoid arcing wide past a candle).
#define MOVETOFIRE_PIVOT_RAD          0.6f

// ExtinguishFire: aim window (turret bearing magnitude) and fire-out
// debounce count (ticks) before the LED is declared extinguished.
#define EXTINGUISH_AIM_RAD            0.35f
#define EXTINGUISH_OUT_DEBOUNCE       10
#define EXTINGUISH_HARD_TIMEOUT_MS    10000UL    // matches FAN_MAX_MS

// AvoidObstacle: window around the turret bearing to treat a forward
// reading as the candle (suppress avoid). The body's front IRs at +-30
// deg can each spot something close - if it lines up with where the
// turret is looking and the candle is bright there, it is the fire.
#define AVOID_FIRE_MASK_RAD           0.5f      // ~28 deg

#endif // MAPPINGS_H

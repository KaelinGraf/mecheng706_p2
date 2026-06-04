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
#define turret_pin 8

// Drive motor PWM duty range (microseconds)
#define max_duty_motor 2300
#define min_duty_motor 700
#define neutral        1500

// Turret / small servo PWM duty range (microseconds). LEGACY: the turret is now driven directly with
// Servo::write(degrees) on a default attach, so these us values are no longer on the run path.
#define max_duty_turret 2100
#define min_duty_turret 900
#define neutral_turret  1500
#define SERVO_CENTER 90
// Turret usable travel (deg) -- mimics the mapping branch. The servo is driven DIRECTLY with
// Servo::write(angle) on a DEFAULT attach (no min/max -> the library's standard 0-180 deg -> 544-2400 us
// map); we command ONLY within [TURRET_DEG_MIN, TURRET_DEG_MAX]. SERVO_CENTER (90) is dead-forward.
#define TURRET_DEG_MIN 10
#define TURRET_DEG_MAX 170

// ---------------------------------------------------------------------------
// IR sensor analog input pins
// Project 2 constraint: only 2 IR sensors of each type may be used.
// Layout (confirmed against the hardware; the FireFighter ctor classes are ground truth):
//   * 2x SHORT-range (2Y0A41SK, reliable ~4-30 cm) on the FRONT -> the close obstacle + approach band.
//     Instantiated as ShortRangeIR.
//   * 2x LONG-range (2Y0A21, ~10-80 cm) over the REAR wheels -> wider side coverage during strafes.
//     Instantiated as LongRangeIR.
// (Earlier comments here had front/rear and short/long SWAPPED -- corrected to match the wiring.)
// ---------------------------------------------------------------------------
#define front_left_ir_pin  A9    // SHORT-range, body-left front
#define front_right_ir_pin A15   // SHORT-range, body-right front
#define rear_left_ir_pin   A3    // LONG-range, over rear-left wheel
#define rear_right_ir_pin  A4    // LONG-range, over rear-right wheel

// ---------------------------------------------------------------------------
// Phototransistor analog input pins (4x for fire LED detection)
// Cardinal layout (front/right/rear/left). Comparing intensities across the
// front-vs-rear and left-vs-right pairs gives a coarse bearing to the fire
// in the robot frame.
// ---------------------------------------------------------------------------
#define photo_sl_pin   A10
#define photo_l_pin   A11
#define photo_r_pin    A12
#define photo_sr_pin    A13

// ---------------------------------------------------------------------------
// Fan (extinguisher) - drives the gate of the FQP30N06 (FDPF55N06-D shown in
// kit) low-side N-channel MOSFET. Pin chosen on Mega Timer4 OC4C so the gate
// is PWM-capable if we want to soft-start the fan (current limit on the 5V
// regulator while the fan spins up). HIGH = fan ON.
// ---------------------------------------------------------------------------
#define fan_pin  A8

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
#define SEARCH_FORWARD_SPEED   60.0f
#define SEARCH_TURN_SPEED       70.0f
#define REVERSE_SPEED           50.0f

// Distance below which an object on the front arc is treated as an obstacle
// (forces transition to AVOID). Sized so the robot can react before the ~10 cm
// dia. cylinder enters the dead-zone of the long-range IRs.
#define OBSTACLE_TRIGGER_CM_F     10.0f
// Front ULTRASONIC frontal-obstacle trigger -- tighter than the IR (10) so the frontal AVOID doesn't
// fire on the FIRE during the close approach (it sits below EXTINGUISH_RANGE_CM=8): the robot reaches
// close_front + extinguishes instead of avoiding the fire. (Teammates' value; used for obstacle_ahead.)
#define OBSTACLE_TRIGGER_CM_US     5.0f
#define OBSTACLE_TRIGGER_CM_R     8.0f

// Hysteresis: must read this clear before AVOID is allowed to release back to
// SEARCH/APPROACH. Wider than OBSTACLE_TRIGGER_CM to avoid limit-cycling.
#define OBSTACLE_CLEAR_CM_F      15.0f
#define OBSTACLE_CLEAR_CM_R      10.0f

// Wall-follow band - if a side IR reads below this, nudge away from that side.
#define WALL_FOLLOW_CM          15.0f
#define SEARCH_SPEED             55.0f

// APPROACH: speed toward detected fire, turn gain for bearing correction. The write path halves this
// (-motor_vx/2), so the effective effort is ~half. 60 was WAY too slow once locked on; raised to 150
// (~teammates' brisk approach) so the robot actually closes on the fire. It hard-stops at close_front
// (us < EXTINGUISH_RANGE_CM), so a higher speed just reaches the fire faster, it doesn't overshoot the
// extinguish gate. BENCH-TUNE down if it overshoots.
#define APPROACH_FORWARD_SPEED  150.0f
#define APPROACH_TURN_GAIN      17.0f
#define APPROACH_MAX_TURN       100.0f

// AVOID: strafe and rotation speeds
#define AVOID_STRAFE_MS         50.0f
#define AVOID_SPEED             60.0f
#define AVOID_ROTATE_SPEED      30.0f
#define AVOID_URGENT            6.0f

// STRAFE-CENTRIC clearance. Instead of the reverse/rotate AVOID for anything on a side, the robot
// CENTRES itself between side obstacles by strafing (+vy = RIGHT) -- so a barely-fit gap stays
// threadable and side objects are edged away (1 cm miss is fine) without rotating. Tracking adds a
// strafe term proportional to the left/right clearance imbalance to its normal drive; reverse/rotate
// AVOID is kept ONLY as the boxed-in fallback for a true frontal wall.
#define WALL_CARE_CM        18.0f   // begin centring/clearing once a side IR reads within this (cm)
#define SIDE_HARD_MIN_CM     5.0f   // a side this close => decisive full strafe away (collision-imminent)
#define WALL_CENTER_GAIN     6.0f   // strafe effort per cm of imbalance (was 3 -- strafe responded too slowly)
#define WALL_STRAFE_SPEED  100.0f   // max lateral effort while clearing/centring (was 60 -- too slow)
#define STRAFE_DIR_SIGN      1      // flip to -1 if the bench shows the strafe goes the WRONG way

// RAMBO mode (default OFF). When 1, Tracking ignores ALL obstacle avoidance EXCEPT a frontal WALL:
// no side-object strafe-centring, no recent-fire back-away -- the robot charges straight at the fire.
// On a frontal wall it rotates AWAY from the heading that drove it in (by RAMBO_TURN_AWAY_DEG) rather
// than reversing and returning to that heading, then resumes the charge. Toggle to 1 to enable.
#define RAMBO_MODE              0
#define RAMBO_TURN_AWAY_DEG     150.0f   // degrees to rotate off the wall-entry heading before resuming

// EXTINGUISH gate: robot's centre is within 20 cm of the fire's centre per the
// brief. The ultrasonic measures from the front face, so trigger when the
// front-of-robot reading is <= (20 cm - half chassis length) ~ 5 cm; a more
// forgiving value is used to allow for sensor noise / cylinder curvature.
#define EXTINGUISH_RANGE_CM     8.0f

// do not transition back to extinguish state if we put out a fire less than
// this ammount of MS ago as it may be the same fire
#define EXTINGUISH_TIMEOUT_MS     3000

// Phototransistor threshold (volts) above which we count a cell as "seeing"
// fire. Calibrated against ambient room light + LED at ~2 m line-of-sight.
// Tune on the bench: cover all 4, read the noise floor, then add ~3 sigma.
#define FIRE_DETECT_V           0.3f

// Phototransistor threshold for declaring a fire is OUT (LED has gone dark). Used ONLY by
// Extinguish::poll() -> FireBank::allBelow(FIRE_OUT_V): at extinguish range a LIT fire saturates the
// cells (~4.8 V), so when the LED cuts they fall through 0.7 V and allBelow() latches "out". This is
// an extinguish-RANGE threshold, NOT a detect-hysteresis pair with FIRE_DETECT_V -- it is intentionally
// ABOVE FIRE_DETECT_V (0.3) and must stay above the post-extinguish floor for reliable detection.
#define FIRE_OUT_V              0.7f

// Maximum time the fan runs on a single fire (per brief: up to 10 s, after
// which the demonstrator turns the LED off manually if it hasn't already).
#define FAN_MAX_MS              10000UL

// AVOID strafe duration (ms). Sized so a full chassis width clears at the
// strafe speed below.
#define AVOID_STRAFE_SPEED      130.0f

// Outer-pair bearing calibration (G, deg/unit, saturation, floors) lives as
// locals inside FireBank::estimateBearing() in fire.cpp (the "new firebank PT"
// implementation), not here.

// ---------------------------------------------------------------------------
// Behaviour 1: turret lock-on gate (outer phototransistor pair).
// The two OUTER cells (_sl, _sr) are mounted flat / forward-facing with larger
// pull-down resistors, so they are the long-range detectors. The turret only
// declares lock-on when BOTH exceed FIRE_LOCK_OUTER_V; once locked it holds
// until BOTH fall below FIRE_UNLOCK_OUTER_V (hysteresis) for LOCK_LOSS_DEBOUNCE
// consecutive turret updates. TUNE ON BENCH against the new resistor values.
// ---------------------------------------------------------------------------
#define FIRE_LOCK_OUTER_V       0.8f
#define FIRE_UNLOCK_OUTER_V     0.20f
#define LOCK_LOSS_DEBOUNCE      4     // teammates' bench value (down from 8): faster lock-loss recovery

// ---------------------------------------------------------------------------
// Behaviour 2: 360-degree spin-scan localisation. On startup and after the
// first fire is extinguished, the chassis rotates in place (turret centred)
// until the outer pair detects the fire (FIRE_LOCK_OUTER_V), then hands off to
// SEARCH for the existing turret lock/track + approach. SPIN_SCAN_SPEED is the
// in-place rotation effort (vtheta, on the [-300, +300] scale). Keep it slow
// enough that the outer-cell EWMA crosses the gate while the fire is still
// inside the +/-25 deg phototransistor cone.
// ---------------------------------------------------------------------------
#define SPIN_SCAN_SPEED         60.0f

#endif // MAPPINGS_H

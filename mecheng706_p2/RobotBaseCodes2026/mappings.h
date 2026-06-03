#ifndef MAPPINGS_H
#define MAPPINGS_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Raw-Serial debug gate for the DBG()/DBGLN() shims below (and DIAGNOSTICS). 0
// (default) => those stray Serial.print sites expand to nothing; 1 => they forward
// to USB Serial, restoring legacy debug chatter (FSM traces, sensor dumps, boot
// markers) on the USB link.
//   NOTE: FireFighter::print/println (firefighter.h) are NOT gated by this — they
//   ALWAYS route to BOTH the USB and Bluetooth links. So anything sent through
//   them, including the occupancy frames from WorldModel::printSet(), reaches the
//   host viewer over Bluetooth regardless of this switch. (Consequence: any FSM
//   debug line that uses ff->print also goes out over Bluetooth; the MATLAB viewer
//   ignores non-`occupancy` lines, but keep per-tick chatter light at 115200.)
// Serial.begin()/Bluetooth setup are left intact regardless.
#define SERIAL_DEBUG 0

// SWEEP_TEST bench mode toggle. false (default) => normal firefighting FSM; true
// => run the outer-pair sweep instead (see sweep_test.* and analysis/plot_sweep.m).
// The sweep streams its CSV via FireFighter::print/println, which now always route
// to USB + Bluetooth. Keep in sync with the #if SWEEP_TEST blocks in
// RobotBaseCodes2026.ino.
#define SWEEP_TEST false

// Drive-gain (KX/KY) calibration bench mode. When true, loop() runs a blocking
// timed sequence (see drive_calibration.*): for each command in CAL_VX[], a 3-2-1
// warning, drive forward CAL_DRIVE_MS, then STOP CAL_STOP_MS so you can tape-
// measure the distance and reposition; then repeat strafing RIGHT. Bypasses the
// FSM. Self-contained (no mapping dependency) so it runs with ENABLE_MAPPING = 0.
#define DRIVE_CALIBRATION false
#define CAL_DRIVE_MS   5000UL   // ms driving at each commanded velocity
#define CAL_STOP_MS   15000UL   // ms stopped to measure + reposition

// Diagnostic prints from sensor classes. Folded under SERIAL_DEBUG so the normal
// run path stays occupancy-only; flip SERIAL_DEBUG to 1 to restore them.
#define DIAGNOSTICS (SERIAL_DEBUG)

// Raw-Serial debug shims. Use these to gate stray `Serial.print(...)` call sites
// that are NOT routed through FireFighter::print/println. When SERIAL_DEBUG==0 they
// expand to nothing (the args are not even evaluated), so these particular raw
// USB-Serial sites stay silent. When ==1 they forward to Serial. (The occupancy
// emitter and SWEEP_TEST go through FireFighter::print, which is always live.)
#if SERIAL_DEBUG
  #define DBG(...)    do { Serial.print(__VA_ARGS__);   } while (0)
  #define DBGLN(...)  do { Serial.println(__VA_ARGS__); } while (0)
#else
  #define DBG(...)    do {} while (0)
  #define DBGLN(...)  do {} while (0)
#endif

// ---------------------------------------------------------------------------
// Onboard mapping pipeline (occupancy grid + dead reckoning + fire localiser).
// OFF by default: the mapping/ sources live in a subfolder Arduino does NOT
// auto-compile, so enabling this also requires the unity shim mapping_build.cpp
// to pull those translation units into the build. Flip to 1 once the mapping
// layer is build-verified on-device.
#define ENABLE_MAPPING    1
// Minimum interval between full WorldModel updates (ms). The grid walk is heavy
// and serial bandwidth is finite, so we throttle rather than run every loop.
#define MAPPING_UPDATE_MS 50
// Minimum interval between occupancy-frame emissions (ms), INDEPENDENT of the
// update cadence above. A 30x30 ASCII frame is ~3 KB; at 115200 baud that caps
// near 3 fps, so the protocol asks for 2-4 Hz. 300 ms ~= 3.3 Hz. printSet() runs
// on this throttle in FireFighter::pollState() while update() runs at MAPPING_UPDATE_MS.
#define PRINT_OCC_MS 1000
// Minimum robot displacement (cm) between fire-localiser bearing observations. The
// localiser only needs a few SPATIALLY DIVERSE bearings to triangulate, so while the
// turret is locked we ingest a new bearing only after the robot has moved this far
// since the last one (see WorldModel::update). Smaller = more (but more redundant)
// observations; larger = fewer, more diverse. Tune vs the ~150 cm arena.
#define FIRE_OBS_MIN_DISP_CM 10.0f

// Stationary mapping test. true => loop() still runs the full sensor + mapping +
// occupancy-emit pipeline, but pollState() NEVER commands the drive motors: the
// chassis holds position while you move obstacles by hand and watch the live
// occupancy map (analysis/occupancy_viz.m over Bluetooth). The turret keeps
// scanning so fire localisation still works. Set back to false for normal
// autonomous driving (that's also when the forward/strafe motor mixing matters).
#define MAPPING_TEST false

// Frame-calibration primitive test. true => pollState() ignores the FSM and drives
// a slow, looping sequence of single-axis primitives — STOP / +vx forward / +vy
// strafe-right / +vtheta rotate — announcing each over serial, while the mapping +
// occupancy stream keeps running. Drive a KNOWN body command, watch the robot move
// physically AND the occupancy-map marker, and compare to calibrate the chassis
// frame against the map frame (i.e. find any axis/sign inversion). Mutually
// exclusive with MAPPING_TEST; set false for normal operation.
#define FRAME_CAL_TEST false

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
#define SERVO_CENTER 85

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
#define OBSTACLE_TRIGGER_CM_F     15.0f
#define OBSTACLE_TRIGGER_CM_R     9.0f

// Hysteresis: must read this clear before AVOID is allowed to release back to
// SEARCH/APPROACH. Wider than OBSTACLE_TRIGGER_CM to avoid limit-cycling.
#define OBSTACLE_CLEAR_CM_F      22.0f
#define OBSTACLE_CLEAR_CM_R      10.0f

// Wall-follow band - if a side IR reads below this, nudge away from that side.
#define WALL_FOLLOW_CM          15.0f
#define SEARCH_SPEED             55.0f

// APPROACH: speed toward detected fire, turn gain for bearing correction
#define APPROACH_FORWARD_SPEED  10.0f
#define APPROACH_TURN_GAIN      17.0f
#define APPROACH_MAX_TURN       100.0f

// AVOID: strafe and rotation speeds
#define AVOID_STRAFE_MS         50.0f
#define AVOID_SPEED             60.0f
#define AVOID_ROTATE_SPEED      30.0f
#define AVOID_URGENT            6.0f

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

// Outer-pair bearing calibration (G, deg/unit, saturation, floors) lives as
// locals inside FireBank::estimateBearing() in fire.cpp (the "new firebank PT"
// implementation), not here.

// ---------------------------------------------------------------------------
// Behaviour 1: turret lock-on gate (outer phototransistor pair). RELAXED lock with
// light dual hysteresis. The displacement-gated fire-localiser ingestion (see
// WorldModel::update / FIRE_OBS_MIN_DISP_CM) now guards observation QUALITY, so the
// lock no longer has to be strict — its only job is "is there a fire to track right
// now?". Keep it permissive so we reliably ACQUIRE fires (the previous both-cells /
// high-threshold gate was missing them):
//   * acquire when EITHER outer (flat, long-range) cell reads >= FIRE_LOCK_OUTER_V
//     for LOCK_GAIN_DEBOUNCE consecutive updates -> an off-centre fire still locks;
//   * hold until BOTH fall below FIRE_UNLOCK_OUTER_V for LOCK_LOSS_DEBOUNCE updates
//     (hysteresis prevents chatter and rides out brief dropouts).
// Lower FIRE_LOCK_OUTER_V / LOCK_GAIN_DEBOUNCE to acquire even more eagerly
// (FIRE_DETECT_V ~ 0.3 is the ambient floor, so stay above it). TUNE ON BENCH.
// ---------------------------------------------------------------------------
#define FIRE_LOCK_OUTER_V       0.4f
#define FIRE_UNLOCK_OUTER_V     0.20f
#define LOCK_GAIN_DEBOUNCE      2     // consecutive detections required to ACQUIRE
#define LOCK_LOSS_DEBOUNCE      8     // consecutive faded updates required to RELEASE

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

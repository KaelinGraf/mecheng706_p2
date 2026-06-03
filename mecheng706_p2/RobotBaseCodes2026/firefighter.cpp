#include "Arduino.h"
#include "HardwareSerial.h"
#include "firefighter.h"
#include "initialising.h"
#include "search.h"
#include "extinguish.h"
#include "stopped.h"
#include "spin_scan.h"
#if ENABLE_MAPPING
#include "mapping/world_model.hpp"   // full mapping definitions (subfolder; see mapping_build.cpp)
#endif

// Ultrasonic ISR plumbing - the ISR needs a free-function entry point but
// must dispatch into the active Ultrasonic instance.
volatile Ultrasonic* ultrasonicISR = nullptr;
ISR(INT4_vect) {
  if (!ultrasonicISR) {
    // ISR called before the Ultrasonic instance is ready; do nothing.
    return;
  }
  // Rising edge on the echo pin: rotate the timestamp window forward
  if (digitalRead(US_INT_PIN)) {
    unsigned long last_t1 = ultrasonicISR->getSentTime();
    ultrasonicISR->setLastSent(last_t1);
    ultrasonicISR->setSentTime(micros());
  }
  // Falling edge: capture the return time
  else {
    ultrasonicISR->setReturnTime(micros());
  }
}

FireFighter::FireFighter(Adafruit_BNO08x* bno08x, sh2_SensorValue_t* sensorValue, HardwareSerial* SerialCom) {
  serialCom_ = SerialCom;

  // Initialise hardware first, so states can safely access sensors/motors
  _gyro = new Gyroscope(bno08x, sensorValue, SerialCom);
#if SERIAL_DEBUG
  serialCom_->print("Gyro exit");
#endif
  _motors = new driveMotors();
  _front_left_ir = new ShortRangeIR(front_left_ir_pin);
  _front_right_ir = new ShortRangeIR(front_right_ir_pin);
  _rear_left_ir = new LongRangeIR(rear_left_ir_pin);
  _rear_right_ir = new LongRangeIR(rear_right_ir_pin);
  _ultrasonic = new Ultrasonic(ECHO_PIN, TRIG_PIN, MAX_DIST);
  ultrasonicISR = _ultrasonic;
  _motors->attatchAll();
  _lastExtinguisedMillis = 0;

  // Fire-detection bank: four cardinal phototransistors plus the
  // aggregator that produces "any detection" / "bearing to fire" answers.
  _photo_sl = new Phototransistor(photo_sl_pin);
  _photo_l = new Phototransistor(photo_l_pin);
  _photo_r  = new Phototransistor(photo_r_pin);
  _photo_sr  = new Phototransistor(photo_sr_pin);
  _fire_bank   = new FireBank(_photo_sl, _photo_l, _photo_r, _photo_sr);

  // Fan / MOSFET driver. begin() forces the gate low before we start
  // running any state's poll(), so the fan can't be left on by a reset
  // mid-extinguish.
  _fan = new Fan(fan_pin);
  _fan->begin();

#if ENABLE_MAPPING
  // World model owns the occupancy grid + dead reckoner + fire localiser. Config
  // (sensor mount poses) lives in RobotModel. Both heap-allocated and never freed
  // (FireFighter lives for the whole run), so no destructors are linker-referenced.
  _robot_model = new RobotModel();
  _world = new WorldModel(this, _robot_model);
#endif

  // Initialise all states up-front to avoid runtime allocation
  states_[State::INITIALISING] = new Initialising(this);
  states_[State::SEARCH]       = new Search(this);
  states_[State::EXTINGUISH]   = new Extinguish(this);
  states_[State::STOPPED]      = new Stopped(this);
  states_[State::SPIN_SCAN]    = new SpinScan(this);

  // Begin initial state AFTER all hardware and states are ready
  current_state_ = states_[State::INITIALISING];
  current_state_->begin();
}

bool FireFighter::switchState(State::Name newState, StateData data) {
  // 1. End the current state
  if (current_state_) {
    current_state_->end();
  }

  if (!is_battery_voltage_OK()) {
    // Fall back to initialising on undervoltage
    current_state_ = states_[State::INITIALISING];
    return false;
  }

  // 2. Look up the new state
  current_state_ = states_[newState];

  // 3. Begin the new state, with optional data payload
  if (current_state_) {
    if (data.param != -1.0) {
      current_state_->begin(data);
      return true;
    }
    else {
      current_state_->begin();
      return true;
    }
  }

  return false;
}

#if FRAME_CAL_TEST
// Frame-calibration primitive sequencer (mappings.h: FRAME_CAL_TEST). Cycles
// through slow single-axis body commands with stops between, announcing each, so
// you can drive a KNOWN command and watch which way the occupancy-map robot marker
// moves vs how the chassis physically moves — exposing any axis/sign inversion
// between the chassis frame and the dead-reckoned map frame. Servos hold their last
// command, so writing once per phase keeps the motion going (and latches
// last_commands for dead-reckoning) until the next phase.
//   Cycle: STOP(sync) -> +vx forward -> STOP -> +vy strafe RIGHT -> STOP -> +vtheta
//   rotate -> repeat. forward/strafe are valid even with a stuck gyro (heading ~0);
//   the rotate phase only shows on the map once the gyro heading actually updates.
static void frameCalStep(FireFighter* ff) {
  static unsigned long phase_start = 0;
  static int phase = -1;
  const unsigned long STOP_MS = 2500;
  const unsigned long MOVE_MS = 2000;
  const float V = 45.0f;   // gentle linear effort
  const float W = 50.0f;   // gentle rotate effort
  unsigned long now = millis();

  bool moving = (phase >= 0) && (phase % 2 != 0);
  unsigned long dwell = moving ? MOVE_MS : STOP_MS;

  if (phase < 0 || now - phase_start >= dwell) {
    phase = (phase + 1) % 6;
    phase_start = now;
    switch (phase) {
      case 0: ff->println("[FRAMECAL] STOP (cycle start)");        ff->_motors->writeAllMotors(0, 0, 0); break;
      case 1: ff->println("[FRAMECAL] +vx  forward");              ff->_motors->writeAllMotors(V, 0, 0); break;
      case 2: ff->println("[FRAMECAL] STOP");                      ff->_motors->writeAllMotors(0, 0, 0); break;
      case 3: ff->println("[FRAMECAL] +vy  strafe RIGHT");         ff->_motors->writeAllMotors(0, V, 0); break;
      case 4: ff->println("[FRAMECAL] STOP");                      ff->_motors->writeAllMotors(0, 0, 0); break;
      case 5: ff->println("[FRAMECAL] +vtheta  rotate (CW)");      ff->_motors->writeAllMotors(0, 0, W); break;
    }
  }
}
#endif

void FireFighter::pollState() {
  _gyro->readSensor();
  _ultrasonic->service();   // pump the non-blocking ultrasonic (schedule pings + cache echoes)
  _fire_bank->update();
  updateIrSensors();

#if ENABLE_MAPPING
  // Drive the mapping pipeline at a throttled cadence (the grid walk is heavy and
  // serial bandwidth is finite). buildTap() consolidates this tick's inputs into
  // the snapshot the WorldModel consumes.
  static unsigned long last_map_ms = 0;
  unsigned long map_now = millis();
  if (map_now - last_map_ms >= MAPPING_UPDATE_MS) {
    Tap tap = buildTap();
    _world->update(&tap);
    last_map_ms = map_now;
  }

  // Stream the occupancy frame. printSet() is now NON-BLOCKING: it emits at most one
  // protocol line per call and self-throttles to PRINT_OCC_MS between frames, so we
  // call it EVERY loop. Spreading the ~3 KB frame across loop iterations keeps each
  // iteration's serial cost to ~one line, so the turret pan-scan isn't starved
  // (emitting the whole frame at once used to block the loop ~250 ms and stutter the
  // servo). It routes through FireFighter::print -> both USB and Bluetooth links.
  _world->printSet();
#endif

  if (current_state_) {
#if MAPPING_TEST
    // Stationary mapping test (mappings.h): hold the drive motors so the chassis
    // never moves. The sensor + mapping + occupancy pipeline above still runs every
    // tick, so you can move obstacles by hand and watch the live occupancy map.
    _motors->writeAllMotors(0.0f, 0.0f, 0.0f);
#elif FRAME_CAL_TEST
    // Frame-calibration primitives (mappings.h): drive known single-axis body
    // commands and watch the occupancy-map marker to calibrate the chassis frame
    // against the map frame. The mapping/occupancy pipeline above keeps streaming.
    frameCalStep(this);
#else
    current_state_->poll();
#endif
  }
}

// Consolidate everything the WorldModel needs this tick into one snapshot. This
// is the single place unit conversions happen: pass-through for already-correct
// values, deg->rad for the turret angle and fire bearing-error.
Tap FireFighter::buildTap() {
  Tap t;
  t.heading = _gyro->getHeading();                 // rad, continuous (never reset) -> pose

  const auto &cmd = _motors->getLastCommand();
  t.vx = cmd.vx;                                   // RAW command; DeadReckoner applies KX/KY
  t.vy = cmd.vy;

  if (_turret) {
    // NEGATED: the turret servo + FireBank::estimateBearing are CCW-positive, but
    // the mapping frame is CW-positive (x fwd, y right). Without the flip the fire
    // localises mirrored about the forward axis.
    t.turret_angle = -(_turret->angle_ - SERVO_CENTER) * DEG_TO_RAD;  // servo deg (SERVO_CENTER=dead-fwd) -> rad, CW frame
    t.fire_locked  = _turret->locked_on_;
  } else {
    t.turret_angle = 0.0f;
    t.fire_locked  = false;
  }
  t.angle_error = -_fire_bank->estimateBearing() * DEG_TO_RAD;  // deg off-axis -> rad, CW frame

  // Order matches RobotModel::SensorType: ULTRASONIC, IR_FRONT_LEFT/RIGHT, IR_REAR_LEFT/RIGHT.
  // All non-blocking cache reads (service()/readSensor keep them fresh). Units: cm.
  t.range[0] = _ultrasonic->getAvg();
  t.range[1] = _front_left_ir->getAvg();
  t.range[2] = _front_right_ir->getAvg();
  t.range[3] = _rear_left_ir->getAvg();
  t.range[4] = _rear_right_ir->getAvg();
  return t;
}

void FireFighter::setBearing(float bearing) {
  bearing_ = bearing;
}

void FireFighter::testSensors() {
  print("us med: "); println(_ultrasonic->getAvg());

  print("front left:");
  println(_front_left_ir->getAvg());

  print("front right:");
  println(_front_right_ir->getAvg());

  print("rear left:");
  println(_rear_left_ir->getAvg());

  print("rear right:");
  println(_rear_right_ir->getAvg());
  println();
  println();
}

void FireFighter::timeStepData() {
  print(millis());
  print(",");

  print(_front_left_ir->mapping_reading_);
  print(",");

  print(_front_right_ir->mapping_reading_);
  print(",");

  print(_rear_left_ir->mapping_reading_);
  print(",");

  print(_rear_right_ir->mapping_reading_);
  print(",");

  print(_ultrasonic->mapping_reading_);
  println(";");
}

bool FireFighter::is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // The voltage of a LiPo cell varies from ~3.5V (discharged) = 717 ADC
  // to ~4.20-4.25V (fully charged) = 860 ADC. Lipo cell voltage should
  // never go below 3V; 3.5V is used as the safety floor here.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Low_voltage_counter = 0;
    return true;
  } else {
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

void FireFighter::updateIrSensors() {
  _front_left_ir->readSensor();
  _front_right_ir->readSensor();
  _rear_left_ir->readSensor();
  _rear_right_ir->readSensor();
}

#include "Arduino.h"
#include "HardwareSerial.h"
#include "firefighter.h"
#include "initialising.h"
#include "search.h"
#include "extinguish.h"
#include "stopped.h"

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
  _motors = new driveMotors();
  _front_left_ir = new LongRangeIR(front_left_ir_pin);
  _front_right_ir = new LongRangeIR(front_right_ir_pin);
  _rear_left_ir = new ShortRangeIR(rear_left_ir_pin);
  _rear_right_ir = new ShortRangeIR(rear_right_ir_pin);
  _ultrasonic = new Ultrasonic(ECHO_PIN, TRIG_PIN, MAX_DIST);
  ultrasonicISR = _ultrasonic;
  _motors->attatchAll();

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

  // Initialise all states up-front to avoid runtime allocation
  states_[State::INITIALISING] = new Initialising(this);
  states_[State::SEARCH]       = new Search(this);
  states_[State::EXTINGUISH]   = new Extinguish(this);
  states_[State::STOPPED]      = new Stopped(this);

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

void FireFighter::pollState() {
  _gyro->readSensor();
  // Refresh the four phototransistor EWMAs every loop so any state's poll()
  // can synchronously query "is there a fire?" without paying the ADC cost
  // itself. Cheap (~4 analogReads) compared with one ultrasonic ping.
  _fire_bank->update();
  if (current_state_) {
    current_state_->poll();
  }
}

void FireFighter::setBearing(float bearing) {
  bearing_ = bearing;
}

void FireFighter::testSensors() {
  print("us med: "); println(_ultrasonic->readBlocking());

  print("front left:");
  println(_front_left_ir->readSensor());

  print("front right:");
  println(_front_right_ir->readSensor());

  print("rear left:");
  println(_rear_left_ir->readSensor());

  print("rear right:");
  println(_rear_right_ir->readSensor());
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

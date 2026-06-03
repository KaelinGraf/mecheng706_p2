#include "HardwareSerial.h"
#include "Arduino.h"
#include"sensors.h"
float readVoltage(uint8_t pin){
  float adc_value = (analogRead(pin));
  return (adc_value / ADC_RANGE) * V_ADC;

}

float Sensor::readSensor(){
  return applyCalibration(float(readVoltage(_read_pin)));
}
float Sensor::readSensorRaw(){
  return readVoltage(_read_pin);
}
void Sensor::setReadPin(uint8_t new_pin){
  _read_pin = new_pin;
}
uint8_t Sensor::getReadPin(){
  return _read_pin;
}

Sensor::Sensor(uint8_t read_pin){
  _read_pin = read_pin;
}

float Sensor::readSensorFiltered(int nSamples, int delayMs) {
  // Cap at a reasonable max to avoid stack issues
  const int MAX_SAMPLES = 20;
  if (nSamples > MAX_SAMPLES) nSamples = MAX_SAMPLES;
  if (nSamples < 1) nSamples = 1;

  float samples[MAX_SAMPLES];
  int valid = 0;

  for (int i = 0; i < nSamples; i++) {
    float reading = readSensor();
    // Discard invalid readings: negative, NaN, or Inf
    if (!isnan(reading) && !isinf(reading) && reading > 0.0) {
      // Insertion sort to keep samples ordered (for median)
      int j = valid;
      while (j > 0 && samples[j - 1] > reading) {
        samples[j] = samples[j - 1];
        j--;
      }
      samples[j] = reading;
      valid++;
    }
    if (delayMs > 0 && i < nSamples - 1) {
      delay(delayMs);
    }
  }

  if (valid == 0) {
    mapping_reading_ = -1.0;
    return -1.0;
  }
  if (valid == 1) {
    mapping_reading_ = samples[0];
    return samples[0];
  }

  // Return the median (robust against outliers and ghost echoes)
  if (valid % 2 == 1) {
    mapping_reading_ = samples[valid / 2];
    return samples[valid / 2];
  } else {
    float read = (samples[valid / 2 - 1] + samples[valid / 2]) / 2.0;
    mapping_reading_ = read;
    return read;
  }
}


float ShortRangeIR::readSensor(){
  float val = applyCalibration(readVoltage(_read_pin));
  _prev_measurements->push(val);
  return val;
}

float ShortRangeIR::getAvg(){
  float median = _prev_measurements->median();
  mapping_reading_ = median;
  return median;
}

float ShortRangeIR::applyCalibration(float adc_voltage){
  //Impliments the calibration profile shown on the datasheet (only for the linear region)
  //Outside of the linear region should be treated as out-of-bounds (Returns -1)
  //Datasheet maps V = (1/[L + 0.42])m + c, between voltage ranges of 0.3V - 3V
  const float c = 0.1097f;
  const float m = 11.33f;
  float x = 0.0;
  if (adc_voltage < _min_voltage){
    return -1.0;
  }
  else if (adc_voltage > _max_voltage){
    return -1.0;
  }
  x = (adc_voltage - c) / m;
  return (1/x) - 0.42;
}

float LongRangeIR::readSensor(){
  float val = applyCalibration(readVoltage(_read_pin));
  _prev_measurements->push(val);
  return val;
}
float LongRangeIR::getAvg(){
  float median = _prev_measurements->median();
  mapping_reading_ = median;
  return median;
}


float LongRangeIR::applyCalibration(float adc_voltage){
  //Impliments the calibration profile shown on the datasheet (only for the linear region)
  //Outside of the linear region should be treated as out-of-bounds (Returns -1)
  //Datasheet maps V = (1/[L])m + c, thus L = 1/V between voltage ranges of 0.3V - 3V
  const float m = 18.744f;
  const float c = 0.3196f;
  if (adc_voltage < _min_voltage){
    return -1.0;
  }
  else if (adc_voltage > _max_voltage){
    return -1.0;
  }

  float val =  (1/((adc_voltage - c) / m));
  return val;
}

void Ultrasonic::initUltrasonic(){
  // Prime the first ping; service() takes over scheduling from here.
  runUltrasonic();
  _in_flight = true;
  _last_trigger_ms = millis();
}

void Ultrasonic::runUltrasonic(){
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);
}

Ultrasonic::Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, int max_dist)
: Sensor(uint8_t(255)),  _echo_pin(echo_pin), _trigger_pin(trigger_pin), _max_dist(max_dist){
  initUltrasonic();
  pinMode(echo_pin, INPUT);
  _prev_measurements = new RingBuffer<float, 3>();
};
// Non-blocking query: ALWAYS returns the last good cached value. Never triggers,
// never blocks. Call service() once per loop to keep the cache fresh.
float Ultrasonic::readSensor() {
  mapping_reading_ = _cached_cm;
  return _cached_cm;
};

// Pump once per loop. Folds a completed echo into the cache, abandons a stalled
// ping, and schedules the next one. Triggering is decoupled from reading so that
// readSensor()/getAvg() can be called any number of times with no side effects.
void Ultrasonic::service() {
  unsigned long now = millis();

  // 1) Consume a completed echo. The ISR sets sent_time on the rising edge and
  //    return_time on the falling edge, so return_time > sent_time means the
  //    current ping finished. Snapshot both atomically (4-byte reads vs the ISR).
  unsigned long t_sent, t_ret;
  noInterrupts();
  t_sent = sent_time;
  t_ret  = return_time;
  interrupts();

  if (_in_flight && t_ret > t_sent && t_ret != _last_consumed_return) {
    _last_consumed_return = t_ret;
    _in_flight = false;                   // ping RESOLVED (regardless of in/out of band)
    _last_resolved_ms = now;
    float cm = (t_ret - t_sent) / 58.0f;
    if (cm >= 10.0f && cm <= 200.0f) {    // valid band: refresh cache + history
      _cached_cm = cm;
      if (_prev_measurements) _prev_measurements->push(cm);
    }
    // Out-of-band returns keep the previous cached value (don't poison it).
  }

  // 2) Watchdog: a ping that NEVER echoes (out of range / absorbed / missed edge)
  //    must not stall the sensor forever. Abandon it so a fresh ping can fire.
  if (_in_flight && (now - _last_trigger_ms > ECHO_TIMEOUT_MS)) {
    _in_flight = false;
    _last_resolved_ms = now;
  }

  // 3) Re-ping once the previous one has RESOLVED (echo or timeout) + a short quiet
  //    gap, so the previous ping's reverb can't contaminate the next measurement.
  if (!_in_flight && (now - _last_resolved_ms >= QUIET_GAP_MS)) {
    runUltrasonic();
    _in_flight = true;
    _last_trigger_ms = now;
  }
}

float Ultrasonic::readBlocking() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(US_INT_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (_max_dist + 1000)) {
      if (DIAGNOSTICS){
        Serial.println("HC-SR04: NOT found");
      }

      mapping_reading_ = -1.0;
      return -2.0;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(US_INT_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (_max_dist + 1000)) {
      if (DIAGNOSTICS){
        Serial.println("HC-SR04: Out of range - Waiting");
      }
      mapping_reading_ = -1.0;
      return -1.0;
    }
  }
  
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (DIAGNOSTICS){
    if (pulse_width > _max_dist) {
      Serial.println("HC-SR04: Out of range - Calced");
    }
  }

  mapping_reading_ = cm;
  return cm;
}

float Ultrasonic::getAvg(){
  // service() maintains the history on each fresh ping, so just read the median.
  float median = _prev_measurements->median();
  mapping_reading_ = median;
  return mapping_reading_;
}



float Gyroscope::readSensor(bool apply_filter){
  // Re-enable the correct report if the sensor resets
  if (_bno08x->wasReset()) {
    _bno08x->enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  }

  if (_bno08x->getSensorEvent(_sensorValue)) {
    // Check for the Game Rotation Vector report
    if (_sensorValue->sensorId == SH2_GAME_ROTATION_VECTOR) {
      
      // 1. Extract quaternion components
      float qr = _sensorValue->un.gameRotationVector.real;
      float qi = _sensorValue->un.gameRotationVector.i;
      float qj = _sensorValue->un.gameRotationVector.j;
      float qk = _sensorValue->un.gameRotationVector.k;

      // 2. Convert quaternion to Yaw (Z-axis rotation) in radians
      float raw_yaw = atan2(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (qj * qj + qk * qk));
      
      // 3. Apply the offset so resetAngle() still works
      _rad = raw_yaw - _yaw_offset;

      // Optional: Normalize _rad to be exactly within [-PI, PI]
      while (_rad > PI) _rad -= 2.0f * PI;
      while (_rad < -PI) _rad += 2.0f * PI;

      _prev_measurements->push(_rad);
      return (apply_filter)? _prev_measurements->average() : _rad;
    }
  }
  return -1001.0;
};

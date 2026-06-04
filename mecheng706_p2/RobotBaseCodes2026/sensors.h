#ifndef SENSORS_H
#define SENSORS_H

#include "HardwareSerial.h"
#include <stdint.h>
#include "mappings.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>  //Need for Gyroscope
#include "utils.h"

//Class methods for reading and calibrating different sensor variants

float readVoltage(uint8_t pin);


class Sensor{
  protected:
    uint8_t _read_pin;
    virtual float applyCalibration(float adc_voltage)=0; //Contains the math for each sensor that maps ADC values to a meaningful value

  public:
    Sensor(uint8_t read_pin);
    virtual float readSensor();
    float readSensorRaw();
    float readSensorFiltered(int nSamples, int delayMs = 0);
    virtual void setReadPin(uint8_t new_pin);
    virtual uint8_t getReadPin();
    float mapping_reading_;

};


class ShortRangeIR: public Sensor{
  private:
    uint32_t _last_millis;
    float _min_voltage = 0.45;
    float _max_voltage = 2.9;
    RingBuffer<float,3>* _prev_measurements;


  public:
    ShortRangeIR(uint8_t read_pin) : Sensor(read_pin){
      pinMode(read_pin, INPUT);
      _last_millis = millis();
      _prev_measurements = new RingBuffer<float, 3>();
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
    float getAvg();
    inline void resetBuffer(){this->_prev_measurements->reset();};
};


class LongRangeIR: public Sensor{
  private:
    uint32_t _last_millis;
    float _min_voltage = 0.4;
    float _max_voltage = 3.0;
    RingBuffer<float,3>* _prev_measurements;

  public:
    LongRangeIR(uint8_t read_pin) : Sensor(read_pin){
      pinMode(read_pin, INPUT);
      _last_millis = millis();
      _prev_measurements = new RingBuffer<float, 3>();
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
    float getAvg();
    inline void resetBuffer(){this->_prev_measurements->reset();};
};

class Ultrasonic: public Sensor{
  private:
    uint8_t _echo_pin = ECHO_PIN;
    uint8_t _trigger_pin = TRIG_PIN;
    unsigned int _max_dist = MAX_DIST;
    // Timestamps written by the echo ISR -> must be volatile.
    volatile unsigned long sent_time = 0;
    volatile unsigned long return_time = 0;
    // Non-blocking measurement state (mapping-branch service() design).
    float _cached_cm = -1.0f;              // last good reading; readSensor() returns this
    bool  _in_flight = false;              // a ping is out, awaiting its echo
    unsigned long _last_trigger_ms = 0;
    unsigned long _last_resolved_ms = 0;   // when the last ping finished (echo OR timeout)
    unsigned long _last_consumed_return = 0;
    static const unsigned long QUIET_GAP_MS    = 10;  // gap AFTER a ping resolves; lets reverb die (tune)
    static const unsigned long ECHO_TIMEOUT_MS = 30;  // watchdog: no echo by now => abandon, re-ping
    RingBuffer<float,3>* _prev_measurements;

  public:
    Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, int max_dist);

    float readSensor() override;
    inline float applyCalibration(float adc_voltage) override {return adc_voltage;};
    float readBlocking();
    void service();      // pump once per loop: schedule pings + consume echoes (non-blocking)
    void initUltrasonic();
    void runUltrasonic();
    void setSentTime(unsigned long t1){sent_time = t1;};
    unsigned long getSentTime(){return sent_time;};
    void setReturnTime(unsigned long t2){return_time = t2;};
    unsigned long getReturnTime(){return return_time;};
    float getAvg();
    inline void resetBuffer(){this->_prev_measurements->reset();};


};

class Gyroscope: public Sensor{
  private:
    Adafruit_BNO08x*   _bno08x;
    sh2_SensorValue_t* _sensorValue;
    HardwareSerial*    _serial_com;
    float _heading  = 0.0f;   // continuous, unwrapped fused yaw (rad) -- drift-corrected by the BNO
    float _last_yaw = 0.0f;   // previous wrapped yaw, used to unwrap into _heading
    float _offset   = 0.0f;   // reference for getAngle() (zeroed by resetAngle)
    bool  _have_yaw = false;  // re-anchors the unwrap on the first reading and after a BNO reset

  public:
    Gyroscope(Adafruit_BNO08x* bno08x, sh2_SensorValue_t* sensorValue, HardwareSerial* SerialCom)
      : Sensor(uint8_t(0)), _bno08x(bno08x), _sensorValue(sensorValue), _serial_com(SerialCom) {
      // Settle delay: let the BNO08x finish its power-on reset before we talk to it. On a cold boot
      // (especially with the motor rail loaded) the chip needs ~200 ms after power before it cleanly
      // accepts a report enable; otherwise the enable is acknowledged but the report never streams and
      // getHeading() reads a constant 0. Boot-time only.
      delay(300);
      _serial_com->println(F("[BOOT] Enabling gyroscope (BNO08x over I2C)..."));
      // 6-axis FUSED orientation (accel+gyro, no magnetometer -> immune to motor EMI), drift-corrected
      // on-chip. (The old branch enabled SH2_GYROSCOPE_CALIBRATED here -- the raw rate -- which both
      // drifts when integrated AND mismatched the GAME_ROTATION_VECTOR id readSensor() looks for, so the
      // heading could sit stuck until a chance reset.) Block until the IMU answers and print every
      // failed attempt (ungated) so a bad I2C connection is obvious instead of a silent dead robot.
      unsigned long imu_tries = 0;
      while (!_bno08x->begin_I2C() ||
             !_bno08x->enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {  // 100 Hz
        _serial_com->print(F("[BOOT][ERR] BNO08x not found on I2C -- check wiring / power / address. retry "));
        _serial_com->println(++imu_tries);
        delay(250);
      }
      _serial_com->println(F("[BOOT] gyroscope online"));
    }

    float readSensor() override;                                 // poll BNO, advance the continuous heading
    inline float applyCalibration(float adc_voltage) override { return adc_voltage; }

    // Continuous absolute heading (rad) -- NEVER reset.
    float getHeading() const { return _heading; }
    // Heading since the last resetAngle() -- used by spin-scan / tracking RETURN_TO_HEADING.
    float getAngle()   const { return _heading - _offset; }
    void  resetAngle()       { _offset = _heading; }

    static float wrapPi(float a) {
      while (a >  PI) a -= TWO_PI;
      while (a < -PI) a += TWO_PI;
      return a;
    }
};


struct gyroData{


};


#endif

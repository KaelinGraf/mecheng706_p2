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
    unsigned long sent_time = 0;
    unsigned long return_time = 0;
    unsigned long _last_sent = 0;
    RingBuffer<float,3>* _prev_measurements;

  public:
    Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, int max_dist);

    float readSensor() override;
    inline float applyCalibration(float adc_voltage) override {return adc_voltage;};
    float readBlocking();
    void initUltrasonic();
    void runUltrasonic();
    void setSentTime(unsigned long t1){sent_time = t1;};
    unsigned long getSentTime(){return sent_time;};
    void setReturnTime(unsigned long t2){return_time = t2;};
    unsigned long getReturnTime(){return return_time;};
    void setLastSent(unsigned long last_sent){_last_sent = last_sent;};
    unsigned long getLastSent(){return _last_sent;};
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

    void configureGameRotationVector() {
      if (!_bno08x->enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
        _serial_com->println("Could not enable game vector");
      }
    }

  public:
    Gyroscope(Adafruit_BNO08x* bno08x, sh2_SensorValue_t* sensorValue, HardwareSerial* SerialCom)
      : Sensor(uint8_t(0)), _bno08x(bno08x), _sensorValue(sensorValue), _serial_com(SerialCom) {

      _serial_com->println("Enabling Gyroscope...");
      // Follow the Adafruit BNO08x pattern: initialize once, then enable the
      // fused game rotation vector report used for heading.
      while (!_bno08x->begin_I2C()) {
        _serial_com->println("Failed to find BNO08x chip");
        delay(10);
      }

      _serial_com->println("BNO08x Found!");
      configureGameRotationVector();
      _serial_com->println("Gyro Success!");
    }

    float readSensor() override;                                 // poll BNO, advance the continuous heading
    inline float applyCalibration(float adc_voltage) override { return adc_voltage; }

    // Continuous absolute heading for mapping/nav -- NEVER reset, so spin-scan's
    // resetAngle() can't yank the map frame out from under the dead-reckoner.
    float getHeading() const { return _heading; }
    // Heading since the last resetAngle() -- used by spin-scan / tracking.
    float getAngle()   const { return _heading - _offset; }
    void  resetAngle()       { _offset = _heading; }

    static float wrapPi(float a) {
      while (a >  PI) a -= TWO_PI;
      while (a < -PI) a += TWO_PI;
      return a;
    }
    void setHeading(float new_head) { _heading = new_head; }
};


#endif

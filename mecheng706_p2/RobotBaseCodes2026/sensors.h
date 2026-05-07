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
    float _prev_reading;
    float _kalman_estimate;
    float _min_voltage = 0.4;
    float _max_voltage = 3.0;
    float _last_y_var = 0.1;
    float process_noise_ = 0.001;
    float sensor_noise_ = 0.2;
    RingBuffer<float,3>* _prev_measurements;

  public:
    LongRangeIR(uint8_t read_pin) : Sensor(read_pin){
      _last_millis = millis();
      _prev_reading = -1.0;
      _kalman_estimate = -1.0;
      _prev_measurements = new RingBuffer<float, 3>();
    }
    float readSensor() override;
    float readSensorKalman();
    void resetKalman();
    float applyCalibration(float adc_voltage) override;
    inline float getKalmanEst() {return _kalman_estimate; };
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
    Adafruit_BNO08x* _bno08x;
    sh2_SensorValue_t* _sensorValue;
    float _rad = 0.0;
    float _last_omega = 0.0;
    uint32_t _prev_micros = 0.0;
    HardwareSerial* _serial_com;
    RingBuffer<float,4>* _prev_measurements;
    float _deadband = 0.01;



  public:
    Gyroscope(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom):Sensor(uint8_t(0)),_bno08x(bno08x),_sensorValue(sensorValue),_serial_com(SerialCom){
      _prev_measurements = new RingBuffer<float,4>();
      _prev_micros = micros();
      _serial_com->println("Enabling Gyroscope...");

      //while (!_bno08x->begin_I2C() ||
      //    !_bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
      //      _serial_com->println("IMU failed");
      //    }

      //_bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED,10000);
      
    };
    float readSensor(bool apply_filter = false);
    inline float applyCalibration(float adc_voltage) override {return adc_voltage;};
    void resetAngle() { _rad = 0.0; _prev_micros = micros(); }
    float getAngle() { return _rad; }



};


struct gyroData{


};


#endif

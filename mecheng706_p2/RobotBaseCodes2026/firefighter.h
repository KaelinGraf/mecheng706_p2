#ifndef FIREFIGHTER_H
#define FIREFIGHTER_H

#include "SoftwareSerial.h"
#include <stdint.h>
#include <Arduino.h>
#include "state.h"
#include "sensors.h"
#include "fire.h"
#include "servo_control.h"
#include "pid.h"

class FireFighter
{
private:
    State *current_state_;
    State *states_[State::NUM_STATES];
    HardwareSerial *serialCom_;
    SoftwareSerial *btSerial_ = nullptr;
    long _lastExtinguisedMillis = 0;

    // Counter shared across EXTINGUISH triggers so the FSM knows when the
    // task is over (after the second fire, the brief requires us to stop).
    int fires_extinguished_ = 0;

public:
    LongRangeIR *_front_left_ir;
    LongRangeIR *_front_right_ir;
    ShortRangeIR *_rear_right_ir;
    ShortRangeIR *_rear_left_ir;
    Gyroscope *_gyro;
    Ultrasonic *_ultrasonic;
    driveMotors *_motors;

    // Fire-detection bank + fan (added for Project 2). The four photocells are
    // owned by the FireFighter and aggregated through _fire_bank for the FSM.
    Phototransistor *_photo_sl;
    Phototransistor *_photo_l;
    Phototransistor *_photo_r;
    Phototransistor *_photo_sr;
    FireBank        *_fire_bank;
    Fan             *_fan;

    // Fire counter accessors (used by Extinguish to drive the SEARCH/STOPPED
    // transition once two fires are out).
    inline int  firesExtinguished() const { return fires_extinguished_; }
    inline void noteFireExtinguished()    { 
        _lastExtinguisedMillis = millis();
        print("Extinguished at: ");
        println(_lastExtinguisedMillis);
        fires_extinguished_++; 
    }
    inline bool recentExtinguish() const { return millis() - _lastExtinguisedMillis < EXTINGUISH_TIMEOUT_MS; }

    FireFighter(Adafruit_BNO08x *bno08x, sh2_SensorValue_t *sensorValue, HardwareSerial *SerialCom);
    ~FireFighter();

    void testSensors();
    void timeStepData();
    bool switchState(State::Name newState, StateData data = {-1.0, false, false});
    void pollState();
    void setBearing(float bearing);
    float bearing_ = 0.0f;

    inline void setSerialCom(HardwareSerial *serialCom) { serialCom_ = serialCom; };
    inline void setBluetoothSerial(SoftwareSerial *btSerial) { btSerial_ = btSerial; };

    template <typename... Args>
    inline void print(Args... args)
    {
        if (serialCom_)
            serialCom_->print(args...);
        if (btSerial_)
            btSerial_->print(args...);
    }
    template <typename... Args>
    inline void println(Args... args)
    {
        if (serialCom_)
            serialCom_->println(args...);
        if (btSerial_)
            btSerial_->println(args...);
    }

    bool is_battery_voltage_OK();

    inline State::Name getCurrentState() { return current_state_ ? current_state_->getState() : State::NUM_STATES; }
};

#endif // FIREFIGHTER_H

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
#include "behavior.h"

// Mapping layer is forward-declared only: firefighter.h is included BY the
// mapping headers (occupancy_grid.h, world_model.hpp), so it must not include
// them back. firefighter.cpp pulls in the full definitions under ENABLE_MAPPING.
class WorldModel;
struct RobotModel;


struct Tap{
    //Tap becomes the package of information passed to world model each tick
    float heading; //game vector gyro heading
    float vx, vy; //last commanded velocities
    float turret_angle, angle_error = 0; //angle error is recorded angle discrepancy between turret and fire this tick 
    bool fire_locked; //is turret locking to a fire
    bool close_to_fire = false;
    BehaviorNS::SearchBehaviour search_behaviour = BehaviorNS::SearchBehaviour::FIND_FIRE;
    float range[5]; //Sensor readings in order: ULTRASONIC, IR_FRONT_LEFT, IR_FRONT_RIGHT, IR_REAR_LEFT, IR_REAR_RIGHT
};

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

    // Onboard mapping pipeline. Pointers (forward-declared types) so this header
    // need not see the mapping definitions; instantiated only under ENABLE_MAPPING.
    WorldModel *_world = nullptr;
    RobotModel *_robot_model = nullptr;
    BehaviorNS::SearchBehaviour _search_behaviour = BehaviorNS::SearchBehaviour::FIND_FIRE;

public:
    ShortRangeIR *_front_left_ir;
    ShortRangeIR *_front_right_ir;
    LongRangeIR *_rear_right_ir;
    LongRangeIR *_rear_left_ir;
    Gyroscope *_gyro;
    Ultrasonic *_ultrasonic;
    driveMotors *_motors;
    // The turret is created in the .ino after the FireFighter; setTurret() wires
    // it in so buildTap() can read turret angle / lock state for fire localising.
    Turret *_turret = nullptr;

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

    inline void setSearchBehavior(BehaviorNS::SearchBehaviour behavior) { _search_behaviour = behavior; }
    inline BehaviorNS::SearchBehaviour getSearchBehavior() const { return _search_behaviour; }
    inline WorldModel* worldModel() const { return _world; }

    // Wire in the turret (owned by the .ino) so buildTap() can reach it.
    inline void setTurret(Turret *t) { _turret = t; }
    // Consolidate this tick's inputs (pose/heading, last command, turret, ranges)
    // into the snapshot the WorldModel consumes. Single point for unit conversion.
    Tap buildTap();

    inline void setSerialCom(HardwareSerial *serialCom) { serialCom_ = serialCom; };
    inline void setBluetoothSerial(SoftwareSerial *btSerial) { btSerial_ = btSerial; };
    void updateIrSensors();

    // Print helpers. These ALWAYS route to BOTH links (USB serialCom_ + Bluetooth
    // btSerial_) when present, so anything you ff->print() — including the
    // occupancy frames from WorldModel::printSet() — auto-reaches the MATLAB viewer
    // over Bluetooth, no SERIAL_DEBUG gate required. Bandwidth note: SoftwareSerial
    // at 115200 is the bottleneck, so keep per-tick chatter light or it crowds the
    // ~3 KB occupancy frame.
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

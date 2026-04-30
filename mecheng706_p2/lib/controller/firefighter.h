#ifndef FIREFIGHTER_H
#define FIREFIGHTER_H

#include <stdint.h>
#include "state.h"
#include "sensors.h"
#include "fire.h"
#include "servo_control.h"
#include "pid.h"
#include "hardware_context.h"

class FireFighter
{
private:
    HardwareContext _hw;
    State *current_state_;
    State *states_[State::NUM_STATES];

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
    Phototransistor *_photo_front;
    Phototransistor *_photo_right;
    Phototransistor *_photo_rear;
    Phototransistor *_photo_left;
    FireBank        *_fire_bank;
    Fan             *_fan;

    // Fire counter accessors (used by Extinguish to drive the SEARCH/STOPPED
    // transition once two fires are out).
    inline int  firesExtinguished() const { return fires_extinguished_; }
    inline void noteFireExtinguished()    { fires_extinguished_++; }

    // Construct the controller against a HardwareContext that supplies all
    // hardware-interactive primitives. The context's pointers must outlive
    // this FireFighter instance (typically they live in the entry point's
    // setup() / module init scope).
    explicit FireFighter(const HardwareContext& hw);
    ~FireFighter();

    void testSensors();
    void timeStepData();
    bool switchState(State::Name newState, StateData data = {-1.0, false, false});
    void pollState();

    // Accessors used by the state classes for timing and logging.
    inline IClock* clock() { return _hw.clock; }
    inline const HardwareContext& hardware() const { return _hw; }

    template <typename... Args>
    inline void print(Args... args)
    {
        if (_hw.printer_primary)   _hw.printer_primary->print(args...);
        if (_hw.printer_secondary) _hw.printer_secondary->print(args...);
    }
    template <typename... Args>
    inline void println(Args... args)
    {
        if (_hw.printer_primary)   _hw.printer_primary->println(args...);
        if (_hw.printer_secondary) _hw.printer_secondary->println(args...);
    }

    bool is_battery_voltage_OK();

    inline State::Name getCurrentState() { return current_state_ ? current_state_->getState() : State::NUM_STATES; }
};

#endif // FIREFIGHTER_H

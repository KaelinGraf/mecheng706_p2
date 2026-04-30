#ifndef FIREFIGHTER_H
#define FIREFIGHTER_H

#include <stdint.h>
#include "sensors.h"
#include "fire.h"
#include "sfh300fa.h"
#include "servo_control.h"
#include "pan_servo.h"
#include "photo_array.h"
#include "imu.h"
#include "turret_controller.h"
#include "motion_controller.h"
#include "behavior_arbiter.h"
#include "mission_complete_behavior.h"
#include "avoid_obstacle_behavior.h"
#include "extinguish_fire_behavior.h"
#include "move_to_fire_behavior.h"
#include "find_fire_behavior.h"
#include "pid.h"
#include "hardware_context.h"

// Owns the controller's sensors, motion layer, panning turret, and the
// behavior arbiter that replaces the legacy FSM. Built once in the entry
// point against a HardwareContext (Arduino or sim); tick() is called once
// per main-loop iteration.
class FireFighter
{
private:
    HardwareContext _hw;

    // Mission progress: incremented by ExtinguishFireBehavior via the
    // arbiter pipeline; surfaced to MissionCompleteBehavior through the
    // WorldView each tick.
    int fires_extinguished_ = 0;

public:
    LongRangeIR  *_front_left_ir;
    LongRangeIR  *_front_right_ir;
    ShortRangeIR *_rear_right_ir;
    ShortRangeIR *_rear_left_ir;
    Gyroscope    *_gyro;          // legacy omega integrator; still cheap
    IMU          *_imu;            // quaternion-derived yaw + accel
    Ultrasonic   *_ultrasonic;
    driveMotors  *_motors;
    MotionController *_motion;
    Fan          *_fan;

    // Panning turret carrying the boom-mounted fan + SFH 300 FA photo array.
    SFH300FA          *_turret_photo_left;
    SFH300FA          *_turret_photo_right;
    PhotoArray        *_turret_photos;
    PanServo          *_pan_servo;
    TurretController  *_turret;

    // Behavior arbiter + the 5 layers, top-priority first.
    BehaviorArbiter         *_arbiter;
    MissionCompleteBehavior *_b_mission_complete;
    AvoidObstacleBehavior   *_b_avoid_obstacle;
    ExtinguishFireBehavior  *_b_extinguish_fire;
    MoveToFireBehavior      *_b_move_to_fire;
    FindFireBehavior        *_b_find_fire;

    inline int  firesExtinguished() const { return fires_extinguished_; }
    inline void noteFireExtinguished()    { fires_extinguished_++; }

    explicit FireFighter(const HardwareContext& hw);
    ~FireFighter();

    void testSensors();
    void timeStepData();

    // Run one control-loop iteration: refreshes the gyro, ticks the turret
    // independently, builds a WorldView snapshot, runs the arbiter, and
    // applies the winning BehaviorOutput to motion / fan / turret-hold.
    void tick();

    // Legacy alias retained so the entry-point .ino does not have to be
    // touched in lock-step with controller refactors.
    inline void pollState() { tick(); }

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
};

#endif // FIREFIGHTER_H

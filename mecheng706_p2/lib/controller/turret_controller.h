#ifndef TURRET_CONTROLLER_H
#define TURRET_CONTROLLER_H

#include <stdint.h>
#include "pan_servo.h"
#include "photo_array.h"
#include "imu.h"
#include "iclock.h"

// Independent control loop for the panning turret carrying the boom-mounted
// fan + SFH 300 FA photodiode array. Runs once per main loop iteration in
// parallel with the body controller (FSM in step 3, behavior arbiter from
// step 5 onwards).
//
// State machine:
//   IDLE   - servo parked at 0 rad. Used at startup and as a safe default
//            on shutdown.
//   SWEEP  - triangle-wave pan from min_rad to max_rad. Promotes to LOCKED
//            when the array reads above FIRE_DETECT_V for `lock_count_in`
//            consecutive ticks.
//   LOCKED - hold the world-frame bearing of the candle as the body
//            rotates: pan_cmd = world_lock_yaw - body_yaw + photo_error*Kp.
//            Demotes back to SWEEP when intensity stays below
//            FIRE_LOST_V for `lock_count_out` consecutive ticks.
//
// Body-yaw source: the IMU wrapper exposes the BNO's quaternion-derived
// absolute yaw (drift-bounded; chosen over the integrated-gyro angle for
// the world-bearing-hold math).
class TurretController {
public:
    enum State { IDLE, SWEEP, LOCKED };

    struct Snapshot {
        State state;
        bool  is_locked;
        float body_relative_bearing_rad;   // turret pan command, body frame
        float world_bearing_rad;           // body yaw + body_relative
        float intensity;                   // max photodiode reading (V)

        Snapshot()
            : state(IDLE), is_locked(false),
              body_relative_bearing_rad(0.0f),
              world_bearing_rad(0.0f),
              intensity(0.0f) {}
    };

    struct Config {
        // Sweep
        float sweep_rate_rad_per_s;     // ~92 deg/s sweep
        // Lock thresholds
        float lock_threshold_v;         // matches FIRE_DETECT_V
        float lost_threshold_v;         // matches FIRE_OUT_V
        uint8_t lock_count_in;
        uint8_t lock_count_out;
        // Tracking gain when LOCKED. The photo array's bearing_error is
        // already in rad; this gain scales how aggressively we recentre.
        float track_kp;

        Config()
            : sweep_rate_rad_per_s(1.6f),
              lock_threshold_v(1.6f),
              lost_threshold_v(0.7f),
              lock_count_in(5),
              lock_count_out(8),
              track_kp(0.6f) {}
    };

    TurretController(PanServo*  servo,
                     PhotoArray* photos,
                     IMU*        imu,
                     IClock*     clock,
                     Config      cfg = Config());

    // Move from IDLE to SWEEP. Call once after begin() of all peripherals.
    void start();

    // Boolean hold latch. While true: in SWEEP the pan command freezes;
    // in LOCKED the auto-demotion to SWEEP on dim readings is suppressed.
    // Behaviors set this each tick (e.g. ExtinguishFireBehavior wants the
    // boom-mounted fan to keep aiming while the LED extinguishes).
    void set_hold(bool hold);

    // Force a return to SWEEP from any state. Clears the hold latch.
    void request_sweep();

    void tick();

    Snapshot snapshot() const;

    inline State state() const { return _state; }
    inline bool  is_locked() const { return _state == LOCKED; }

private:
    void _enter(State s);
    float _body_yaw() const;

    PanServo*   _servo;
    PhotoArray* _photos;
    IMU*        _imu;
    IClock*     _clock;
    Config      _cfg;

    State    _state;
    float    _pan_cmd;             // body-frame command (rad)
    int8_t   _sweep_dir;           // +1 / -1
    uint8_t  _lock_in_count;
    uint8_t  _lock_out_count;
    float    _world_lock_yaw;      // captured at lock onset
    uint32_t _last_tick_us;
    bool     _hold_request;
};

#endif

#ifndef WORLD_VIEW_H
#define WORLD_VIEW_H

#include <stdint.h>
#include "turret_controller.h"

// Read-only snapshot of every sensor and subsystem that behaviors may
// consume. Built once per loop iteration by FireFighter::tick() and passed
// by const-ref to the BehaviorArbiter, which forwards it to each
// IBehavior::tick(WorldView&). Behaviors do not read sensors directly -
// the WorldView decouples the behavior layer from sensor classes and
// guarantees that all behaviors see a consistent snapshot per tick.
struct WorldView {
    // ------- Body distance sensors (cm). -1 = invalid / out of range. ----
    float us_cm           = -1.0f;
    float front_left_ir   = -1.0f;
    float front_right_ir  = -1.0f;
    float rear_left_ir    = -1.0f;
    float rear_right_ir   = -1.0f;

    // ------- Turret subsystem snapshot ----------------------------------
    TurretController::Snapshot turret;

    // ------- IMU (yaw integrated from gyro through step 5; quaternion-
    // derived absolute yaw from step 6 onwards). ------------------------
    float body_yaw_rad    = 0.0f;
    float body_omega_z    = 0.0f;

    // ------- Mission state ---------------------------------------------
    int   fires_extinguished = 0;

    // ------- Time (ms since boot, monotonic) ---------------------------
    unsigned long now_ms  = 0;
};

#endif

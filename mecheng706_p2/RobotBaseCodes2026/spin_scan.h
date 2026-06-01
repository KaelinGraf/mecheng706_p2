#ifndef SPIN_SCAN_H
#define SPIN_SCAN_H

#include "state.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// SPIN_SCAN
// ---------------------------------------------------------------------------
// Rotate the chassis in place (turret held dead-centre) until the OUTER
// phototransistor pair detects the fire, then hand off to SEARCH so the
// existing turret lock/track + approach logic takes over.
//
// Entered:
//   * at startup (from INITIALISING), and
//   * after the FIRST fire is extinguished (from EXTINGUISH).
//
// The "fire found" test uses the same gate as the turret lock
// (FireBank::bothOuterAbove(FIRE_LOCK_OUTER_V)), so the instant we switch to
// SEARCH the turret locks on within the same loop. If a full revolution finds
// nothing, we fall back to SEARCH (normal drive + turret pan-scan wander).
//
// While in this state, RobotBaseCodes2026.ino suppresses updateTurret()/
// testSensors(), so SpinScan owns the fire-bank sampling and the motors.
// ---------------------------------------------------------------------------
class SpinScan : public State {
  private:
    float max_intensity_;
    float best_angle_;
    bool returning_to_best_;
    float prev_angle_;
    float total_swept_angle_;
    
  public:
    SpinScan(FireFighter* firefighter) : State(State::SPIN_SCAN, firefighter) {};
    ~SpinScan() {};

    void begin() override;
    void end() override;
    void poll() override;
};

#endif // SPIN_SCAN_H

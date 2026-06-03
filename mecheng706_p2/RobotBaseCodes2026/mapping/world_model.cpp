#include "world_model.hpp"

// ---------------------------------------------------------------------------
// WorldModel::printSet — occupancy serial protocol v1 emitter.
//
// Emits ONE frame: a header `F` line, then WIN_N row `R` lines, then an `E`
// line. EVERY line is prefixed with the literal token `occupancy` so a consumer
// can ignore any other chatter on the same UART (see mapping/OCCUPANCY_PROTOCOL.md).
// Field/token order matches that spec exactly:
//
//   occupancy F <seq> <n> <cell_cm> <org_cx> <org_cy>
//               <pose_x> <pose_y> <pose_th> <fire_x> <fire_y> <fire_conf>
//               <goal_x> <goal_y>
//   occupancy R <y> <v0> <v1> ... <v(n-1)>          (x = 0..n-1, raw log-odds)
//   ... exactly n R lines, y = 0..n-1 ...
//   occupancy E <seq>
//
// NON-BLOCKING, stepped emitter: emits AT MOST ONE protocol line per call (the F
// header, then one R row per call, then the E line) and self-throttles to
// PRINT_OCC_MS between frames. pollState() calls this EVERY loop. Emitting the whole
// ~3 KB frame in a single call blocked the loop ~quarter-second over the 115200 link,
// which starved the turret update (servo target froze, then snapped) — that was the
// pan-scan stutter. One line per call keeps each loop's serial cost tiny so the
// turret pans smoothly. Lines stream through FireFighter::print/println (always-on,
// dual-port USB + Bluetooth). Other serial chatter may interleave BETWEEN lines; the
// host parser ignores any line not starting with `occupancy` and only completes a
// frame on the E whose seq matches the F, so interleaving is harmless.
// ---------------------------------------------------------------------------
void WorldModel::printSet(){
    // Stepped state (the .hpp can't be touched, so keep it function-local):
    //   line == -1            -> idle between frames (waiting out PRINT_OCC_MS)
    //   line == 0             -> emit the F header
    //   line == 1..WIN_N      -> emit row y = line-1
    //   line == WIN_N+1 path  -> emit the E line, return to idle
    static unsigned long seq = 0;
    static unsigned long this_seq = 0;
    static int           line = -1;
    static unsigned long last_start_ms = 0;

    if (line < 0) {                                   // idle: start a frame on schedule
        if (millis() - last_start_ms < PRINT_OCC_MS) return;
        last_start_ms = millis();
        this_seq = seq++;
        line = 0;
    }

    const OccupancyGrid& g = grid();

    if (line == 0) {                                  // ---- Header line F ----
        const Pose2D rp = robotPose();      // x,y in cm; th in rad
        const Pose2D fe = fireEstimate();   // x,y in cm; .th carries confidence [0..1]
        _ff->print("occupancy F ");
        _ff->print(this_seq);          _ff->print(' ');
        _ff->print((int)WIN_N);        _ff->print(' ');
        _ff->print((float)CELL_CM);    _ff->print(' ');   // cell size, cm
        _ff->print(g.originX());       _ff->print(' ');   // window origin X, world cells
        _ff->print(g.originY());       _ff->print(' ');   // window origin Y, world cells
        _ff->print(rp.x);              _ff->print(' ');   // robot X, cm
        _ff->print(rp.y);              _ff->print(' ');   // robot Y, cm
        _ff->print(rp.th, 4);          _ff->print(' ');   // robot heading, rad
        _ff->print(fe.x);              _ff->print(' ');   // fire X, cm (0 if no fix)
        _ff->print(fe.y);              _ff->print(' ');   // fire Y, cm (0 if no fix)
        _ff->print(fe.th, 4);          _ff->print(' ');   // fire confidence [0..1]
        _ff->print(-1);                _ff->print(' ');   // goal_x: no planner yet
        _ff->println(-1);                                 // goal_y: no planner yet
        line = 1;
        return;
    }

    if (line <= WIN_N) {                              // ---- one Row line R (y=line-1) ----
        const int y = line - 1;
        _ff->print("occupancy R ");
        _ff->print(y);
        for (int x = 0; x < WIN_N; ++x) {
            _ff->print(' ');
            _ff->print((int)g.logOddsAt(x, y));   // raw int8 log-odds [-LOC_CLAMP..+LOC_CLAMP]
        }
        _ff->println();
        line++;
        return;
    }

    // ---- End line E ----
    _ff->print("occupancy E ");
    _ff->println(this_seq);
    line = -1;                                        // frame done -> back to idle
}

void WorldModel::update(Tap* tap){
    /*
    Performs a full update of the entire dead reckoning -> occupancy map + fire localisation pipeline
    Does NOT interact with the navigator. The navigator is only queried for a command signal by appropriate sub-states in FSM.
    Navigator replans are also called asynchronously as needed.
    */
   long now = micros();
   //First update pose (affects all downstream functions)
    _deadReckoner.updatePose(tap->heading,tap->vx,tap->vy,(now-_lastUpdateTime));
    //compute pose once, reuse downstream
    const Pose2D robotPose = _deadReckoner.getPose();
    //full grid walk (recenter + update cells)
    _occupancyGrid.update(robotPose,tap->range);
    //update fire localisation if locked — DISPLACEMENT-GATED ingestion.
    // A rigorous lock (see updateTurret) gives high-quality bearings, but feeding one
    // EVERY tick from nearly the same spot floods the least-squares with redundant,
    // co-located observations that bias the fit. The fire localiser only needs a few
    // SPATIALLY DIVERSE bearings to triangulate, so only ingest once the robot has
    // moved >= FIRE_OBS_MIN_DISP_CM since the last ingested observation. Function-
    // local statics persist across ticks (world_model.hpp left untouched).
    if(tap->fire_locked){
        static bool  haveLastObs = false;
        static float lastObsX = 0.0f, lastObsY = 0.0f;
        const float dxo = robotPose.x - lastObsX;
        const float dyo = robotPose.y - lastObsY;
        const bool moved_enough = !haveLastObs ||
            (dxo*dxo + dyo*dyo) >= (FIRE_OBS_MIN_DISP_CM * FIRE_OBS_MIN_DISP_CM);
        if (moved_enough) {
            Pose2D turretPose = _robot->TurretPose; turretPose.th += tap->turret_angle;
            _fireLocaliser.update_localisation(robotPose,turretPose,tap->angle_error);
            lastObsX = robotPose.x; lastObsY = robotPose.y;
            haveLastObs = true;
        }
    }
    _lastUpdateTime = now;
}
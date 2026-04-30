# Behavior-control refactor — what's left

The FSM-to-behavior refactor (steps 1–6 of the plan at
`~/.claude/plans/the-current-codebase-is-crispy-meerkat.md`) is landed.
Both build targets compile clean (`pio run -e megaatmega2560` and
`pio run -e native`); the host smoke test passes. This file tracks the
work that still wants doing before the demonstration.

## Hardware wiring (placeholders to verify against the build)

These constants are in [`lib/controller/mappings.h`](lib/controller/mappings.h)
and were chosen to keep the build green; they probably need adjusting
once the boom and the photodiode array are physically wired.

- [ ] `turret_servo_pin = 11` — confirm the pin the small servo is on.
- [ ] `TURRET_SERVO_CHANNEL = 4` — only matters if any other servo
      grabbed channels 0–4 (drive uses 0–3).
- [ ] `turret_photo_left_pin = 55` (A1) and `turret_photo_right_pin = 56` (A2) —
      confirm against actual SFH 300 FA wiring.
- [ ] `TURRET_PHOTO_LEFT_RAD / RIGHT_RAD = ±30°` — measure the actual
      fan-out of the boom-mounted photodiodes once installed.
- [ ] `TURRET_PAN_MIN_RAD / MAX_RAD = ±90°` — check that a full ±90 °
      sweep doesn't crash the boom into the chassis or its own wiring.
- [ ] `PanServo` pulse range `(900, 2100) µs` — verify against the
      datasheet of the actual hobby servo from the kit.
- [ ] `PanServo` `mount_offset_rad = 0` — adjust if the servo is not
      installed pointing straight forward at its mechanical centre.
- [ ] `SFH300FA` pull-down resistor (default `10 kΩ` in the constructor)
      — verify the actual resistor on the boom; it is informational
      today (no calibration math depends on it) but ought to be
      accurate.

## Tuning constants — placeholders to bench-tune

All numeric defaults are bench guesses. They live in `mappings.h` for
the report's reproducibility section.

- [ ] `FINDFIRE_FORWARD_SPEED / NUDGE_*` — cruise speed and exploration
      cadence while the turret hunts. Watch for the body chasing its
      own nudge or stalling at walls.
- [ ] `MOVETOFIRE_KP_TURN`, `MOVETOFIRE_FORWARD_SPEED`, `MOVETOFIRE_PIVOT_RAD`
      — fall-back open-loop gain (used only when the IMU degrades) and
      the threshold beyond which we pivot in place instead of arcing
      around the candle.
- [ ] `EXTINGUISH_AIM_RAD` (turret bearing window for "body is aimed"),
      `EXTINGUISH_OUT_DEBOUNCE` (ticks below `FIRE_OUT_V` to declare
      done). Tune against the actual LED extinction profile.
- [ ] `EXTINGUISH_HARD_TIMEOUT_MS = 10000` matches the brief; do not
      change without re-reading the rubric.
- [ ] `AVOID_FIRE_MASK_RAD = 0.5 rad` (~28 °) — window around the turret
      bearing within which a forward-IR hit is treated as the candle.
      Too tight: avoid would jam into the candle. Too loose: avoid
      misses real obstacles next to a candle.
- [ ] `OBSTACLE_TRIGGER_CM / CLEAR_CM = 22 / 35` — already tuned on the
      legacy FSM; re-check now that the body-mounted phototransistors
      are gone (no obstruction from the old boom-less head).
- [ ] `FIRE_DETECT_V = 1.6 V`, `FIRE_OUT_V = 0.7 V` — calibrate against
      the actual SFH 300 FA + 10 kΩ pull-down + arena ambient light.
      The IR-filtered SFH 300 FA may need lower thresholds than the
      legacy visible-light phototransistors did.
- [ ] `TurretController::Config` defaults (`sweep_rate_rad_per_s = 1.6`,
      `lock_count_in = 5`, `lock_count_out = 8`, `track_kp = 0.6`) —
      tune sweep speed against servo response, debounce against the
      arena's noise floor, gain against the photo-array geometry.
- [ ] `HeadingPid` defaults (`Kp = 80`, `Ki = 0`, `Kd = 5`,
      `effort_limit = 200`) — tune on the arena once closed-loop is
      driving. Start by watching the response of `MoveToFire` to a
      candle placed 90 ° off body heading.

## Bench tests to run before the demo

- [ ] **IMU yaw drift**: stationary robot, watch `IMU::yaw()` over
      5 min via the BT log. Target: < 1 °/min. If the magnetometer-free
      game-rotation-vector still drifts more than that, consider
      switching `ArduinoIMUSource` to `SH2_ROTATION_VECTOR` (pulls in
      the magnetometer) and accept the magnetic-field noise tradeoff.
- [ ] **Turret sweep + lock**: hold an IR source (phone IR LED or the
      actual fire candle) in front of the boom; watch the snapshot
      transition IDLE → SWEEP → LOCKED. Verify the lock survives the
      `lock_count_in` debounce and falls back through `lock_count_out`
      cleanly when the source is removed.
- [ ] **World-bearing hold**: turret locked, manually rotate the body;
      pan command should compensate so the boom keeps aiming at the
      source. Verify the sign matches the IMU yaw convention (right-
      positive yaw should pan the turret left in the body frame).
- [ ] **Closed-loop heading**: drop the robot in front of a candle at
      ±90 ° from initial heading; `MoveToFire` should converge with no
      overshoot (Kp tuning) and no steady-state offset.
- [ ] **Avoid mask**: place the candle directly in front of an
      obstacle that is also directly in front of the robot;
      `AvoidObstacle` should abstain (the candle bearing aligns) and
      `MoveToFire` should drive in.
- [ ] **Extinguish + fire-out**: trigger the candle at close range,
      run the fan, watch the LED go out; `ExtinguishFire` should bump
      `fires_extinguished` exactly once and abstain so the arbiter
      can fall through.
- [ ] **Mission complete latch**: set up two candles, run the full
      mission; after the second extinguish the body should come to a
      hard stop and stay stopped (no arbiter fall-through to
      `FindFire`).

## Deferred from the plan / known gaps

- [ ] **`BumpEscape` behavior** (deferred from step 4). Would sit at
      priority 1 (above `AvoidObstacle`), use IMU linear acceleration
      to detect collisions, and back the body up. Not implemented —
      the IMU already exposes `ax/ay/az` so wiring it in is a
      one-evening job if a collision rate problem turns up on the
      arena.
- [ ] **Telemetry over BT during a run**. The CSV header in the .ino
      still reflects the legacy IR/ultrasonic format. A cleaner per-
      tick telemetry line would be:
      `time, winner_tag, body_yaw, turret_state, turret_world_bearing, turret_intensity, fires_extinguished`.
      Useful for the report's "evidence" section.
- [ ] **Battery monitor**. `FireFighter::is_battery_voltage_OK()` is
      still defined but never called by `tick()` — the legacy FSM used
      it during state transitions. The behavior arbiter should
      probably shut down to `MissionComplete` (or a dedicated
      `LowBattery` behavior) when the LiPo dips. Not urgent for the
      demo; nice for the report.
- [ ] **More than 2 photodiodes on the boom**. `PhotoArray` already
      supports up to 4 with the soft-centroid math; just instantiate
      and `add()` two more in the `FireFighter` constructor and add
      pins to `mappings.h`. Improves bearing accuracy when the candle
      is between two adjacent photodiodes.
- [ ] **Low-yaw-drift mode for static testing**. `IMU::tare_yaw()` is
      called once at startup. If the bench session is long, an
      operator-triggered re-tare (e.g. via a BT command) would be
      handy.

## Report-writing TODOs (Project 2 technical report)

- [ ] Behavior-control chapter (student 3) — owns the centrepiece of
      this refactor. Cite the priority ordering in
      [`behavior_arbiter.cpp`](lib/controller/behavior_arbiter.cpp)
      and the per-behavior tuning block at the bottom of `mappings.h`.
- [ ] Fire sensing/extinguishing chapter (student 1) — cite SFH 300 FA
      datasheet (730–1120 nm range, 880 nm peak), the `PhotoArray`
      soft-centroid formula, the `ExtinguishFireBehavior` debounce.
- [ ] Obstacle sensing chapter (student 2) — cite the unchanged
      ±30 ° front IR layout, the `AvoidObstacle` strafe-and-creep
      strategy, and the IMU-based fire mask.
- [ ] System integration chapter (student 4) — flow chart of
      `FireFighter::tick()`, pin allocation table, the HAL boundary
      diagram (`lib/hal/` interfaces vs `lib/hal_arduino/` /
      `lib/hal_sim/` impls), and a state diagram of the
      `TurretController`.
- [ ] Project management chapter — already structured in the brief.

## Build verification

Quick command to verify both targets compile cleanly after any change:

```bash
pio run -e native && pio run -e megaatmega2560 && .pio/build/native/program
```

Expected output: both targets `SUCCESS`, smoke test prints
`[host] motor channel 0 last us = <some non-1500 value>` and
`[host] smoke test ok`.

Current footprint on the Mega: **flash 14.9 % (37862 B), RAM 49.8 % (4079 B)**.
The deletion of the FSM offset most of the weight added by the
behavior layer + IMU + turret subsystem; the project has plenty of
headroom for the deferred items above.

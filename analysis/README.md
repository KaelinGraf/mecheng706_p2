# Occupancy map live visualiser (MATLAB)

A live MATLAB viewer for the Arduino-streamed occupancy map of the MechEng 706
firefighting robot. It parses the **occupancy serial protocol v1** and renders
the grid as a world-frame heatmap with robot, fire, and goal overlays.

The wire contract is specified in
[`../mecheng706_p2/RobotBaseCodes2026/mapping/OCCUPANCY_PROTOCOL.md`](../mecheng706_p2/RobotBaseCodes2026/mapping/OCCUPANCY_PROTOCOL.md)
and is implemented here byte-for-byte. Everything in this folder is host-side
MATLAB; it only *consumes* the protocol and never drives the robot.

## Files

| File | Purpose |
|------|---------|
| `parse_occupancy.m`        | Incremental, robust protocol parser. Returns a handle that consumes lines one at a time and yields complete frames. |
| `occupancy_viz.m`          | Live viewer with two modes: real serial port, or file replay. |
| `gen_synthetic_occupancy.m`| Writes a synthetic `.log` in the exact protocol for offline testing (no robot needed). |
| `test_occupancy.m`         | Self-verification: generates a log, drives the parser, asserts on the parsed structs. |

## Run modes

### 1. Live serial

```matlab
occupancy_viz('serial', 'COM5', 115200)
```

Opens the named port with `serialport()`, reads `\n`-terminated lines as they
arrive, parses them, and redraws on every complete frame. Use **>= 115200 baud**
(the firmware emits at 2-4 Hz). Runs until you close the figure window (or
Ctrl-C). On Linux/macOS the port is something like `'/dev/ttyACM0'`.

### 2. File replay

```matlab
occupancy_viz('file', 'capture.log')        % default ~4 fps
occupancy_viz('file', 'capture.log', 10)    % cap replay at ~10 fps
```

Replays a captured text log line-by-line through the **same** parser and
renderer, so a recorded run looks like a live one. Capture a real run with any
serial terminal (PuTTY, `screen`, Arduino IDE monitor) logging to a text file.

### 3. Generate a synthetic log (offline / no hardware)

```matlab
gen_synthetic_occupancy('capture.log')      % 8 frames (1 deliberately truncated)
gen_synthetic_occupancy('capture.log', 20)  % 20 frames
occupancy_viz('file', 'capture.log')
```

The synthetic scene (n = 30, 5 cm cells) shows a robot translating right, a wall
filling in (log-odds climbing toward +12), free space carving out (negatives),
and a fire fix appearing mid-sequence (confidence 0 -> ~1). It also interleaves
non-`occupancy` noise lines and one truncated frame (missing its `E` line) to
exercise the parser's robustness.

## What gets drawn

- **Occupancy heatmap** from the raw log-odds, on a diverging colormap
  (**blue = free/neg, white = unknown/0, red = occupied/pos**), colour limits
  clamped to the protocol's `[-12, +12]`, placed in **world coordinates (cm)**
  via the window origin and cell size.
- **Robot marker** at `(pose_x, pose_y)` with a **heading arrow** from `pose_th`.
- **Fire marker** at `(fire_x, fire_y)` **only when `fire_conf > 0`**; its size
  and opacity scale with confidence.
- **Goal cell** highlighted with a green outline when `goal >= 0`.
- `axis equal`, cm axis labels, and a title showing the frame `seq` and the
  measured fps. All graphics objects update **in place** (no figure churn).

## Parser API (`parse_occupancy.m`)

```matlab
p = parse_occupancy();
frame  = p.feed(line);        % [] until a frame completes, else a struct
frames = p.feedChunk(text);   % parse a multi-line blob -> struct array
p.reset();                    % drop any partial accumulation
```

A completed `frame` struct has fields:

| field     | meaning |
|-----------|---------|
| `seq`     | frame counter |
| `n`       | grid dimension (grid is `n x n`) |
| `cell_cm` | cell size in cm |
| `org`     | `[org_cx org_cy]` window origin in world **cells** |
| `pose`    | `[pose_x pose_y pose_th]` robot pose (cm, cm, radians) |
| `fire`    | `[fire_x fire_y fire_conf]` (cm, cm, `[0..1]`; conf `0` => no target) |
| `goal`    | `[goal_x goal_y]` goal **cell** index, or `[-1 -1]` when none |
| `grid`    | `n x n` double of raw log-odds; `grid(y+1, x+1)` is cell `(x, y)` |

Cell `(x, y)` world centre: `wx = (org_cx + x + 0.5) * cell_cm`,
`wy = (org_cy + y + 0.5) * cell_cm`.

### Robustness

The parser ignores any line whose first whitespace token is not exactly
`occupancy` (boot logs and library chatter share the UART). A frame is emitted
only on an `E` line whose `seq` matches the most recent `F` **and** after all `n`
rows have arrived. A new `F` discards any partial accumulation, so a dropped or
truncated frame never renders and the stream resynchronises on the next header.
Malformed lines (wrong row width, non-numeric fields, mismatched `seq`) abort
the in-progress frame rather than throwing, so a glitched UART byte cannot crash
the live viewer.

## Verifying (no robot required)

Requires MATLAB (developed on R2024b; uses only base graphics, no toolboxes).

```matlab
test_occupancy
```

This generates `capture.log`, drives it through `parse_occupancy`, and asserts:
every well-formed frame parses; grids are `30 x 30` double; log-odds stay in
`[-12, 12]`; interleaved noise lines are ignored; the truncated frame does not
complete; the fire fix appears mid-sequence; the cell->world mapping holds; the
protocol's n=3 example parses exactly; a new `F` mid-stream resyncs; and an `E`
with a mismatched `seq` is rejected. It prints `ALL TESTS PASSED` on success.

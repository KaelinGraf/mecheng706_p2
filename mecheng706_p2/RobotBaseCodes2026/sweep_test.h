#ifndef SWEEP_TEST_H
#define SWEEP_TEST_H

#include <Arduino.h>
#include "firefighter.h"
#include "servo_control.h"

// ---------------------------------------------------------------------------
// SweepTest - bench characterisation of the OUTSIDE phototransistor pair.
//
// Background: the turret used to carry four photocells splayed at 15 deg and
// the firefighting code triangulated across all four. The hardware has since
// changed - the inner pair (_l, _r) are unchanged, but the OUTER pair
// (_sl, _sr) are now mounted FLAT (facing straight forward) with higher-gain
// bias resistors so they respond out to a longer range. A teammate is
// reworking the triangulation to use the difference across that outer pair;
// this test exists purely to gather the raw data that informs that work.
//
// Method: the light source is placed directly ahead of the turret's CENTRE
// position and held still. The turret then sweeps across its full mechanical
// range one degree at a time. Because the source is fixed, the turret's
// angle-from-centre IS the light's incidence angle on the flat outer cells, so
// a single sweep traces each outer cell's angular response. Repeating the
// sweep at successive distances stacks those curves into a distance family for
// MATLAB.
//
// The distance schedule is AUTOMATIC and purely timed - no serial input is
// required (the Bluetooth link's 115200 SoftwareSerial RX is unreliable). The
// test walks a fixed list of distances; before each one it pauses with a
// printed countdown so the operator can physically move the lamp to that
// distance, then sweeps.
//
// Protocol (one distance per loop() call, blocking):
//   1. Print "position the light at D cm" and wait out a timed countdown.
//   2. Sweep the turret SWEEP_MIN -> SWEEP_MAX in 1 deg steps. At each step
//      settle the servo, converge the EWMA, and emit one CSV row of all four
//      cell voltages, tagged with the distance and turret angle.
//   3. Recentre and advance to the next distance; stop after the last one.
//
// Output is machine-parsable: every data row begins with the literal token
// "DATA," so the MATLAB importer can ignore banners / prompts in the PuTTY log.
//   DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v
//     sl,sr = outer FLAT pair (the cells under test)
//     l ,r  = inner angled pair (logged for context)
//     rel_deg = servo_deg - SERVO_CENTER  (0 = boresight, light dead ahead)
//
// The module only ever calls the existing public APIs of FireFighter / Turret
// / FireBank - it does not modify any other translation unit.
// ---------------------------------------------------------------------------
class SweepTest {
  public:
    // in0/in1 are the serial streams (USB Serial + Bluetooth SoftwareSerial).
    // They are no longer used to drive the test - kept only so any stray bytes
    // on a flaky link can be drained and ignored. Either may be nullptr.
    SweepTest(FireFighter* ff, Turret* turret, Stream* in0, Stream* in1 = nullptr)
      : _ff(ff), _turret(turret), _in0(in0), _in1(in1) {}

    // Print the banner + column legend and centre the turret. Call once in
    // setup() after the FireFighter and Turret have been constructed.
    void begin();

    // Advance the automatic schedule by one distance: pause for repositioning,
    // run a full sweep, recentre. Call from loop(); it self-terminates after
    // the last distance.
    void loop();

  private:
    FireFighter* _ff;
    Turret*      _turret;
    Stream*      _in0;        // retained only to drain/ignore stray input bytes
    Stream*      _in1;
    int          _dist_idx = 0;     // index into the automatic distance schedule
    bool         _finished = false; // set once the whole schedule has run

    // One full sweep across the range, emitting tagged CSV rows.
    void runSweep(float dist_cm);

    // Command the servo to angle_deg, then settle + converge the EWMAs so the
    // logged getFilteredV() reflects the steady-state reading at this angle.
    void settleAt(int angle_deg);

    // Timed pause (printed countdown) so the operator can move the lamp to
    // dist_cm before the next sweep. No serial input required.
    void pauseForRepositioning(float dist_cm);
};

#endif // SWEEP_TEST_H

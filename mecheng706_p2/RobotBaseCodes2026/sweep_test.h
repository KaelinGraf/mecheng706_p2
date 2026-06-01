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
// sweep at successive operator-entered distances stacks those curves into a
// distance family for MATLAB.
//
// Protocol (one full cycle per loop() call, blocking on operator input):
//   1. Prompt for the source distance (cm); read it from USB and/or Bluetooth.
//   2. Sweep the turret SWEEP_MIN -> SWEEP_MAX in 1 deg steps. At each step
//      settle the servo, converge the EWMA, and emit one CSV row of all four
//      cell voltages, tagged with the distance and turret angle.
//   3. Return to centre and prompt for the next distance.
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
    // in0/in1 are the serial streams the operator might type into (e.g. USB
    // Serial and the Bluetooth SoftwareSerial). Either may be nullptr; both are
    // polled so it doesn't matter which port PuTTY is attached to.
    SweepTest(FireFighter* ff, Turret* turret, Stream* in0, Stream* in1 = nullptr)
      : _ff(ff), _turret(turret), _in0(in0), _in1(in1) {}

    // Print the banner + column legend and centre the turret. Call once in
    // setup() after the FireFighter and Turret have been constructed.
    void begin();

    // Run exactly one prompt -> sweep -> recentre cycle. Blocks while waiting
    // for the operator's distance entry, so simply call it from loop().
    void loop();

  private:
    FireFighter* _ff;
    Turret*      _turret;
    Stream*      _in0;
    Stream*      _in1;

    // Blocking read of a numeric value terminated by Enter, from either input
    // stream. Re-reads on blank lines; returns the parsed distance in cm.
    float readDistanceBlocking();

    // Non-blocking: next pending byte from either input, or -1 if none.
    int readByteAny();

    // One full sweep across the range, emitting tagged CSV rows.
    void runSweep(float dist_cm);

    // Command the servo to angle_deg, then settle + converge the EWMAs so the
    // logged getFilteredV() reflects the steady-state reading at this angle.
    void settleAt(int angle_deg);
};

#endif // SWEEP_TEST_H

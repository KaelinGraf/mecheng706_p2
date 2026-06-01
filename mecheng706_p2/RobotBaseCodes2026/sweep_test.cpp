#include "sweep_test.h"
#include "mappings.h"

// ---------------------------------------------------------------------------
// Sweep geometry / timing. SWEEP_MIN/MAX match Turret::writeAngle's clip band
// (SERVO_CENTER +/- 60) so every commanded angle is reached without clipping.
// ---------------------------------------------------------------------------
namespace {
  const int SWEEP_MIN  = SERVO_CENTER - 60;  // 18 deg  (rel -60)
  const int SWEEP_MAX  = SERVO_CENTER + 60;  // 138 deg (rel +60)
  const int SWEEP_STEP = 1;                  // 1 deg resolution

  // Automatic distance schedule: DIST_START_CM up to DIST_END_CM in DIST_STEP_CM
  // steps (10, 30, ... 190 cm = 10 sweeps covering ~10 cm to 2 m). Tweak freely.
  const float DIST_START_CM = 10.0f;
  const float DIST_END_CM   = 200.0f;
  const float DIST_STEP_CM  = 20.0f;

  // Timed pause before each sweep so the operator can reposition the lamp. No
  // serial input is needed; bump this if you need more time to move the light.
  const unsigned long PAUSE_BETWEEN_MS  = 12000;  // total pause per distance
  const unsigned long COUNTDOWN_TICK_MS = 2000;   // countdown print interval

  const unsigned long SETTLE_BIG_MS  = 500;  // after a large move (centre <-> ends)
  const unsigned long SETTLE_STEP_MS = 60;   // after each 1 deg step
  const uint8_t       N_FLUSH        = 12;    // EWMA convergence reads per step
  const unsigned long FLUSH_GAP_MS   = 4;     // spacing between convergence reads
}

void SweepTest::begin() {
  _turret->attach();
  _turret->center();
  delay(SETTLE_BIG_MS);

  _ff->println();
  _ff->println("========================================");
  _ff->println(" OUTSIDE-PAIR PHOTOTRANSISTOR SWEEP TEST");
  _ff->println("        (automatic distance schedule)");
  _ff->println("========================================");
  _ff->println(" Place the light directly ahead of the");
  _ff->println(" turret CENTRE. The test sweeps the turret");
  _ff->println(" automatically at each distance below,");
  _ff->println(" pausing with a countdown between sweeps so");
  _ff->println(" you can move the lamp. NO serial input used.");
  _ff->println("----------------------------------------");
  _ff->print  (" Distances (cm):");
  for (float d = DIST_START_CM; d <= DIST_END_CM + 0.001f; d += DIST_STEP_CM) {
    _ff->print(" ");
    _ff->print((int)d);
  }
  _ff->println();
  _ff->println(" Rows: DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v");
  _ff->println("   sl,sr = outer FLAT pair (under test)");
  _ff->println("   l ,r  = inner angled pair (context)");
  _ff->println("   rel_deg = servo_deg - center (0 = boresight)");
  _ff->println("========================================");
}

void SweepTest::loop() {
  float dist_cm = DIST_START_CM + _dist_idx * DIST_STEP_CM;

  // Schedule exhausted: announce once, hold centred, then idle.
  if (dist_cm > DIST_END_CM + 0.001f) {
    if (!_finished) {
      _ff->println();
      _ff->println("# ============================================");
      _ff->println("# ALL SWEEPS COMPLETE - capture finished.");
      _ff->println("# Turret centred. You can stop PuTTY logging.");
      _ff->println("# ============================================");
      _turret->center();
      _finished = true;
    }
    delay(500);
    return;
  }

  pauseForRepositioning(dist_cm);
  runSweep(dist_cm);
  _dist_idx++;
}

void SweepTest::pauseForRepositioning(float dist_cm) {
  _ff->println();
  _ff->print("# >>> NEXT: position the light at ");
  _ff->print((int)dist_cm);
  _ff->println(" cm, directly ahead of turret centre.");

  // Purely timed countdown - no serial input needed. Any bytes that do arrive
  // on the (flaky) link are drained and ignored so they can't disturb timing.
  for (long remaining = (long)PAUSE_BETWEEN_MS; remaining > 0;
       remaining -= (long)COUNTDOWN_TICK_MS) {
    _ff->print("#   sweeping in ");
    _ff->print((int)((remaining + 999) / 1000));
    _ff->println(" s ...");

    unsigned long t0 = millis();
    while (millis() - t0 < COUNTDOWN_TICK_MS) {
      while (_in0 && _in0->available() > 0) _in0->read();   // drain + ignore
      while (_in1 && _in1->available() > 0) _in1->read();
      delay(10);
    }
  }
}

void SweepTest::settleAt(int angle_deg) {
  _turret->writeAngle(angle_deg);
  delay(SETTLE_STEP_MS);
  // Converge the EWMA so getFilteredV() below is the steady-state value at this
  // angle rather than a value still lagging from the previous step.
  for (uint8_t i = 0; i < N_FLUSH; i++) {
    _ff->_fire_bank->update();
    delay(FLUSH_GAP_MS);
  }
}

void SweepTest::runSweep(float dist_cm) {
  _ff->print("# SWEEP START dist=");
  _ff->print((int)dist_cm);
  _ff->println(" cm");
  _ff->println("# DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v");

  // Pre-position at the start of the sweep and let the large move settle fully
  // before the first sample.
  _turret->writeAngle(SWEEP_MIN);
  delay(SETTLE_BIG_MS);

  FireBank* fb = _ff->_fire_bank;

  for (int a = SWEEP_MIN; a <= SWEEP_MAX; a += SWEEP_STEP) {
    settleAt(a);

    float sl = fb->_sl->getFilteredV();
    float l  = fb->_l->getFilteredV();
    float r  = fb->_r->getFilteredV();
    float sr = fb->_sr->getFilteredV();

    _ff->print("DATA,");
    _ff->print((int)dist_cm);
    _ff->print(",");
    _ff->print(a);
    _ff->print(",");
    _ff->print(a - SERVO_CENTER);
    _ff->print(",");
    _ff->print(sl, 4);
    _ff->print(",");
    _ff->print(l, 4);
    _ff->print(",");
    _ff->print(r, 4);
    _ff->print(",");
    _ff->println(sr, 4);
  }

  _ff->print("# SWEEP END dist=");
  _ff->print((int)dist_cm);
  _ff->println(" cm");

  // Recentre so the next distance is measured from the same boresight.
  _turret->center();
  delay(SETTLE_BIG_MS);
}

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
  _ff->println("========================================");
  _ff->println(" Place the light source directly ahead of");
  _ff->println(" the turret CENTRE, then enter a distance.");
  _ff->println(" The turret sweeps its full range while all");
  _ff->println(" four cells are logged. When it recentres,");
  _ff->println(" move the light to a new distance and repeat.");
  _ff->println("----------------------------------------");
  _ff->println(" Rows: DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v");
  _ff->println("   sl,sr = outer FLAT pair (under test)");
  _ff->println("   l ,r  = inner angled pair (context)");
  _ff->println("   rel_deg = servo_deg - center (0 = boresight)");
  _ff->println("========================================");
}

void SweepTest::loop() {
  float dist_cm = readDistanceBlocking();
  runSweep(dist_cm);
}

int SweepTest::readByteAny() {
  if (_in0 && _in0->available() > 0) return _in0->read();
  if (_in1 && _in1->available() > 0) return _in1->read();
  return -1;
}

float SweepTest::readDistanceBlocking() {
  _ff->println();
  _ff->println("Enter distance in cm, then press Enter (e.g. 40):");

  char buf[16];
  uint8_t idx = 0;
  for (;;) {
    int c = readByteAny();
    if (c < 0) {
      delay(2);
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (idx == 0) continue;          // ignore blank lines / CRLF artefacts
      buf[idx] = '\0';
      return atof(buf);
    }
    // Accept digits and a single decimal point; silently drop anything else.
    if (idx < sizeof(buf) - 1 && ((c >= '0' && c <= '9') || c == '.')) {
      buf[idx++] = (char)c;
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
  _ff->print(dist_cm);
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
    _ff->print(dist_cm);
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
  _ff->print(dist_cm);
  _ff->println(" cm");

  // Recentre so the next distance is measured from the same boresight.
  _turret->center();
  delay(SETTLE_BIG_MS);
}

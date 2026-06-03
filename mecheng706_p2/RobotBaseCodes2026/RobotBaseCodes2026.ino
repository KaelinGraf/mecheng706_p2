/*
  MechEng 706 Project 2 - Behavior-Based Control Robotics (Fire Fighting)

  Re-uses the Project 1 mecanum chassis, gyroscope, IR/ultrasonic sensors and
  servo/motor framework. Project-1-specific tilling/snake/homing logic has
  been stripped; the system now uses behavior-based control with two main
  states: TRACKING (which coordinates search, avoid, and approach behaviors)
  and EXTINGUISHING (which coordinates the fan control behavior).

  Hardware (carried over from Project 1):
    Arduino Mega2560, BNO085 IMU, HC-SR04, 2x long-range IR, 2x short-range IR,
    Servo, mecanum motor drivers, HC-12 wireless module, LiPo battery.

  Project 2 additions (not yet wired here):
    Phototransistors for fire detection, fan + MOSFET gate, 5V regulator.
*/

#include <Adafruit_BNO08x.h> // Gyroscope
#include "mappings.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include "pid.h"
#include "servo_control.h"
#include "firefighter.h"
#include "sweep_test.h"

// Bluetooth Setup matching WirelessSetup2026.ino
#define BLUETOOTH_RX 19
#define BLUETOOTH_TX 18

// ===========================================================================
// SWEEP_TEST: bench characterisation of the outer (flat) phototransistor pair.
// When defined, setup()/loop() run the sensor-sweep test (sweep_test.*) INSTEAD
// of the firefighting FSM. Left OFF (commented) for normal robot operation;
// uncomment to re-run a bench sweep.
// ===========================================================================
#define TEST_FIRE_BANK false
#define SWEEP_TEST  false
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Gyroscope initialisation
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float rad = 0.0;
float targetBearing;
float angleError;
float angleControl;
int count;

// #define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about battery damage.

int speed_val = 100;
int speed_change;

void delaySeconds(int TimedDelaySeconds);
void flashLED(int LedNumber, int TimedDelay);
void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3);
void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3);
void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2,
                                  int32_t Value3);
void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3);
void setupWireless();
void readTurretSerial();
float turretAngleToBearing(int angle);

int pos = 0;
FireFighter *firefighter = nullptr;
Turret *turret = nullptr;
SweepTest *sweepTest = nullptr;
int noFireDetectCount = 0;
long lastSensPrint;
long lastSensTurret;

void setup(void)
{
  // Configure INT4 (digital pin 2 ultrasonic echo) for any-edge interrupts
  EICRB |= (1 << ISC40);
  EICRB &= ~(1 << ISC41);
  // Enable INT4 after the Ultrasonic instance is created in FireFighter
  // (see below) to avoid the ISR firing before `ultrasonicISR` is set.

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug
  // info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  BluetoothSerial.begin(115200);
  Serial.println("0");

  firefighter = new FireFighter(&bno08x, &sensorValue, &Serial);
  Serial.println("1");
  firefighter->setBluetoothSerial(&BluetoothSerial); // Enable dual-printing to Bluetooth
  Serial.println("2");
  //firefighter->println("Serial");

  // Now that the FireFighter (and its Ultrasonic instance) exist, enable
  // the external interrupt which the Ultrasonic ISR expects.
  EIMSK |= (1 << INT4);
  Serial.println("3");

  // Turret is independent of FireFighter — create and initialise here.
  turret = new Turret(firefighter->_fire_bank, turret_pin);

  turret->attach();
  turret->center();
  //firefighter->println("Turret middle");
  Serial.println("4");

  delay(2000); // settling time but not really needed
  // Brief rotational nudge to confirm motors are alive, then zero the gyro
  firefighter->_gyro->resetAngle();
  lastSensPrint = millis();

#if SWEEP_TEST
  // Bench test mode: drive the outer-pair sweep instead of the FSM. Reads
  // operator input from either USB Serial or the Bluetooth link.
  sweepTest = new SweepTest(firefighter, turret, &Serial, &BluetoothSerial);
  sweepTest->begin();
#endif

  /*
  // CSV header for sensor logging
  firefighter->print("time");
  firefighter->print(",");
  firefighter->print("f_ir1");
  firefighter->print(",");
  firefighter->print("f_ir2");
  firefighter->print(",");
  firefighter->print("b_ir1");
  firefighter->print(",");
  firefighter->print("b_ir2");
  firefighter->print(",");
  firefighter->print("y_ult");
  firefighter->println(";");
  */
}

void loop(void) // main loop
{
#if SWEEP_TEST
  // Outer-pair sensor sweep test owns the loop while enabled; everything below
  // (the firefighting FSM) is skipped.
  sweepTest->loop();
  return;
#elif TEST_FIRE_BANK
  firefighter->_fire_bank->update();
  firefighter->updateIrSensors();

  if ((millis() - lastSensTurret > 50)) {

    updateTurret();
    lastSensTurret = millis();
  }

  if (millis() - lastSensPrint > 500) {
    firefighter->testSensors();
    printFireSensors();  
    lastSensPrint = millis();
  }
#else
  firefighter->pollState();
  if (firefighter->getCurrentState() == State::STOPPED) return;
  
  firefighter->setBearing(turret->angle_);
  if ((millis() - lastSensTurret > 30)) {

    updateTurret();
    lastSensTurret = millis();
  }
  if (millis() - lastSensPrint > 1000) {
    // Skip the blocking ultrasonic/IR dump during the spin-scan so the spin
    // stays smooth (SpinScan needs a fast, uninterrupted loop).
    if (firefighter->getCurrentState() != State::SPIN_SCAN) {
      firefighter->testSensors();
    }
    printFireSensors();  
    lastSensPrint = millis();
  }
#endif
}

void printFireBank() {
    FireBank *fb = firefighter->_fire_bank;
    firefighter->print("est: ");
    firefighter->print(fb->isValid() ? angleError : -1.0);

    firefighter->print("\t| turret: ");
    firefighter->print(turret->angle_);

    firefighter->print("\t| Commanded:");
    firefighter->print(angleControl);


    firefighter->print("\t| SL: ");
    firefighter->print(fb->_sl->getFilteredV());
    
    firefighter->print("\t| L: ");
    firefighter->print(fb->_l->getFilteredV());
    
    firefighter->print("\t| R: ");
    firefighter->print(fb->_r->getFilteredV());
    
    firefighter->print("\t| SR: ");
    firefighter->println(fb->_sr->getFilteredV());
}



void updateTurret() {
  // Behaviour 2: while the chassis is doing its 360 spin-scan, SpinScan holds
  // the turret dead-centre and owns the fire bank. Don't pan/aim/lock here.
  if (firefighter->getCurrentState() == State::SPIN_SCAN) return;

  // Fire bank is refreshed every loop in FireFighter::pollState().
  angleError = firefighter->_fire_bank->estimateBearing(); // degrees off-axis; 0 = aimed

  static unsigned long last_error_print_ms = 0;
  static unsigned long last_scan_print_ms = 0;
  unsigned long now = millis();
  if (now - last_error_print_ms >= 250UL) {
    Serial.print("Error ");
    Serial.println(angleError);
    last_error_print_ms = now;
  }
  // Serial.print("Locked ");  
  // Serial.println(turret->locked_on_);
  // Serial.print("angle ");  
  // Serial.println(turret->angle_);
  // Serial.print("Readings ");  

  // Behaviour 1: lock on only when BOTH outer (long-range) cells exceed the
  // gate. Hysteresis: once locked, hold until both fall below the lower unlock
  // threshold for LOCK_LOSS_DEBOUNCE consecutive updates.
  float lock_v = turret->locked_on_ ? FIRE_UNLOCK_OUTER_V : FIRE_LOCK_OUTER_V;
  if (!firefighter->_fire_bank->isValid()) {
    noFireDetectCount++;
    if (noFireDetectCount > LOCK_LOSS_DEBOUNCE) {
      turret->lockOn(false);
    }
  } else {
    noFireDetectCount = 0;
    turret->lockOn(true);
  }

  if (!turret->locked_on_) {
    if (now - last_scan_print_ms >= 500UL) {
      firefighter->println("Scan");
      last_scan_print_ms = now;
    }
    turret->pan_scan(now);
    return;
  }

  float old_angle = turret->angle_;
  if (fabs(angleError)<4.0){
    Serial.println("Aimed");  
    return;
  }
  float MAX_SLEW = 10.0;
  //if(fabs(angleError - old_angle)> )
  angleControl = turret->angle_ + 0.6 * angleError;
  Serial.print("Control ");  
  Serial.println(angleControl);

  if(firefighter->_fire_bank->isValid()) turret->writeAngle(angleControl);
}

void printFireSensors() {
    firefighter->print("FireBank: ");
    firefighter->print(firefighter->_fire_bank->_sl->getFilteredV());
    firefighter->print(" ");
    firefighter->print(firefighter->_fire_bank->_l->getFilteredV());
    firefighter->print(" ");
    firefighter->print(firefighter->_fire_bank->_r->getFilteredV());
    firefighter->print(" ");
    firefighter->print(firefighter->_fire_bank->_sr->getFilteredV());
    firefighter->println(" ");
}

void printBluetooth()
{
  serialOutput(analogRead(A4), analogRead(A4), analogRead(A4));
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

float turretAngleToBearing(int angle)
{
  const float degreesToRadians = 0.017453292519943295f;
  return (static_cast<float>(angle) - 90.0f) * degreesToRadians;
}

void readTurretSerial()
{
  static char buffer[8];
  static uint8_t bufferIndex = 0;
  static unsigned long lastInputMillis = 0;

  auto finalizeCommand = [&]() {
    if (bufferIndex == 0)
    {
      return;
    }

    buffer[bufferIndex] = '\0';
    Serial.print("Turret input: ");
    Serial.println(buffer);

    int angle = atoi(buffer);
    turret->writeAngle(angle);
    Serial.print("Turret angle set to: ");
    Serial.println(turret->angle_);
    bufferIndex = 0;
  };

  while (Serial.available() > 0)
  {
    char incoming = static_cast<char>(Serial.read());
    lastInputMillis = millis();

    if (incoming == '\r' || incoming == '\n')
    {
      finalizeCommand();
    }
    else if ((incoming >= '0' && incoming <= '9') || (incoming == '-' && bufferIndex == 0))
    {
      if (bufferIndex < sizeof(buffer) - 1)
      {
        buffer[bufferIndex++] = incoming;
        if (bufferIndex == 3)
        {
          finalizeCommand();
        }
      }
    }
    else
    {
      Serial.print("Ignored turret input char: ");
      Serial.println(incoming);
      bufferIndex = 0;
    }
  }

  if (bufferIndex > 0 && (millis() - lastInputMillis) > 100)
  {
    finalizeCommand();
  }
}

#ifndef NO_BATTERY_V_OK

#endif

void Analog_Range_A4()
{
  firefighter->print("Analog Range A4:");
  firefighter->println(analogRead(A4));
}


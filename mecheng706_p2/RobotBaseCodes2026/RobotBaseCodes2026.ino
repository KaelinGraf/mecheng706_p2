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

// Bluetooth Setup matching WirelessSetup2026.ino
#define BLUETOOTH_RX 19
#define BLUETOOTH_TX 18
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Gyroscope initialisation
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float rad = 0.0;

// #define NO_READ_GYRO  //Uncomment if GYRO is not attached.

#define NO_HC \
  -SR04 // Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.

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
long lastSensPrint;

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

  firefighter = new FireFighter(&bno08x, &sensorValue, &Serial);
  firefighter->setBluetoothSerial(&BluetoothSerial); // Enable dual-printing to Bluetooth
  firefighter->println("Serial");

  // Now that the FireFighter (and its Ultrasonic instance) exist, enable
  // the external interrupt which the Ultrasonic ISR expects.
  EIMSK |= (1 << INT4);
  firefighter->println("Interupt");

  // Turret is independent of FireFighter — create and initialise here.
  turret = new Turret(turret_pin);

  turret->attach();
  turret->center();
  turret->writeAngle(140);
  float targetBearing = 90;
  firefighter->println("Turret middle");

  delay(100); // settling time but not really needed
  // Brief rotational nudge to confirm motors are alive, then zero the gyro
  firefighter->_gyro->resetAngle();
  lastSensPrint = millis();

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
  //firefighter->pollState();
  //targetBearing = turretAngleToBearing(turret->angle_);
  //firefighter->setBearing(targetBearing);
  //Serial.print("Target bearing rad: ");
  //Serial.println(targetBearing, 4);
  if (millis() - lastSensPrint > 1000)
  {
    firefighter->testSensors();
    lastSensPrint = millis();
  }
  delay(1000);
  
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

#ifndef NO_HC - SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      firefighter->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      firefighter->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST)
  {
    firefighter->println("HC-SR04: Out of range");
  }
  else
  {
    firefighter->print("HC-SR04:");
    firefighter->print(cm);
    firefighter->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  firefighter->print("Analog Range A4:");
  firefighter->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  if (bno08x.wasReset())
  {
    bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  }

  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED)
    {
      float gyroZ = sensorValue.un.gyroscope.z; // Current Measured Angular Velocity Around The Z Axis
      firefighter->print("Gyroscope I2C: ");
      firefighter->println(gyroZ);
    }
  }
  return;
}
#endif

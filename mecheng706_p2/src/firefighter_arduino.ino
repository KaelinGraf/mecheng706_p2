/*
  MechEng 706 Project 2 - Behavior Control Robotics (Fire Fighting)

  Arduino entry point. This file is intentionally thin: it constructs
  the concrete Arduino HAL backends (clock, ADC, gyro source, ultrasonic
  source, motor output, fan output, serial printers), bundles them into a
  HardwareContext, and hands the context to the FireFighter. The FSM and
  every sensor/motor abstraction inside the controller are HAL-clean and
  identical to the host build.

  Hardware:
    Arduino Mega 2560, BNO085 IMU, HC-SR04, 2x long-range IR, 2x short-range
    IR, mecanum motor drivers, HC-12 wireless module, LiPo battery, fan.
*/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include "mappings.h"
#include "hardware_context.h"

#include "arduino_clock.h"
#include "arduino_analog_input.h"
#include "arduino_imu_source.h"
#include "arduino_ultrasonic_source.h"
#include "arduino_motor_output.h"
#include "arduino_fan_output.h"
#include "arduino_printer.h"

#include "firefighter.h"

// ---------------------------------------------------------------------------
// Bluetooth pin mapping (HC-12 wireless module on the Mega).
// ---------------------------------------------------------------------------
#define BLUETOOTH_RX 19
#define BLUETOOTH_TX 18
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// ---------------------------------------------------------------------------
// BNO085 driver instance + the buffer it fills with sensor events.
// Owned at file scope so the ArduinoIMUSource can keep references to them.
// ---------------------------------------------------------------------------
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// ---------------------------------------------------------------------------
// HAL backend instances. Each implements one IXxx interface.
// ---------------------------------------------------------------------------
ArduinoClock              hal_clock;
ArduinoAnalogInput        hal_adc;
ArduinoIMUSource          hal_imu(&bno08x, &sensorValue, &hal_clock);
ArduinoUltrasonicSource   hal_us(TRIG_PIN, ECHO_PIN);
ArduinoMotorOutput        hal_motors;
ArduinoFanOutput          hal_fan(fan_pin);
ArduinoPrinter            printer_usb(&Serial);
ArduinoPrinter            printer_bt(&BluetoothSerial);

FireFighter*              firefighter = nullptr;
unsigned long             lastSensPrint = 0;

void setup(void)
{
    Serial.begin(115200);
    BluetoothSerial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    // Bring up the BNO before we hand its source to the FireFighter; the
    // controller's Gyroscope assumes the source is producing samples.
    hal_imu.begin();

    // Configure the ultrasonic ISR + trigger pin.
    hal_us.begin();

    // Build the HAL bundle and construct the controller.
    HardwareContext hw;
    hw.clock              = &hal_clock;
    hw.analog_input       = &hal_adc;
    hw.gyro_source        = &hal_imu;
    hw.imu_source         = &hal_imu;
    hw.ultrasonic_source  = &hal_us;
    hw.motor_output       = &hal_motors;
    hw.fan_output         = &hal_fan;
    hw.printer_primary    = &printer_usb;
    hw.printer_secondary  = &printer_bt;

    static FireFighter ff(hw);
    firefighter = &ff;

    delay(100);

    // Brief rotational nudge to confirm motors are alive, then zero the
    // gyro and tare the IMU so the starting pose is "yaw = 0".
    firefighter->_motors->writeAllMotors(0.0f, 0.0f, -100.0f);
    delay(450);
    firefighter->_motors->writeAllMotors(0.0f, 0.0f, 0.0f);
    delay(100);
    firefighter->_gyro->resetAngle();
    firefighter->_imu->update();
    firefighter->_imu->tare_yaw();
    lastSensPrint = millis();

    // CSV header for sensor logging.
    firefighter->print("time"); firefighter->print(",");
    firefighter->print("f_ir1"); firefighter->print(",");
    firefighter->print("f_ir2"); firefighter->print(",");
    firefighter->print("b_ir1"); firefighter->print(",");
    firefighter->print("b_ir2"); firefighter->print(",");
    firefighter->print("y_ult"); firefighter->println(";");
}

void loop(void)
{
    if (!firefighter) return;
    firefighter->pollState();
    if (millis() - lastSensPrint > 100) {
        firefighter->timeStepData();
        lastSensPrint = millis();
    }
}

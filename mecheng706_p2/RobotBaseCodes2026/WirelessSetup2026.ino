// Author: Marshall Lim

#include <SoftwareSerial.h>
#include "initialising.h"

#define INTERNAL_LED 13

// Serial Data input pin
#define BLUETOOTH_RX 19
// Serial Data output pin
#define BLUETOOTH_TX 18

#define STARTUP_DELAY 1   // Seconds
#define LOOP_DELAY 10     // miliseconds
#define SAMPLE_DELAY 10   // miliseconds

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

volatile int32_t Counter = 1;

void delaySeconds(int TimedDelaySeconds) {
  for (int i = 0; i < TimedDelaySeconds; i++) {
    delay(1000);
  }
}

void flashLED(int LedNumber, int TimedDelay) {
  digitalWrite(LedNumber, HIGH);
  delaySeconds(TimedDelay);
  digitalWrite(LedNumber, LOW);
  delaySeconds(TimedDelay);
}

void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2,
                                  int32_t Value3) {
  String Delimiter = ", ";

  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3) {
  if (OUTPUTMONITOR) {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER) {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR) {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);
    ;
  }
}

void readBluetoothSerial() {
  if (BluetoothSerial.available()) {
    String command = BluetoothSerial.readStringUntil('\n');
    Serial.print("Received via Bluetooth: ");
    Serial.println(command);
  }
}

void setupWireless() {
  BluetoothSerial.begin(115200);

  Serial.print("Ready, waiting for ");
  Serial.print(STARTUP_DELAY, DEC);
  Serial.println(" seconds");

  delaySeconds(STARTUP_DELAY);
}

void Initialising::begin() {
  delay(1000);  // One second delay to see the serial string "INITIALISING...."

  // ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  pinMode(INTERNAL_LED, OUTPUT);

  // IR sensors
  pinMode(front_right_ir_pin, INPUT);
  pinMode(front_left_ir_pin, INPUT);
  pinMode(rear_right_ir_pin, INPUT);
  pinMode(rear_left_ir_pin, INPUT);

  // TODO Project 2: pinMode(...) for phototransistors and fan_pin (OUTPUT)
}

void Initialising::end() {
}

void Initialising::poll() {
  // Hand off to the SEARCH (wander) state once initialisation completes.
  firefighter_->switchState(State::SEARCH);
}

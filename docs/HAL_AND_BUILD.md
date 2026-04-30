# Build & HAL guide

This project used to be an Arduino IDE sketch. It's now a [PlatformIO](https://platformio.org/) project with a hardware-abstraction layer (HAL) so the same controller code can compile two ways:

* For the **Arduino Mega 2560** (the actual robot), against real `analogRead`, the BNO08x driver, the HC-SR04 ISR, etc.
* For a **host build** (`g++` on your laptop), against simulated/pushed values. Used to exercise the controller against a Python sim and as a fast compile-time sanity check that nothing has accidentally pulled in `<Arduino.h>`.

The point of the HAL is to keep all the `analogRead` / `millis()` / `Serial.print` calls in one small set of files so the FSM and sensor algorithm code can compile in either environment unchanged.

If you only want to flash the robot, you can mostly ignore the HAL and just run `pio run -e megaatmega2560 -t upload`.

---

## 1. One-time setup

You need PlatformIO installed once:

```bash
pip install --user platformio        # or: pipx install platformio
```

Verify:

```bash
pio --version
# PlatformIO Core, version 6.x.x
```

VS Code users: install the **PlatformIO IDE** extension. It picks the project up automatically when you open `mecheng706_p2/` (the inner directory containing `platformio.ini`).

The Arduino IDE will *not* work on this layout any more.

---

## 2. Building & flashing the robot

From the inner `mecheng706_p2/` directory (the one with `platformio.ini`):

```bash
# Compile only
pio run -e megaatmega2560

# Compile and upload over USB
pio run -e megaatmega2560 -t upload

# Open the serial monitor at 115200 baud (USB or HC-12 wireless)
pio device monitor -b 115200
```

PlatformIO automatically downloads the AVR toolchain, the Adafruit BNO08x library, the Servo library and SoftwareSerial on first run.

If you want, you can use the IDE buttons too:

* VS Code PlatformIO: `Build` / `Upload` / `Monitor` icons in the bottom status bar.
* CLI is the source of truth — if a build fails on someone's machine but works on the CLI, the IDE has a stale environment.

---

## 3. Building the host (sim) target

```bash
pio run -e native
.pio/build/native/program
```

This compiles the controller against the **Sim HAL** (the implementations under `lib/hal_sim/`). The smoke-test `main()` lives in [`src/firefighter_host.cpp`](../mecheng706_p2/src/firefighter_host.cpp) — it pushes plausible voltages into the Sim ADC, steps the FSM 50 times, and prints the final state. Useful as a fast "does the controller still compile cleanly without Arduino?" check.

If `pio run -e native` fails but `pio run -e megaatmega2560` succeeds, you almost certainly accidentally `#include <Arduino.h>` (or a header that does) inside `lib/controller/` or `lib/hal/`. See section 5.

---

## 4. Project layout

```
mecheng706_p2/
├── platformio.ini              # two envs: megaatmega2560, native
├── src/
│   ├── firefighter_arduino.ino # Arduino entry (built only for the robot)
│   └── firefighter_host.cpp    # host smoke-test (built only for native)
├── lib/
│   ├── controller/             # FSM + sensors + motors + fire + utils
│   │                           # *** This is where you write features. ***
│   ├── hal/                    # HAL interfaces (header-only, abstract)
│   ├── hal_arduino/            # Arduino-side HAL impls (real hardware)
│   └── hal_sim/                # Sim-side HAL impls (push-from-tests)
└── docs/
    └── HAL_AND_BUILD.md        # (this file)
```

`lib_ignore` in `platformio.ini` makes `hal_arduino/` invisible to the native build and `hal_sim/` invisible to the Arduino build, so neither side accidentally pulls in the other's dependencies.

---

## 5. The HAL contract

The HAL is **a small set of pure abstract C++ interfaces** that sit between the controller and any hardware-interactive call. Everything in `lib/controller/` either:

1. Doesn't touch hardware at all (FSM logic, math, ring buffers), or
2. Calls a HAL interface (`IClock`, `IAnalogInput`, `IGyroSource`, `IUltrasonicSource`, `IMotorOutput`, `IFanOutput`, `IPrinter`).

**Nothing under `lib/controller/` is allowed to `#include <Arduino.h>`** (or any Arduino-only header — `Servo.h`, `Adafruit_BNO08x.h`, `HardwareSerial.h`, `SoftwareSerial.h`, `Print.h`, …). That's the rule. The native build will fail loudly if you break it, which is the point.

| Interface (`lib/hal/<name>.h`) | Replaces | Arduino backend (`lib/hal_arduino/`) | Sim backend (`lib/hal_sim/`) |
| --- | --- | --- | --- |
| `IClock`              | `millis()`, `micros()`             | `arduino_clock.cpp`        | `sim_clock.cpp`        |
| `IAnalogInput`        | `analogRead()` + ADC scaling       | `arduino_analog_input.cpp` | `sim_analog_input.cpp` |
| `IGyroSource`         | `Adafruit_BNO08x` events           | `arduino_gyro_source.cpp`  | `sim_gyro_source.cpp`  |
| `IUltrasonicSource`   | INT4 ISR + trigger pulse on HC-SR04 | `arduino_ultrasonic_source.cpp` | `sim_ultrasonic_source.cpp` |
| `IMotorOutput`        | `Servo::writeMicroseconds()`       | `arduino_motor_output.cpp` | `sim_motor_output.cpp` |
| `IFanOutput`          | MOSFET-gate digital/PWM with soft-start | `arduino_fan_output.cpp` | `sim_fan_output.cpp`   |
| `IPrinter`            | `Serial.print` / `BluetoothSerial.print` | `arduino_printer.h` (inline) | `sim_printer.cpp`      |

The `HardwareContext` struct in [`lib/hal/hardware_context.h`](../mecheng706_p2/lib/hal/hardware_context.h) bundles all eight pointers; `FireFighter` takes one in its constructor and stores it. The state classes get to it via `firefighter_->clock()` / `firefighter_->hardware()`.

---

## 6. How to add a feature

Most additions don't need any HAL changes. The decision tree:

### 6.1 You're adding behaviour to an existing state

Just edit the state's `.cpp` (e.g. `lib/controller/search.cpp`). The state already has access to every sensor and the motors via `firefighter_->_front_left_ir->getAvg()`, `firefighter_->_motors->writeAllMotors(...)`, etc. **No HAL work needed.**

If you need timing, use:

```cpp
unsigned long now = firefighter_->clock()->now_ms();
```

…not `millis()`. (`millis()` won't compile here — `<Arduino.h>` isn't included.)

### 6.2 You're adding a new state

1. Add an entry to the `State::Name` enum in [`lib/controller/state.h`](../mecheng706_p2/lib/controller/state.h).
2. Create `my_state.h` and `my_state.cpp` in `lib/controller/`, mirroring the shape of e.g. `search.h` / `search.cpp` (subclass `State`, override `begin()`, `end()`, `poll()`).
3. Allocate the state in the array inside `FireFighter::FireFighter` in `firefighter.cpp`.

Same rule: read sensors via the existing handles, command motors via `_motors`, time via `firefighter_->clock()`. **No HAL work needed.**

### 6.3 You're adding a new sensor that uses an existing HAL primitive

E.g. another analog sensor, or another phototransistor variant. It uses `IAnalogInput::read_voltage(pin)` — which already exists. Steps:

1. Write your sensor class in `lib/controller/`. Take an `IAnalogInput*` plus a pin number in the constructor (look at how `Phototransistor` does it).
2. Add an instance in `FireFighter::FireFighter`, passing it `_hw.analog_input` and the pin number from `mappings.h`.
3. Add the pin number to `mappings.h` (use the **raw Mega pin number**, e.g. `A6` is `60`. Don't write `A6` — the controller can't see Arduino's macros).

**No HAL work needed.** The Sim HAL `set_voltage(pin, v)` will work for your sensor automatically.

### 6.4 You're adding a *kind* of hardware that isn't covered by an existing HAL primitive

E.g. an I2C device that's not the BNO08x, a different ranging sensor with its own protocol, an SD card. This is the only case where you actually have to extend the HAL. Steps:

1. **Define the interface.** Create `lib/hal/imy_thing.h` with a pure abstract class:

   ```cpp
   class IMyThing {
   public:
       virtual ~IMyThing() = default;
       virtual float read_value() = 0;
   };
   ```

   Keep it small — one method per coherent operation. Use plain C/POD types (`float`, `uint8_t`, `bool`, structs of those). No STL, no Arduino types, no `String`.

2. **Add a member to `HardwareContext`** in `lib/hal/hardware_context.h`:

   ```cpp
   IMyThing* my_thing = nullptr;
   ```

3. **Implement the Arduino backend** in `lib/hal_arduino/arduino_my_thing.{h,cpp}`. This is where `<Arduino.h>` and any vendor library go. Inherit `IMyThing` and implement the methods.

4. **Implement the Sim backend** in `lib/hal_sim/sim_my_thing.{h,cpp}`. Should be trivial — usually just a value the test/sim pushes in via a setter.

5. **Wire the Arduino backend** in `src/firefighter_arduino.ino`: declare an `ArduinoMyThing hal_my_thing(...);` at file scope, set `hw.my_thing = &hal_my_thing;` in `setup()`.

6. **Wire the Sim backend** in `src/firefighter_host.cpp`: same idea but with `SimMyThing`.

7. **Use it from the controller.** Wherever you need it (a sensor wrapper, a state) read it via `_hw.my_thing->read_value()`.

If you find yourself wanting to call a HAL method *directly* from a state, prefer wrapping it in a controller-level class instead. The state code stays clean of HAL concerns; the wrapper class deals with the HAL.

---

## 7. Common gotchas

* **`millis()` / `micros()` won't compile in the controller.** Use `firefighter_->clock()->now_ms()` / `now_us()`. The IDE may not catch this until you run `pio run -e native`; the Arduino target *will* compile because `<Arduino.h>` is in scope through transitive includes.
* **`Serial.print(...)` won't either.** Use `firefighter_->print(...)` / `firefighter_->println(...)`. They fan out to both USB and Bluetooth automatically.
* **Pin macros (`A0`..`A15`).** The controller can't see them. Use raw pin numbers (Mega `A0 == 54`) in `mappings.h`. You can write the macro name in a comment for cross-reference.
* **Adding a new motor channel.** Bump `MAX_CHANNELS` in both `arduino_motor_output.h` and `sim_motor_output.h`. Channel index is just a uint8_t — it's a logical index, not a pin.
* **Native build fails with "undefined reference to `__cxa_pure_virtual`" or similar.** Almost always means a `lib/controller/` file `#includes` an Arduino header (often via `<Arduino.h>` -> `Print.h` -> `String.h`). Grep for `Arduino.h`; if you find it under `lib/controller/`, take it out.

---

## 8. Why this matters

The HAL refactor cost ~one work-day. In return:

* The controller code is portable — nothing prevents you running it against a different microcontroller later, or against a sim, or in a unit test.
* Bugs in the FSM can be reproduced and stepped through on your laptop without flashing.
* The Arduino-only files (ISR, `Servo`, `Adafruit_BNO08x` init, etc.) are isolated. If a vendor library breaks an upgrade, you fix it in one file, not everywhere.

It's the same architecture you'd use for any embedded system you intend to maintain for more than a couple of weeks.

#ifndef HAL_ARDUINO_IMU_SOURCE_H
#define HAL_ARDUINO_IMU_SOURCE_H

#include <Adafruit_BNO08x.h>
#include "iimu_source.h"
#include "igyro_source.h"
#include "iclock.h"

// Wraps Adafruit's BNO08x driver and exposes a multi-output HAL source.
//
// Implements both IIMUSource (yaw + omega + accel, used by the new IMU
// sensor wrapper) and IGyroSource (omega-only, used by the legacy
// Gyroscope sensor wrapper) so a single concrete instance can serve both
// consumers during the FSM-to-behavior migration.
//
// Sensor reports enabled in begin():
//   * SH2_GAME_ROTATION_VECTOR   - accel+gyro fusion, drift-bounded yaw,
//                                  no magnetometer (the arena's motors and
//                                  metal walls would corrupt it).
//   * SH2_GYROSCOPE_CALIBRATED   - calibrated angular velocity.
//   * SH2_LINEAR_ACCELERATION    - gravity-removed accel for future
//                                  bump-detection behaviors.
class ArduinoIMUSource : public IIMUSource, public IGyroSource {
public:
    ArduinoIMUSource(Adafruit_BNO08x* bno,
                     sh2_SensorValue_t* sensor_value,
                     IClock* clock = nullptr);

    // Initialise the I2C link and enable reports. Spins until the BNO
    // handshakes (preserves the original behavior). Safe to call repeatedly.
    bool begin();

    // IIMUSource: drains all pending BNO events into the cache, then returns
    // true if any modality refreshed since the last successful poll().
    bool poll(IMUSample* out) override;

    // IGyroSource (back-compat): drains pending events and returns the
    // latest omega_z if a new gyro sample arrived since the last call.
    bool poll_omega(float* omega_z) override;

private:
    void _drain_events();
    void _enable_reports();

    Adafruit_BNO08x*   _bno;
    sh2_SensorValue_t* _sv;
    IClock*            _clock;
    bool               _initialised;

    // Cached sample, sticky: each field reflects the latest value we've ever
    // seen; the *_fresh flags below track per-consumer freshness.
    IMUSample _cache;
    bool _yaw_fresh;
    bool _omega_fresh;
    bool _accel_fresh;
};

#endif

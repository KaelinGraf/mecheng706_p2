#ifndef HAL_ARDUINO_GYRO_SOURCE_H
#define HAL_ARDUINO_GYRO_SOURCE_H

#include <Adafruit_BNO08x.h>
#include "igyro_source.h"

// Wraps Adafruit's BNO08x driver and exposes the calibrated Z-axis gyroscope
// reports as a HAL IGyroSource. Owns the BNO08x init (I2C begin + report
// enable) so the controller side never sees the Adafruit type.
class ArduinoGyroSource : public IGyroSource {
public:
    ArduinoGyroSource(Adafruit_BNO08x* bno, sh2_SensorValue_t* sensor_value);

    // Initialise the I2C link and enable the gyro report. Returns true once
    // the driver has handshaked. Safe to call repeatedly; no-op once ready.
    bool begin();

    bool poll_omega(float* omega_z) override;

private:
    Adafruit_BNO08x*   _bno;
    sh2_SensorValue_t* _sensor_value;
    bool _initialised;
};

#endif

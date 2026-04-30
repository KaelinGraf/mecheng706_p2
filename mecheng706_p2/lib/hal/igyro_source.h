#ifndef HAL_IGYRO_SOURCE_H
#define HAL_IGYRO_SOURCE_H

// Single-axis Z-gyro source. The controller's Gyroscope class consumes
// (omega, dt) samples and integrates them; the source is responsible for
// producing fresh samples (driven by the BNO08x event reports on the
// Arduino target, or pushed in by the sim on the host target).
class IGyroSource {
public:
    virtual ~IGyroSource() = default;

    // Pulls a fresh angular-velocity sample if one is available. Returns
    // true and writes into *omega_z (rad/s) when a sample was produced
    // since the last successful poll; returns false otherwise.
    virtual bool poll_omega(float* omega_z) = 0;
};

#endif

#include "pan_servo.h"

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

PanServo::PanServo(IMotorOutput* hw, uint8_t channel, uint8_t pin,
                   float min_rad, float max_rad,
                   float mount_offset_rad,
                   uint16_t pulse_min_us, uint16_t pulse_max_us)
    : _hw(hw), _channel(channel), _pin(pin),
      _min_rad(min_rad), _max_rad(max_rad),
      _mount_offset_rad(mount_offset_rad),
      _pulse_min_us(pulse_min_us), _pulse_max_us(pulse_max_us),
      _current_rad(0.0f), _attached(false) {}

void PanServo::attach() {
    if (_attached || !_hw) return;
    _hw->attach(_channel, _pin);
    _attached = true;
    write_rad(0.0f);
}

void PanServo::write_rad(float angle_rad) {
    if (angle_rad < _min_rad) angle_rad = _min_rad;
    if (angle_rad > _max_rad) angle_rad = _max_rad;
    _current_rad = angle_rad;

    if (!_hw) return;

    // Apply mount offset: physical servo centre may not equal body-forward.
    const float servo_rad = angle_rad + _mount_offset_rad;

    // Map [-pi/2, +pi/2] across the configured pulse range. Centre of the
    // mechanical servo travel is the centre of the pulse range.
    const float t = (servo_rad + kPi * 0.5f) / kPi;     // 0..1 over [-pi/2, +pi/2]
    float clamped = t;
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;
    const uint16_t pw = (uint16_t)(_pulse_min_us +
        (uint32_t)(_pulse_max_us - _pulse_min_us) *
        (uint32_t)(clamped * 1000.0f) / 1000UL);
    _hw->write_us(_channel, pw);
}

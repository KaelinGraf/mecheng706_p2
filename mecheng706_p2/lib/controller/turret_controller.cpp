#include "turret_controller.h"

namespace {
inline float wrap_pi(float a) {
    constexpr float kPi = 3.14159265358979323846f;
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}
}

TurretController::TurretController(PanServo* servo, PhotoArray* photos,
                                   IMU* imu, IClock* clock,
                                   Config cfg)
    : _servo(servo), _photos(photos), _imu(imu), _clock(clock),
      _cfg(cfg),
      _state(IDLE),
      _pan_cmd(0.0f),
      _sweep_dir(+1),
      _lock_in_count(0),
      _lock_out_count(0),
      _world_lock_yaw(0.0f),
      _last_tick_us(0),
      _hold_request(false) {}

void TurretController::start() {
    if (_servo) _servo->attach();
    _enter(SWEEP);
}

void TurretController::set_hold(bool hold) {
    _hold_request = hold;
}

void TurretController::request_sweep() {
    _hold_request = false;
    _enter(SWEEP);
}

void TurretController::_enter(State s) {
    _state = s;
    _lock_in_count = 0;
    _lock_out_count = 0;
    if (s == LOCKED) {
        _world_lock_yaw = wrap_pi(_body_yaw() + _pan_cmd);
    }
}

float TurretController::_body_yaw() const {
    return _imu ? _imu->yaw() : 0.0f;
}

void TurretController::tick() {
    if (!_servo || !_photos || !_clock) return;

    const uint32_t now_us = _clock->now_us();
    float dt_s = 0.0f;
    if (_last_tick_us != 0) {
        const uint32_t du = now_us - _last_tick_us;
        dt_s = du * 1e-6f;
        if (dt_s > 0.2f) dt_s = 0.2f;          // clamp after long stalls
    }
    _last_tick_us = now_us;

    _photos->update();
    const float intensity = _photos->max_intensity();
    const float photo_err = _photos->bearing_error();

    switch (_state) {
        case IDLE:
            _pan_cmd = 0.0f;
            break;

        case SWEEP: {
            if (_hold_request) {
                // External request to freeze sweep without entering LOCKED.
                break;
            }
            // Triangle-wave pan command across the servo's mechanical range.
            _pan_cmd += _sweep_dir * _cfg.sweep_rate_rad_per_s * dt_s;
            if (_pan_cmd >= _servo->max_rad()) {
                _pan_cmd = _servo->max_rad();
                _sweep_dir = -1;
            } else if (_pan_cmd <= _servo->min_rad()) {
                _pan_cmd = _servo->min_rad();
                _sweep_dir = +1;
            }

            // Promote to LOCKED after `lock_count_in` consecutive bright reads.
            if (intensity >= _cfg.lock_threshold_v) {
                if (_lock_in_count < 255) ++_lock_in_count;
                if (_lock_in_count >= _cfg.lock_count_in) {
                    _enter(LOCKED);
                }
            } else {
                _lock_in_count = 0;
            }
            break;
        }

        case LOCKED: {
            // Hold the captured world bearing while the body rotates, then
            // refine with the photo-array soft-centroid error.
            const float body_yaw = _body_yaw();
            const float world_to_body = wrap_pi(_world_lock_yaw - body_yaw);
            float cmd = world_to_body + _cfg.track_kp * photo_err;

            // Update the captured world bearing so the array's centroid
            // becomes the new lock target as the candle is recentred.
            _world_lock_yaw = wrap_pi(body_yaw + cmd);
            _pan_cmd = cmd;

            // Demote to SWEEP after `lock_count_out` consecutive dim reads,
            // unless an external hold is requested (e.g. the extinguish
            // behavior wants the fan to keep aiming as the LED fades out;
            // it owns the release decision via its own debounce).
            if (intensity < _cfg.lost_threshold_v) {
                if (_lock_out_count < 255) ++_lock_out_count;
                if (!_hold_request && _lock_out_count >= _cfg.lock_count_out) {
                    _enter(SWEEP);
                }
            } else {
                _lock_out_count = 0;
            }
            break;
        }
    }

    _servo->write_rad(_pan_cmd);
}

TurretController::Snapshot TurretController::snapshot() const {
    Snapshot s;
    s.state = _state;
    s.is_locked = (_state == LOCKED);
    s.body_relative_bearing_rad = _pan_cmd;
    s.world_bearing_rad = wrap_pi(_body_yaw() + _pan_cmd);
    s.intensity = _photos ? _photos->max_intensity() : 0.0f;
    return s;
}

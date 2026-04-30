#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "iclock.h"

template<typename OutputType = uint8_t>
class PID {
    // Use PID<> myPid(values) to instantiate for PWM, else PID<type> myPid(values).
private:
    IClock* _clock;
    float _ki;
    float _kp;
    float _kd;
    float _f;
    float _prev_error = 0;
    float _error_integral = 0;
    uint32_t _prev_micros = 0;
    float _alpha = 0.7f;
    float _prev_derivative = 0;
    bool _integral_enabled = false;
    OutputType _output_min;
    OutputType _output_max;

public:
    PID(IClock* clock,
        float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float f = 0.0f,
        bool enable_integral = false,
        OutputType output_min = 0, OutputType output_max = 255)
        : _clock(clock), _ki(ki), _kp(kp), _kd(kd), _f(f),
          _integral_enabled(enable_integral),
          _output_min(output_min), _output_max(output_max) {}

    void setFeedForward(float f) { _f = f; }
    float getFeedForward() { return _f; }
    void setKi(float ki) { _ki = ki; }
    float getKi() { return _ki; }
    void setKd(float kd) { _kd = kd; }
    float getKd() { return _kd; }
    void setKp(float kp) { _kp = kp; }
    float getKp() { return _kp; }
    void enableIntegral() { _integral_enabled = true; }
    void disableIntegral() { _integral_enabled = false; }
    void setOutputLimits(OutputType min, OutputType max) {
        _output_min = min;
        _output_max = max;
    }
    OutputType update(float error, float derivative = 0);
    void resetPID() {
        _prev_error = 0.0f;
        _error_integral = 0.0f;
        if (_clock) _prev_micros = _clock->now_us();
    }
};

template<typename OutputType>
OutputType PID<OutputType>::update(float error, float derivative) {
    float d_term = 0;
    float filtered_derivative = 0;
    uint32_t now = _clock ? _clock->now_us() : _prev_micros;
    float delta_time = (now - _prev_micros) / 1000000.0f;
    _prev_micros = now;
    float p_term = _kp * error;
    _error_integral = (_integral_enabled) ? _error_integral + (error * delta_time) : 0.0f;
    float i_term = _error_integral * _ki;
    if (derivative == 0) {
        d_term = _kd * ((error - _prev_error) / delta_time);
    } else {
        d_term = _kd * derivative;
    }
    filtered_derivative = (_alpha * d_term) + ((1.0f - _alpha) * _prev_derivative);
    _prev_derivative = filtered_derivative;
    float control_effort = p_term + i_term - filtered_derivative;

    if (control_effort > static_cast<float>(_output_max)) {
        control_effort = _output_max;
        if (error > 0) {
            _error_integral = (_integral_enabled) ? _error_integral - (error * delta_time) : 0.0f;
        }
    } else if (control_effort < static_cast<float>(_output_min)) {
        control_effort = _output_min;
        if (error < 0) {
            _error_integral = (_integral_enabled) ? _error_integral - (error * delta_time) : 0.0f;
        }
    }
    _prev_error = error;

    return static_cast<OutputType>(control_effort);
}

#endif

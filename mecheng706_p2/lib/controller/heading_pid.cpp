#include "heading_pid.h"

HeadingPid::HeadingPid(float kp, float ki, float kd, float effort_limit)
    : _kp(kp), _ki(ki), _kd(kd),
      _i_acc(0.0f), _last_err(0.0f), _has_last(false),
      _effort_limit(effort_limit) {}

void HeadingPid::reset() {
    _i_acc = 0.0f;
    _last_err = 0.0f;
    _has_last = false;
}

void HeadingPid::set_gains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float HeadingPid::compute(float err_rad, float dt_s) {
    float p = _kp * err_rad;

    if (dt_s > 0.0f) {
        _i_acc += err_rad * dt_s;
    }
    float i = _ki * _i_acc;

    float d = 0.0f;
    if (_has_last && dt_s > 0.0f) {
        d = _kd * (err_rad - _last_err) / dt_s;
    }
    _last_err = err_rad;
    _has_last = true;

    float u = p + i + d;
    if (u >  _effort_limit) u =  _effort_limit;
    if (u < -_effort_limit) u = -_effort_limit;
    return u;
}

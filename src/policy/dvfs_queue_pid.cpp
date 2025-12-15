#include "policy/dvfs_queue_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

QueuePIDPolicy::QueuePIDPolicy(double target, double kp, double ki, double kd,
                               double min_scale, double max_scale,
                               bool per_router, double headroom_margin)
    : _target(target), _kp(kp), _ki(ki), _kd(kd),
      _min_scale(min_scale), _max_scale(max_scale),
      _per_router(per_router), _headroom_margin(headroom_margin) {}

void QueuePIDPolicy::_EnsureSize(size_t n) {
  if(_int_err.size() < n) {
    _int_err.assign(n, 0.0);
    _prev_err.assign(n, 0.0);
    _prev_scale.assign(n, 1.0);
  }
}

void QueuePIDPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) {
  (void)epoch;
  size_t n_units = _per_router ? pwr.router_occupancy.size() : 1;
  _EnsureSize(n_units);

  auto clamp = [&](double v)->double {
    if(v > _max_scale) return _max_scale;
    if(v < _min_scale) return _min_scale;
    return v;
  };

  auto headroom_ok = [&](double new_scale, double old_scale)->bool {
    if(_headroom_margin <= 0.0) return true;
    if(new_scale <= old_scale) return true;
    return pwr.headroom > _headroom_margin;
  };

  if(_per_router) {
    for(size_t r = 0; r < n_units; ++r) {
      double meas = (r < pwr.router_occupancy.size()) ? pwr.router_occupancy[r] : 0.0;
      // FIXED: Invert error sign so higher occupancy → speed up
      double err = meas - _target;  // Positive when congested
      _int_err[r] += err;
      double deriv = err - _prev_err[r];
      double delta = _kp * err + _ki * _int_err[r] + _kd * deriv;
      double new_scale = clamp(_prev_scale[r] + delta);
      if(headroom_ok(new_scale, _prev_scale[r])) {
        net.SetRouterSpeed(static_cast<int>(r), new_scale);
        _prev_scale[r] = new_scale;
      }
      _prev_err[r] = err;
    }
  } else {
    double meas = 0.0;
    if(!pwr.router_occupancy.empty()) {
      double sum = 0.0;
      for(double v : pwr.router_occupancy) sum += v;
      meas = sum / static_cast<double>(pwr.router_occupancy.size());
    }
    // FIXED: Invert error sign so higher occupancy → speed up
    double err = meas - _target;  // Positive when congested
    _int_err[0] += err;
    double deriv = err - _prev_err[0];
    double delta = _kp * err + _ki * _int_err[0] + _kd * deriv;
    double new_scale = clamp(_prev_scale[0] + delta);
    
    std::cout << "QUEUE_PID: epoch=" << epoch 
              << " target=" << _target 
              << " meas=" << meas 
              << " err=" << err 
              << " delta=" << delta 
              << " old_scale=" << _prev_scale[0]
              << " new_scale=" << new_scale << std::endl;
    
    if(headroom_ok(new_scale, _prev_scale[0])) {
      net.SetDomainSpeed(0, new_scale);
      _prev_scale[0] = new_scale;
    }
    _prev_err[0] = err;
  }
}


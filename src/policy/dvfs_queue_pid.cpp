#include "policy/dvfs_queue_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

QueuePIDPolicy::QueuePIDPolicy(double target, double kp, double ki, double kd,
                               double min_scale, double max_scale,
                               bool per_router, double headroom_margin,
                               int control_class)
    : _target(target), _kp(kp), _ki(ki), _kd(kd),
      _min_scale(min_scale), _max_scale(max_scale),
      _per_router(per_router), _headroom_margin(headroom_margin),
      _control_class(control_class) {}

void QueuePIDPolicy::_EnsureSize(size_t n) {
  if(_int_err.size() < n) {
    _int_err.assign(n, 0.0);
    _prev_err.assign(n, 0.0);
    _prev_scale.assign(n, _max_scale);
  }
}

void QueuePIDPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) {
  (void)epoch;
  
  const size_t n_routers = pwr.router_occupancy.size();
  if(n_routers == 0) return;
  
  _EnsureSize(n_routers);

  auto clamp = [&](double v)->double {
    return std::max(_min_scale, std::min(_max_scale, v));
  };

  // === CLASS-AWARE LATENCY CHECK ===
  double class_latency = 0.0;
  if(!pwr.class_latency_p99.empty()) {
    if(_control_class >= 0 && _control_class < static_cast<int>(pwr.class_latency_p99.size())) {
      class_latency = pwr.class_latency_p99[_control_class];
    } else if(!pwr.class_latency_p99.empty()) {
      class_latency = pwr.class_latency_p99[0];
    }
  }
  
  // Priority class needs attention if latency > SLO (30 cycles)
  bool priority_stressed = (class_latency > 30.0);
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER CLASS-AWARE PID ===
    
    double max_occ = 0.0, sum_occ = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      max_occ = std::max(max_occ, pwr.router_occupancy[r]);
      sum_occ += pwr.router_occupancy[r];
    }
    double avg_occ = sum_occ / n_routers;
    
    // Power budget
    double power_budget = 1.0;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      power_budget = 1.0 - std::min(over * 1.2, 0.4);
    }
    
    for(size_t r = 0; r < n_routers; ++r) {
      double occ = pwr.router_occupancy[r];
      double rel_load = (max_occ > 1e-9) ? occ / max_occ : 0.0;
      
      // Adaptive target
      double target = _target;
      if(priority_stressed) {
        target = _target * 0.3;  // Aggressive speedup
      } else if(rel_load > 0.7) {
        target = _target * 0.5;
      } else if(rel_load < 0.3) {
        target = _target * 1.5;
      }
      
      // PID with anti-windup
      double err = occ - target;
      bool sat = (_prev_scale[r] <= _min_scale && err < 0) ||
                 (_prev_scale[r] >= _max_scale && err > 0);
      if(!sat) _int_err[r] = _int_err[r] * 0.85 + err;
      _int_err[r] = std::max(-3.0, std::min(3.0, _int_err[r]));
      double deriv = 0.5 * (err - _prev_err[r]);
      _prev_err[r] = err;
      
      // Tuned gains
      double delta = 0.15 * err + 0.02 * _int_err[r] + 0.03 * deriv;
      double target_scale = _prev_scale[r] + delta;
      
      // Class-aware budget
      double budget = power_budget;
      if(priority_stressed && rel_load > 0.5) {
        budget = std::max(power_budget, 0.9);
      } else if(rel_load < 0.3) {
        budget = power_budget * 0.7;
      }
      
      target_scale = _min_scale + budget * (clamp(target_scale) - _min_scale);
      double new_scale = 0.7 * _prev_scale[r] + 0.3 * clamp(target_scale);
      
      net.SetRouterSpeed(static_cast<int>(r), new_scale);
      _prev_scale[r] = new_scale;
    }
    
    std::cout << "QUEUE_PID_CLASS: epoch=" << epoch 
              << " class" << _control_class << "_lat=" << class_latency
              << " stressed=" << priority_stressed
              << " budget=" << power_budget << std::endl;
    
  } else {
    // === GLOBAL CLASS-AWARE PID ===
    double meas = 0.0;
    for(double v : pwr.router_occupancy) meas += v;
    meas /= std::max(1.0, static_cast<double>(pwr.router_occupancy.size()));
    
    double target = priority_stressed ? _target * 0.5 : _target;
    double err = meas - target;
    _int_err[0] = std::max(-5.0, std::min(5.0, _int_err[0] * 0.9 + err));
    double deriv = err - _prev_err[0];
    
    double delta = 0.2 * err + 0.02 * _int_err[0] + 0.05 * deriv;
    double target_scale = clamp(_prev_scale[0] + delta);
    
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      double budget = 1.0 - std::min(over * 1.2, 0.4);
      target_scale = _min_scale + budget * (target_scale - _min_scale);
    }
    
    double new_scale = 0.7 * _prev_scale[0] + 0.3 * clamp(target_scale);
    net.SetDomainSpeed(0, new_scale);
    _prev_scale[0] = new_scale;
    _prev_err[0] = err;
    
    std::cout << "QUEUE_PID: epoch=" << epoch 
              << " class_lat=" << class_latency
              << " scale=" << new_scale << std::endl;
  }
}


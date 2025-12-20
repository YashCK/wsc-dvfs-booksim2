#include "policy/dvfs_queue_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

QueuePIDPolicy::QueuePIDPolicy(double target, double kp, double ki, double kd,
                               double min_scale, double max_scale,
                               bool per_router, double headroom_margin,
                               int control_class, double control_slo)
    : _target(target), _kp(kp), _ki(ki), _kd(kd),
      _min_scale(min_scale), _max_scale(max_scale),
      _per_router(per_router), _headroom_margin(headroom_margin),
      _control_class(control_class), _control_slo(control_slo) {}

void QueuePIDPolicy::_EnsureSize(size_t n) {
  if(_int_err.size() < n) {
    _int_err.assign(n, 0.0);
    _prev_err.assign(n, 0.0);
    _prev_scale.assign(n, 1.0);
  }
}

void QueuePIDPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) {
  (void)epoch;
  
  const size_t n_routers = pwr.router_occupancy.size();
  if(n_routers == 0) return;
  
  _EnsureSize(n_routers);

  auto clamp = [&](double v)->double {
    if(v > _max_scale) return _max_scale;
    if(v < _min_scale) return _min_scale;
    return v;
  };

  // DIFFERENTIATED CONTROL: Use PID per-router with power budget awareness
  // Key insight: High-load routers need speed, low-load routers can be throttled
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER DIFFERENTIATED PID ===
    // Strategy: Each router runs its own PID, but we bias allocations
    // based on relative load to prioritize busy routers
    
    // 1. Compute load metrics
    double max_occ = 0.0;
    double sum_occ = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      max_occ = std::max(max_occ, pwr.router_occupancy[r]);
      sum_occ += pwr.router_occupancy[r];
    }
    double avg_occ = sum_occ / n_routers;
    
    // 2. Compute power budget - how much headroom do we have?
    double power_budget_factor = 1.0;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      // Over budget - need to throttle overall
      double over_ratio = -pwr.headroom / pwr.power_cap;
      power_budget_factor = 1.0 - std::min(over_ratio * 1.5, 0.5);
    }
    
    // 3. Run PID for each router with load-aware bias
    for(size_t r = 0; r < n_routers; ++r) {
      double meas = pwr.router_occupancy[r];
      
      // Compute relative load (0 = idle, 1 = busiest)
      double rel_load = (max_occ > 1e-9) ? meas / max_occ : 0.0;
      
      // MODIFIED PID: Target varies by load
      // High-load routers: try to reduce congestion (run fast)
      // Low-load routers: can tolerate more occupancy (run slow)
      double adaptive_target = _target;
      if(rel_load > 0.5) {
        // Busy router: lower target to trigger speedup
        adaptive_target = _target * 0.5;
      } else {
        // Idle router: raise target to allow slowdown
        adaptive_target = _target * 2.0;
      }
      
      // PID error: positive when occupancy > target (congested)
      // We want: congested → speed up, idle → slow down
      double err = meas - adaptive_target;
      _int_err[r] = _int_err[r] * 0.9 + err; // decay integral term
      double deriv = err - _prev_err[r];
      
      // PID output: positive error → positive delta → speed up
      // Use POSITIVE gains (Kp > 0) since we want congestion to increase freq
      double delta = std::abs(_kp) * err + std::abs(_ki) * _int_err[r] + std::abs(_kd) * deriv;
      
      // Apply power budget factor - scale down if over cap
      // But high-load routers get priority (less throttling)
      double router_budget = power_budget_factor;
      if(rel_load > 0.5) {
        // High-load router: protect from excessive throttling
        router_budget = std::max(power_budget_factor, 0.8);
      } else {
        // Low-load router: can throttle more aggressively
        router_budget = power_budget_factor * (0.5 + 0.5 * rel_load);
      }
      
      double new_scale = clamp(_prev_scale[r] + delta);
      new_scale = clamp(new_scale * router_budget / std::max(0.5, _prev_scale[r] / _max_scale));
      
      // Simpler: just apply budget factor to computed scale
      new_scale = clamp(_prev_scale[r] + delta);
      double budget_adjusted = _min_scale + router_budget * (new_scale - _min_scale);
      new_scale = clamp(budget_adjusted);
      
      net.SetRouterSpeed(static_cast<int>(r), new_scale);
      _prev_scale[r] = new_scale;
      _prev_err[r] = err;
    }
    
    std::cout << "QUEUE_PID_DIFF: epoch=" << epoch 
              << " max_occ=" << max_occ 
              << " avg_occ=" << avg_occ
              << " budget=" << power_budget_factor
              << " headroom=" << pwr.headroom << std::endl;
    
  } else {
    // === GLOBAL PID CONTROL WITH CLASS AWARENESS ===
    // Use both queue occupancy and control class latency for feedback
    
    double meas = 0.0;
    if(!pwr.router_occupancy.empty()) {
      for(double v : pwr.router_occupancy) meas += v;
      meas /= static_cast<double>(pwr.router_occupancy.size());
    }
    
    // Get control class latency for SLO-aware adjustment
    double control_latency = 0.0;
    if(!pwr.class_latency_p99.empty() && _control_class >= 0 &&
       _control_class < static_cast<int>(pwr.class_latency_p99.size())) {
      control_latency = pwr.class_latency_p99[_control_class];
    }
    
    // PID: we want to maintain occupancy near target
    // If occupancy > target (congested): speed up
    // If occupancy < target (idle): slow down for power
    double err = meas - _target;
    
    // CLASS-AWARE ADJUSTMENT: If control class latency exceeds SLO, boost speed
    double latency_boost = 0.0;
    if(_control_slo > 0.0 && control_latency > _control_slo) {
      // Control class is exceeding SLO - need to prioritize speed
      double overshoot = (control_latency - _control_slo) / _control_slo;
      latency_boost = std::min(overshoot * 0.2, 0.3);  // Up to 30% boost
    }
    
    _int_err[0] = _int_err[0] * 0.9 + err;
    double deriv = err - _prev_err[0];
    
    // Positive gains: congestion → positive error → speed up
    double delta = std::abs(_kp) * err + std::abs(_ki) * _int_err[0] + std::abs(_kd) * deriv;
    double new_scale = clamp(_prev_scale[0] + delta + latency_boost);
    
    // Power cap enforcement - but be gentler if control latency is high
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      double throttle_factor = 1.5;
      // If control class is struggling, reduce throttling intensity
      if(_control_slo > 0.0 && control_latency > _control_slo * 0.8) {
        throttle_factor = 1.0;  // Gentler throttling when latency is high
      }
      new_scale = new_scale * (1.0 - std::min(over_ratio * throttle_factor, 0.4));
      new_scale = clamp(new_scale);
    }
    
    std::cout << "QUEUE_PID: epoch=" << epoch 
              << " target=" << _target 
              << " meas=" << meas 
              << " err=" << err 
              << " delta=" << delta 
              << " latency_boost=" << latency_boost
              << " ctrl_lat=" << control_latency
              << " ctrl_slo=" << _control_slo
              << " headroom=" << pwr.headroom << std::endl;
    
    net.SetDomainSpeed(0, new_scale);
    _prev_scale[0] = new_scale;
    _prev_err[0] = err;
  }
}


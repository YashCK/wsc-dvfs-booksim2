#include "policy/dvfs_perf_target.hpp"

#include <algorithm>
#include <cctype>
#include <numeric>
#include <iostream>
#include <cmath>

PerfTargetDVFSPolicy::PerfTargetDVFSPolicy(std::string metric, double target_value,
                                           int target_class, double kp,
                                           double min_scale, double max_scale,
                                           bool per_router, double headroom_margin)
    : _metric(metric), _target(target_value), _class(target_class), _kp(kp),
      _min_scale(min_scale), _max_scale(max_scale),
      _per_router(per_router), _headroom_margin(headroom_margin) {}

void PerfTargetDVFSPolicy::_EnsureSize(size_t n) {
  if(_prev_scale.size() < n) {
    _prev_scale.assign(n, 1.0);
  }
}

void PerfTargetDVFSPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) {
  (void)epoch;
  
  const size_t n_routers = pwr.router_power.size();
  if(n_routers == 0) return;
  
  _EnsureSize(n_routers);

  auto clamp = [&](double v)->double {
    if(v > _max_scale) return _max_scale;
    if(v < _min_scale) return _min_scale;
    return v;
  };

  auto lower = _metric;
  std::transform(lower.begin(), lower.end(), lower.begin(), 
                 [](unsigned char c){ return std::tolower(c); });

  auto measure_latency = [&]()->double {
    if(!pwr.class_latency_p99.empty() && _class >= 0 &&
       _class < static_cast<int>(pwr.class_latency_p99.size())) {
      return pwr.class_latency_p99[_class];
    }
    if(!pwr.class_latency_p99.empty()) return pwr.class_latency_p99[0];
    return 0.0;
  };

  // DIFFERENTIATED CONTROL: Meet latency SLO while minimizing power
  // Key insight: Only throttle routers that have headroom to spare
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER LATENCY-AWARE CONTROL ===
    // Strategy: 
    // 1. If latency is within SLO, throttle low-load routers to save power
    // 2. If latency exceeds SLO, speed up high-load (critical path) routers
    
    double current_latency = measure_latency();
    
    // Collect router loads
    double max_occ = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      if(r < pwr.router_occupancy.size()) {
        max_occ = std::max(max_occ, pwr.router_occupancy[r]);
      }
    }
    
    // Compute power budget
    double power_budget = 1.0;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      power_budget = 1.0 - std::min(over_ratio * 1.5, 0.5);
    }
    
    // Determine control mode based on latency
    bool latency_ok = (current_latency <= _target) || (current_latency < 1e-9);
    
    for(size_t r = 0; r < n_routers; ++r) {
      double occ = (r < pwr.router_occupancy.size()) ? pwr.router_occupancy[r] : 0.0;
      double rel_load = (max_occ > 1e-9) ? occ / max_occ : 0.0;
      
      double target_scale;
      
      if(!latency_ok) {
        // LATENCY TOO HIGH - need to speed up
        // Prioritize high-load routers (they're likely on critical path)
        if(rel_load > 0.5) {
          // High-load router: run at max speed
          target_scale = _max_scale;
        } else {
          // Low-load router: can still throttle somewhat
          target_scale = _min_scale + rel_load * (_max_scale - _min_scale);
        }
      } else {
        // LATENCY OK - can throttle for power savings
        // Throttle low-load routers more aggressively
        double throttle_factor = power_budget;
        if(rel_load < 0.3) {
          // Very idle - can throttle a lot
          throttle_factor *= 0.6;
        } else if(rel_load < 0.6) {
          // Moderate load - throttle moderately
          throttle_factor *= 0.8;
        }
        // else: high load - minimal throttle (throttle_factor stays at power_budget)
        
        target_scale = _min_scale + throttle_factor * (_max_scale - _min_scale);
      }
      
      target_scale = clamp(target_scale);
      net.SetRouterSpeed(static_cast<int>(r), target_scale);
      _prev_scale[r] = target_scale;
    }
    
    std::cout << "PERF_TARGET_DIFF: epoch=" << epoch 
              << " latency=" << current_latency 
              << " target=" << _target
              << " latency_ok=" << latency_ok
              << " budget=" << power_budget
              << " headroom=" << pwr.headroom << std::endl;
    
  } else {
    // === GLOBAL LATENCY-AWARE CONTROL ===
    // If latency exceeds SLO, speed up; otherwise throttle for power
    
    double current_latency = measure_latency();
    double target_scale = _prev_scale[0];
    
    if(current_latency > _target && current_latency > 1e-9) {
      // Latency exceeds SLO - need to speed up
      double overshoot = (current_latency - _target) / _target;
      double delta = _kp * overshoot;
      target_scale = clamp(_prev_scale[0] + delta);
    } else if(current_latency > 1e-9) {
      // Latency within SLO - can try to throttle for power
      double margin = (_target - current_latency) / _target;
      // Only throttle if we have significant margin
      if(margin > 0.2) {
        double delta = -_kp * margin * 0.5; // gentle throttle
        target_scale = clamp(_prev_scale[0] + delta);
      }
    }
    
    // Power cap enforcement
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      target_scale = target_scale * (1.0 - std::min(over_ratio * 1.5, 0.4));
      target_scale = clamp(target_scale);
    }
    
    std::cout << "PERF_TARGET: epoch=" << epoch 
              << " latency=" << current_latency 
              << " target=" << _target
              << " new_scale=" << target_scale
              << " headroom=" << pwr.headroom << std::endl;
    
    net.SetDomainSpeed(0, target_scale);
    _prev_scale[0] = target_scale;
  }
}


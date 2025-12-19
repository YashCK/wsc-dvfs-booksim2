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
    _prev_scale.assign(n, _max_scale);
    _integral_err.assign(n, 0.0);
  }
}

void PerfTargetDVFSPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) {
  (void)epoch;
  
  const size_t n_routers = pwr.router_power.size();
  if(n_routers == 0) return;
  
  _EnsureSize(n_routers);

  auto clamp = [&](double v)->double {
    return std::max(_min_scale, std::min(_max_scale, v));
  };

  // === CLASS-AWARE LATENCY MEASUREMENT ===
  auto measure_class_latency = [&](int cls)->double {
    if(!pwr.class_latency_p99.empty() && cls >= 0 &&
       cls < static_cast<int>(pwr.class_latency_p99.size())) {
      return pwr.class_latency_p99[cls];
    }
    if(!pwr.class_latency_p99.empty()) return pwr.class_latency_p99[0];
    return 0.0;
  };
  
  double priority_latency = measure_class_latency(_class);
  double other_latency = 0.0;
  if(pwr.class_latency_p99.size() > 1) {
    int other_class = (_class == 0) ? 1 : 0;
    other_latency = measure_class_latency(other_class);
  }
  
  // Priority class stress level (0-1, higher = more urgent)
  double priority_stress = 0.0;
  if(priority_latency > 1e-9) {
    priority_stress = std::min(2.0, priority_latency / _target) - 1.0;
    priority_stress = std::max(0.0, priority_stress);
  }
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER CLASS-AWARE SLO CONTROL ===
    
    double max_occ = 0.0, sum_occ = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      if(r < pwr.router_occupancy.size()) {
        max_occ = std::max(max_occ, pwr.router_occupancy[r]);
        sum_occ += pwr.router_occupancy[r];
      }
    }
    double avg_occ = sum_occ / std::max(1.0, (double)n_routers);
    
    // Power budget
    double power_budget = 1.0;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      power_budget = 1.0 - std::min(over * 1.2, 0.4);
    }
    
    // Latency error (positive = SLO violated)
    double lat_err = (priority_latency - _target) / std::max(_target, 1.0);
    
    for(size_t r = 0; r < n_routers; ++r) {
      double occ = (r < pwr.router_occupancy.size()) ? pwr.router_occupancy[r] : 0.0;
      double rel_load = (max_occ > 1e-9) ? occ / max_occ : 0.0;
      
      double target_scale;
      
      if(lat_err > 0.0) {
        // === SLO VIOLATED - boost proportionally ===
        // High-load routers get more boost (critical path)
        double boost = _kp * lat_err * (0.5 + 0.5 * rel_load);
        target_scale = clamp(_prev_scale[r] + boost);
        
        // Priority class protection: minimal throttling on high-load routers
        double eff_budget = power_budget;
        if(rel_load > 0.5 && priority_stress > 0.3) {
          eff_budget = std::max(power_budget, 0.9);
        }
        target_scale = _min_scale + eff_budget * (target_scale - _min_scale);
        
      } else {
        // === SLO MET - save power on idle routers ===
        double margin = std::max(0.0, -lat_err);  // How much under SLO
        
        // Calculate throttle based on load and margin
        double throttle = 0.0;
        if(rel_load < 0.3) {
          // Very idle router - aggressive throttle
          throttle = _kp * margin * 0.5;
        } else if(rel_load < 0.6) {
          // Moderate load - gentle throttle
          throttle = _kp * margin * 0.2;
        }
        // High load routers: no throttle, keep ready
        
        target_scale = clamp(_prev_scale[r] - throttle);
        
        // Apply power budget uniformly when SLO is met
        target_scale = _min_scale + power_budget * (target_scale - _min_scale);
      }
      
      // Smooth transitions
      double new_scale = 0.6 * _prev_scale[r] + 0.4 * clamp(target_scale);
      net.SetRouterSpeed(static_cast<int>(r), new_scale);
      _prev_scale[r] = new_scale;
    }
    
    std::cout << "PERF_TARGET_CLASS: epoch=" << epoch 
              << " class" << _class << "_lat=" << priority_latency 
              << " target=" << _target
              << " lat_err=" << lat_err
              << " stress=" << priority_stress
              << " budget=" << power_budget << std::endl;
    
  } else {
    // === GLOBAL CLASS-AWARE SLO CONTROL ===
    
    double lat_err = (priority_latency - _target) / std::max(_target, 1.0);
    double target_scale;
    
    if(lat_err > 0.0) {
      // SLO violated - proportional boost
      double boost = _kp * lat_err;
      target_scale = clamp(_prev_scale[0] + boost);
    } else {
      // SLO met - gentle throttle
      double margin = -lat_err;
      if(margin > 0.3) {
        target_scale = clamp(_prev_scale[0] - _kp * margin * 0.3);
      } else {
        target_scale = _prev_scale[0];
      }
    }
    
    // Power cap enforcement
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      double budget = 1.0 - std::min(over * 1.2, 0.4);
      target_scale = _min_scale + budget * (target_scale - _min_scale);
    }
    
    double new_scale = 0.7 * _prev_scale[0] + 0.3 * clamp(target_scale);
    
    std::cout << "PERF_TARGET: epoch=" << epoch 
              << " class_lat=" << priority_latency 
              << " target=" << _target
              << " lat_err=" << lat_err
              << " new_scale=" << new_scale << std::endl;
    
    net.SetDomainSpeed(0, new_scale);
    _prev_scale[0] = new_scale;
  }
}


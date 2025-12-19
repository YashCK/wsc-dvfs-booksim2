#include "policy/dvfs_policy.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <numeric>
#include <cmath>

namespace {
double norm_signal(const std::vector<double> &v) {
  if(v.empty()) return 0.0;
  double sum = 0.0;
  for(double x : v) sum += x;
  return sum / static_cast<double>(v.size());
}
} // namespace

static std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

void HWReactiveDVFSPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net,
                                  int epoch) {
  if((_last_change_epoch >= 0) && (_hysteresis > 0) &&
     ((epoch - _last_change_epoch) < _hysteresis)) {
    return;
  }
  auto lower_str = to_lower(_signal);

  const size_t n_routers = pwr.router_occupancy.size();
  if(n_routers == 0) return;

  // === CLASS-AWARE LATENCY CHECK ===
  auto get_class_latency = [&](int cls)->double {
    if(!pwr.class_latency_p99.empty() && cls >= 0 &&
       cls < static_cast<int>(pwr.class_latency_p99.size())) {
      return pwr.class_latency_p99[cls];
    }
    if(!pwr.class_latency_p99.empty()) return pwr.class_latency_p99[0];
    return 0.0;
  };
  
  double priority_latency = get_class_latency(_control_class);
  double other_latency = 0.0;
  if(pwr.class_latency_p99.size() > 1) {
    int other_class = (_control_class == 0) ? 1 : 0;
    other_latency = get_class_latency(other_class);
  }
  
  // Priority class stress (0 = fine, 1+ = critical)
  double priority_stress = 0.0;
  if(priority_latency > 1e-9 && _control_slo > 0.0) {
    priority_stress = std::max(0.0, (priority_latency / _control_slo) - 1.0);
  }
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER CLASS-AWARE DIFFERENTIATED CONTROL ===
    
    // 1. Collect signals for each router
    std::vector<double> signals(n_routers);
    double max_signal = 0.0, sum_signal = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      if(lower_str == "queue") {
        signals[r] = pwr.router_occupancy[r];
      } else if(lower_str == "inj") {
        signals[r] = (r < pwr.router_injection_rate.size()) ? pwr.router_injection_rate[r] : 0.0;
      } else if(lower_str == "stall") {
        signals[r] = (r < pwr.router_stall_rate.size()) ? pwr.router_stall_rate[r] : 0.0;
      } else {
        signals[r] = pwr.router_occupancy[r];
      }
      max_signal = std::max(max_signal, signals[r]);
      sum_signal += signals[r];
    }
    double avg_signal = (n_routers > 0) ? (sum_signal / n_routers) : 0.0;
    
    // 2. Compute power budget with smooth adjustment
    double target_base = _high_scale;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      target_base = _high_scale * (1.0 - std::min(over * 1.2, 0.35));
    }
    double base_scale = 0.7 * _current_scale + 0.3 * target_base;
    base_scale = std::max(_low_scale, std::min(_high_scale, base_scale));
    
    // 3. CLASS-AWARE frequency allocation
    if(_router_scales.size() < n_routers) {
      _router_scales.resize(n_routers, _high_scale);
    }
    
    for(size_t r = 0; r < n_routers; ++r) {
      double rel_load = (max_signal > 1e-9) ? signals[r] / max_signal : 0.0;
      double target_scale;
      
      if(priority_stress > 0.3) {
        // === PRIORITY CLASS STRESSED - boost high-load routers ===
        double boost = std::min(priority_stress, 1.0) * 0.3;  // Up to 30% boost
        if(rel_load > 0.5) {
          // Hot router: full boost
          target_scale = base_scale + boost * (_high_scale - base_scale);
          target_scale = std::min(_high_scale, target_scale * (1.0 + boost * 0.2));
        } else if(rel_load > 0.3) {
          // Warm router: partial boost
          target_scale = base_scale + 0.5 * boost * (_high_scale - base_scale);
        } else {
          // Cold router: maintain minimal to save power for hot ones
          target_scale = _low_scale + 0.3 * (base_scale - _low_scale);
        }
      } else {
        // === PRIORITY CLASS OK - normal differentiation ===
        if(signals[r] > avg_signal * 1.2) {
          // Hot router: keep ready
          target_scale = base_scale + 0.3 * (_high_scale - base_scale);
        } else if(signals[r] < avg_signal * 0.5) {
          // Cold router: save power
          target_scale = _low_scale + 0.4 * (base_scale - _low_scale);
        } else {
          // Normal router: base scale
          target_scale = base_scale;
        }
      }
      
      // Smooth per-router transition
      target_scale = 0.6 * _router_scales[r] + 0.4 * target_scale;
      target_scale = std::max(_low_scale, std::min(_high_scale, target_scale));
      net.SetRouterSpeed(static_cast<int>(r), target_scale);
      _router_scales[r] = target_scale;
    }
    
    _current_scale = base_scale;
    _last_change_epoch = epoch;
    
    std::cout << "HW_REACTIVE_CLASS: epoch=" << epoch 
              << " class" << _control_class << "_lat=" << priority_latency
              << " stress=" << priority_stress
              << " base=" << base_scale << std::endl;
    
  } else {
    // === GLOBAL CLASS-AWARE CONTROL ===
    
    auto pick_scale = [&](double val, double headroom)->double {
      // Priority class override
      if(priority_stress > 0.3) return _high_scale;
      
      if((lower_str == "latency") && (_control_slo > 0.0) && (val > _control_slo)) {
        return _high_scale;
      }
      if(val >= _high_thresh) return _high_scale;
      if(val <= _low_thresh) {
        if((_headroom_margin > 0.0) && (headroom <= _headroom_margin)) {
          return -1.0;
        }
        return _low_scale;
      }
      return -1.0;
    };

    double sig = 0.0;
    if(lower_str == "queue") {
      sig = norm_signal(pwr.router_occupancy);
    } else if(lower_str == "inj") {
      sig = norm_signal(pwr.router_injection_rate);
    } else if(lower_str == "stall") {
      sig = norm_signal(pwr.router_stall_rate);
    } else if(lower_str == "latency") {
      sig = priority_latency;
    }
    
    double target = pick_scale(sig, pwr.headroom);
    if(target < 0.0) target = _current_scale;
    
    // Power cap enforcement
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over = -pwr.headroom / pwr.power_cap;
      target = target * (1.0 - std::min(over * 1.5, 0.4));
      target = std::max(target, _low_scale);
    }
    
    // Smooth transition
    target = 0.7 * _current_scale + 0.3 * std::max(_low_scale, std::min(_high_scale, target));
    
    std::cout << "HW_REACTIVE: epoch=" << epoch 
              << " class_lat=" << priority_latency 
              << " sig=" << sig 
              << " scale=" << target << std::endl;
    
    net.SetDomainSpeed(0, target);
    _current_scale = target;
    _last_change_epoch = epoch;
  }
}

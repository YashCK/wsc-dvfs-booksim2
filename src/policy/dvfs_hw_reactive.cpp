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

  // DIFFERENTIATED CONTROL: Prioritize high-load routers, throttle idle ones
  // This lets us achieve LOWER latency than uniform throttle at same power!
  
  if(_per_router && n_routers > 1) {
    // === PER-ROUTER DIFFERENTIATED CONTROL ===
    // Strategy: Compute load-weighted frequency allocation
    // High-load routers get high freq, idle routers get low freq
    
    // 1. Collect signals for each router
    std::vector<double> signals(n_routers);
    double max_signal = 0.0;
    double sum_signal = 0.0;
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
    
    // 2. IMPROVED: Use latency as secondary signal for prioritization
    double current_latency = 0.0;
    if(!pwr.class_latency_p99.empty()) {
      current_latency = pwr.class_latency_p99[0];
    }
    bool latency_critical = (current_latency > _control_slo) && (_control_slo > 0.0);
    
    // 3. Compute power budget with GRADUAL adjustment (anti-oscillation)
    double target_base_scale = _high_scale;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      // Gentler response to avoid oscillation
      target_base_scale = _high_scale * (1.0 - std::min(over_ratio * 1.2, 0.35));
    }
    // Smooth transition (exponential moving average)
    double base_scale = 0.7 * _current_scale + 0.3 * target_base_scale;
    base_scale = std::max(_low_scale, std::min(_high_scale, base_scale));
    
    // 4. IMPROVED: Smart frequency allocation
    double avg_signal = (n_routers > 0) ? (sum_signal / n_routers) : 0.0;
    
    for(size_t r = 0; r < n_routers; ++r) {
      double target_scale;
      double rel_load = (max_signal > 1e-9) ? signals[r] / max_signal : 1.0;
      
      if(latency_critical) {
        // LATENCY CRITICAL: Boost high-load routers, minimal throttle on others
        target_scale = base_scale + rel_load * (_high_scale - base_scale);
      } else {
        // NORMAL: Proportional scaling based on load
        // High load (>avg) -> higher freq, Low load (<avg) -> lower freq  
        if(signals[r] > avg_signal * 1.2) {
          // Hot router: keep fast
          target_scale = base_scale + 0.5 * (_high_scale - base_scale);
        } else if(signals[r] < avg_signal * 0.5) {
          // Cold router: can throttle more
          target_scale = _low_scale + 0.5 * (base_scale - _low_scale);
        } else {
          // Normal router: use base scale
          target_scale = base_scale;
        }
      }
      
      // Apply smoothing per-router to avoid abrupt changes
      if(_router_scales.size() <= r) {
        _router_scales.resize(n_routers, _high_scale);
      }
      target_scale = 0.6 * _router_scales[r] + 0.4 * target_scale;
      target_scale = std::max(_low_scale, std::min(_high_scale, target_scale));
      net.SetRouterSpeed(static_cast<int>(r), target_scale);
      _router_scales[r] = target_scale;
    }
    
    _current_scale = base_scale;
    _last_change_epoch = epoch;
    
    std::cout << "HW_REACTIVE_DIFF: epoch=" << epoch 
              << " max_signal=" << max_signal
              << " base_scale=" << base_scale
              << " headroom=" << pwr.headroom << std::endl;
    
  } else {
    // === GLOBAL CONTROL (single domain) ===
    auto latency_signal = [&](const PowerTelemetry &pt)->double {
      if(!pt.class_latency_p99.empty() && _control_class >= 0 &&
         _control_class < static_cast<int>(pt.class_latency_p99.size())) {
        return pt.class_latency_p99[_control_class];
      }
      if(!pt.class_latency_p99.empty()) return pt.class_latency_p99[0];
      return 0.0;
    };

    auto pick_scale = [&](double val, double headroom)->double {
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
      sig = latency_signal(pwr);
    }
    
    double target = pick_scale(sig, pwr.headroom);
    if(target < 0.0) target = _current_scale;
    
    // Power cap enforcement
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      target = target * (1.0 - std::min(over_ratio * 2.0, 0.5));
      target = std::max(target, _low_scale);
    }
    
    target = std::max(_low_scale, std::min(_high_scale, target));
    
    std::cout << "HW_REACTIVE: epoch=" << epoch << " signal=" << sig 
              << " target=" << target << " headroom=" << pwr.headroom << std::endl;
    
    net.SetDomainSpeed(0, target);
    _current_scale = target;
    _last_change_epoch = epoch;
  }
}

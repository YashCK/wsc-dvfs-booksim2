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
    
    // 2. Compute priority scores (higher load = higher priority = higher freq)
    std::vector<double> priorities(n_routers);
    double sum_priorities = 0.0;
    for(size_t r = 0; r < n_routers; ++r) {
      if(max_signal > 1e-9) {
        priorities[r] = signals[r] / max_signal;
      } else {
        priorities[r] = 1.0; // all idle, treat equally
      }
      // Apply sqrt to make distribution less extreme
      priorities[r] = std::pow(priorities[r], 0.5);
      sum_priorities += priorities[r];
    }
    
    // 3. Compute power budget allocation
    double base_scale = _high_scale;
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      base_scale = _current_scale * (1.0 - std::min(over_ratio * 1.5, 0.4));
      base_scale = std::max(base_scale, _low_scale);
    }
    
    // 4. Allocate frequencies based on priority
    double avg_priority = sum_priorities / n_routers;
    
    for(size_t r = 0; r < n_routers; ++r) {
      double target_scale;
      if(priorities[r] >= avg_priority) {
        double t = (avg_priority < 1.0) ? 
                   (priorities[r] - avg_priority) / (1.0 - avg_priority) : 0.0;
        target_scale = base_scale + t * (_high_scale - base_scale);
      } else {
        double t = (avg_priority > 0.0) ? priorities[r] / avg_priority : 0.0;
        target_scale = _low_scale + t * (base_scale - _low_scale);
      }
      target_scale = std::max(_low_scale, std::min(_high_scale, target_scale));
      net.SetRouterSpeed(static_cast<int>(r), target_scale);
    }
    
    _current_scale = base_scale;
    _last_change_epoch = epoch;
    
    std::cout << "HW_REACTIVE_DIFF: epoch=" << epoch 
              << " max_signal=" << max_signal
              << " base_scale=" << base_scale
              << " headroom=" << pwr.headroom << std::endl;
    
  } else {
    // === GLOBAL CONTROL (single domain) WITH CLASS AWARENESS ===
    auto latency_signal = [&](const PowerTelemetry &pt)->double {
      if(!pt.class_latency_p99.empty() && _control_class >= 0 &&
         _control_class < static_cast<int>(pt.class_latency_p99.size())) {
        return pt.class_latency_p99[_control_class];
      }
      if(!pt.class_latency_p99.empty()) return pt.class_latency_p99[0];
      return 0.0;
    };

    // Get control class latency for SLO-aware decisions
    double control_latency = latency_signal(pwr);
    
    // PROACTIVE SLO PROTECTION: Be more cautious as we approach SLO
    bool latency_ok = (_control_slo <= 0.0) || (control_latency <= _control_slo);
    bool latency_comfortable = (_control_slo <= 0.0) || (control_latency <= _control_slo * 0.7);
    double latency_slack = (_control_slo > 0.0 && control_latency > 0.0) 
                          ? (_control_slo - control_latency) / _control_slo : 1.0;

    auto pick_scale = [&](double val, double headroom)->double {
      // CLASS-AWARE: If control class latency exceeds SLO, force high scale
      if(!latency_ok) {
        return _high_scale;
      }
      // If approaching SLO (< 70% slack), boost frequency to maintain headroom
      if(!latency_comfortable && _control_slo > 0.0) {
        // Interpolate between current and high based on how close to SLO
        double boost_factor = 1.0 - latency_slack;  // 0.0 at 100% slack, 0.3 at 70% slack
        return _current_scale + boost_factor * (_high_scale - _current_scale);
      }
      if((lower_str == "latency") && (_control_slo > 0.0) && (val > _control_slo)) {
        return _high_scale;
      }
      if(val >= _high_thresh) return _high_scale;
      if(val <= _low_thresh) {
        // Only throttle aggressively if latency has good slack
        if(latency_comfortable) {
          if((_headroom_margin > 0.0) && (headroom <= _headroom_margin)) {
            return -1.0;
          }
          // Throttle proportionally to latency slack
          double throttle_range = _high_scale - _low_scale;
          double target_scale = _low_scale + latency_slack * throttle_range * 0.5;
          return std::max(_low_scale, target_scale);
        }
        return -1.0;  // Keep current if latency not comfortable
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
      sig = control_latency;
    }
    
    double target = pick_scale(sig, pwr.headroom);
    if(target < 0.0) target = _current_scale;
    
    // Power cap enforcement - but gentler if control latency is high
    if(pwr.headroom < 0.0 && pwr.power_cap > 0.0) {
      double over_ratio = -pwr.headroom / pwr.power_cap;
      double throttle_factor = 2.0;
      // If control class is struggling or approaching SLO, reduce throttling intensity
      if(!latency_ok) {
        throttle_factor = 0.5;  // Minimal throttling when exceeding SLO
      } else if(!latency_comfortable) {
        throttle_factor = 1.0;  // Gentler throttling when approaching SLO
      }
      target = target * (1.0 - std::min(over_ratio * throttle_factor, 0.3));
      target = std::max(target, _low_scale);
    }
    
    target = std::max(_low_scale, std::min(_high_scale, target));
    
    std::cout << "HW_REACTIVE: epoch=" << epoch << " signal=" << sig 
              << " target=" << target << " ctrl_lat=" << control_latency
              << " ctrl_slo=" << _control_slo << " lat_ok=" << latency_ok
              << " lat_comf=" << latency_comfortable << " slack=" << latency_slack
              << " headroom=" << pwr.headroom << std::endl;
    
    net.SetDomainSpeed(0, target);
    _current_scale = target;
    _last_change_epoch = epoch;
  }
}
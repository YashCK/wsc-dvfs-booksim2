#include "policy/dvfs_policy.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <numeric>

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
      return _high_scale; // SLO violation: force max
    }
    if(val >= _high_thresh) return _high_scale;
    if(val <= _low_thresh) {
      if((_headroom_margin > 0.0) && (headroom <= _headroom_margin)) {
        return -1.0; // avoid throttling when near cap
      }
      return _low_scale;
    }
    return -1.0; // no change
  };

  auto clamp_scale = [&](double s)->double {
    if(s < 0.0) return s;
    if(s > _high_scale) return _high_scale;
    if(s < _low_scale) return _low_scale;
    return s;
  };

  if(_per_router) {
    const size_t n = pwr.router_occupancy.size();
    for(size_t r = 0; r < n; ++r) {
      double sig = 0.0;
      if(lower_str == "queue") {
        sig = (r < pwr.router_occupancy.size()) ? pwr.router_occupancy[r] : 0.0;
      } else if(lower_str == "inj") {
        sig = (r < pwr.router_injection_rate.size()) ? pwr.router_injection_rate[r] : 0.0;
      } else if(lower_str == "stall") {
        sig = (r < pwr.router_stall_rate.size()) ? pwr.router_stall_rate[r] : 0.0;
      } else if(lower_str == "latency") {
        sig = latency_signal(pwr);
      }
      double target = clamp_scale(pick_scale(sig, pwr.headroom));
      if(target > 0.0) {
        net.SetRouterSpeed(static_cast<int>(r), target);
        _last_change_epoch = epoch;
      }
    }
  } else {
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
    std::cout << "HW_REACTIVE: epoch=" << epoch << " signal=" << sig 
              << " thresholds=[" << _low_thresh << "," << _high_thresh << "] headroom=" << pwr.headroom << std::endl;
    double target = clamp_scale(pick_scale(sig, pwr.headroom));
    if(target > 0.0) {
      std::cout << "HW_REACTIVE: Changing domain 0 frequency to " << target << std::endl;
      net.SetDomainSpeed(0, target); // domains map handled by TrafficManager
      _last_change_epoch = epoch;
    } else {
      std::cout << "HW_REACTIVE: No change needed (sig in dead zone)" << std::endl;
    }
  }
}

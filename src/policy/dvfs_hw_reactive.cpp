#include "policy/dvfs_policy.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace {
double norm_signal(const std::vector<double> &v) {
  if(v.empty()) return 0.0;
  double sum = 0.0;
  for(double x : v) sum += x;
  return sum / static_cast<double>(v.size());
}
} // namespace

void HWReactiveDVFSPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net,
                                  int epoch) {
  if((_last_change_epoch >= 0) && (_hysteresis > 0) &&
     ((epoch - _last_change_epoch) < _hysteresis)) {
    return;
  }
  auto lower_str = _signal;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                 [](unsigned char c){ return std::tolower(c); });

  auto pick_scale = [&](double val)->double {
    if(val >= _high_thresh) return _high_scale;
    if(val <= _low_thresh) return _low_scale;
    return -1.0; // no change
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
      }
      double target = pick_scale(sig);
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
    }
    double target = pick_scale(sig);
    if(target > 0.0) {
      net.SetDomainSpeed(0, target); // domains map handled by TrafficManager
      _last_change_epoch = epoch;
    }
  }
}

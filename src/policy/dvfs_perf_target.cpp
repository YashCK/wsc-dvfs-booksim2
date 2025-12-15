#include "policy/dvfs_perf_target.hpp"

#include <algorithm>
#include <cctype>
#include <numeric>
#include <iostream>

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

  auto lower = _metric;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c){ return std::tolower(c); });

  auto measure_latency = [&](const PowerTelemetry &pt)->double {
    if(!pt.class_latency_p99.empty() && _class >= 0 &&
       _class < static_cast<int>(pt.class_latency_p99.size())) {
      return pt.class_latency_p99[_class];
    }
    if(!pt.class_latency_p99.empty()) return pt.class_latency_p99[0];
    return 0.0;
  };
  auto measure_throughput = [&](const PowerTelemetry &pt)->double {
    if(!pt.class_throughput.empty() && _class >= 0 &&
       _class < static_cast<int>(pt.class_throughput.size())) {
      return pt.class_throughput[_class];
    }
    if(!pt.class_throughput.empty()) return pt.class_throughput[0];
    return 0.0;
  };

  auto decide = [&](double meas, double old_scale)->double {
    if(lower == "latency") {
      // For latency: if meas > target (too slow), we want to speed up (positive delta)
      // So we need: err positive → delta positive → speed up
      // This requires NEGATIVE Kp: delta = (-Kp) * (+err) = negative × positive = negative
      // Wait, that's wrong. Let me think again:
      // meas > target (latency too high) → err = meas - target > 0 → want delta > 0 (speed up)
      // So: delta = Kp * err, with POSITIVE Kp
      // BUT: speed up means INCREASE frequency scale
      // Actually the issue is that positive Kp makes it slow down!
      // When latency < target (fast), err < 0, delta < 0, scale decreases → slows down ✓
      // When latency > target (slow), err > 0, delta > 0, scale increases → speeds up ✓
      // So positive Kp SHOULD work... unless the initial latency is BELOW target!
      //
      // AH! The problem: target=70, initial latency ~19
      // err = 19 - 70 = -51 (negative)
      // delta = 0.03 × (-51) = -1.53 (negative, but normalized)
      // delta = 0.03 × (-51/70) = -0.022 (negative)
      // scale = 1.0 + (-0.022) = 0.978 → slows down
      // This causes latency to increase, which is BACKWARDS!
      //
      // We want: if latency < target, do NOTHING or speed up slightly for power savings
      // if latency > target, speed up to reduce latency
      //
      // The right interpretation: target is a MAXIMUM latency constraint
      // Use NEGATIVE Kp so that we only speed up when needed:
      // latency < target: err < 0, delta = (-Kp) × err = positive → slow down for power ✓
      // latency > target: err > 0, delta = (-Kp) × err = negative → speed up ✓
      // Wait no, that's also wrong!
      //
      // Let me be very explicit:
      // - old_scale + delta = new_scale
      // - If delta > 0, we increase frequency (speed up)
      // - If delta < 0, we decrease frequency (slow down)
      //
      // When latency > target (performance is bad):
      //   err = meas - target > 0
      //   We want delta > 0 (speed up)
      //   So: delta = Kp × err requires Kp > 0
      //
      // When latency < target (performance is good):
      //   err = meas - target < 0
      //   We want delta < 0 (slow down to save power)
      //   So: delta = Kp × err requires Kp > 0
      //
      // So positive Kp is CORRECT! The bug is that the target is set too HIGH.
      // Initial latency ~19, target 70 → system tries to slow down to reach 70!
      //
      // The fix: interpret target as a MAXIMUM (SLO), not a setpoint
      // Only act if latency EXCEEDS target:
      double err = std::max(0.0, meas - _target); // only positive when violating SLO
      double delta = _kp * (err / (_target > 1e-9 ? _target : 1.0));
      return clamp(old_scale + delta);
    } else { // throughput
      double err = _target - meas; // positive when too slow (want more throughput)
      double delta = _kp * (err / (_target > 1e-9 ? _target : 1.0));
      return clamp(old_scale + delta);
    }
  };

  if(_per_router) {
    size_t n = pwr.router_power.size();
    _EnsureSize(n);
    for(size_t r = 0; r < n; ++r) {
      double meas = (lower == "latency") ? measure_latency(pwr) : measure_throughput(pwr);
      double new_scale = decide(meas, _prev_scale[r]);
      if(headroom_ok(new_scale, _prev_scale[r])) {
        net.SetRouterSpeed(static_cast<int>(r), new_scale);
        _prev_scale[r] = new_scale;
      }
    }
  } else {
    _EnsureSize(1);
    double meas = (lower == "latency") ? measure_latency(pwr) : measure_throughput(pwr);
    double new_scale = decide(meas, _prev_scale[0]);
    if(headroom_ok(new_scale, _prev_scale[0])) {
      net.SetDomainSpeed(0, new_scale);
      _prev_scale[0] = new_scale;
    }
  }
}


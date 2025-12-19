#ifndef _DVFS_PERF_TARGET_HPP_
#define _DVFS_PERF_TARGET_HPP_

#include <string>
#include <vector>
#include "policy/dvfs_policy.hpp"

class PerfTargetDVFSPolicy : public DVFSPolicy {
public:
  PerfTargetDVFSPolicy(std::string metric, double target_value, int target_class,
                       double kp, double min_scale, double max_scale,
                       bool per_router, double headroom_margin);
  void Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) override;
  std::string GetType() const override { return "perf_target"; }

private:
  std::string _metric; // "latency" or "throughput"
  double _target;
  int _class;
  double _kp;
  double _min_scale;
  double _max_scale;
  bool _per_router;
  double _headroom_margin;
  std::vector<double> _prev_scale;
  std::vector<double> _integral_err;  // For PI control
  void _EnsureSize(size_t n);
};

#endif

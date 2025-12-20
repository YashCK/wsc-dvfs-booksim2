#ifndef _DVFS_QUEUE_PID_HPP_
#define _DVFS_QUEUE_PID_HPP_

#include <vector>
#include <string>
#include "policy/dvfs_policy.hpp"

class QueuePIDPolicy : public DVFSPolicy {
public:
  QueuePIDPolicy(double target, double kp, double ki, double kd,
                 double min_scale, double max_scale,
                 bool per_router, double headroom_margin,
                 int control_class = 0, double control_slo = 0.0);
  void Update(const PowerTelemetry &pwr, NetworkControl &net, int epoch) override;
  std::string GetType() const override { return "queue_pid"; }

private:
  double _target;
  double _kp, _ki, _kd;
  double _min_scale, _max_scale;
  bool _per_router;
  double _headroom_margin;
  int _control_class;       // Class to prioritize (control class)
  double _control_slo;      // Target P99 latency for control class

  std::vector<double> _int_err;
  std::vector<double> _prev_err;
  std::vector<double> _prev_scale;

  void _EnsureSize(size_t n);
};

#endif

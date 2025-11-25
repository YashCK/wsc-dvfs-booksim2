#include "policy/dvfs_policy.hpp"

void UniformDVFSPolicy::Update(const PowerTelemetry &,
                               NetworkControl &net, int) {
  net.SetDomainSpeed(0, _target);
}

void BudgetDVFSPolicy::Update(const PowerTelemetry &pwr, NetworkControl &net,
                              int) {
  if (_cap <= 0.0) {
    net.SetDomainSpeed(0, _max_scale);
    return;
  }
  double scale = _max_scale;
  if (pwr.total_power > _cap) {
    double over = pwr.total_power - _cap;
    double frac = over / _cap;
    scale = _max_scale - frac;
    if (scale < _min_scale) scale = _min_scale;
  }
  net.SetDomainSpeed(0, scale);
}


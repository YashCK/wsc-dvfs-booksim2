// Interfaces for DVFS control policies.
#ifndef _DVFS_POLICY_HPP_
#define _DVFS_POLICY_HPP_

#include <functional>

#include "policy/telemetry.hpp"

class NetworkControl {
public:
  virtual ~NetworkControl() {}
  virtual void SetRouterSpeed(int router_id, double freq_scale) = 0;
  virtual void SetDomainSpeed(int domain_id, double freq_scale) = 0;
};

class DVFSPolicy {
public:
  virtual ~DVFSPolicy() {}
  virtual void Update(const PowerTelemetry &pwr, NetworkControl &net,
                      int epoch) = 0;
  virtual std::string GetType() const { return "dvfs_policy"; }
};

class StaticDVFSPolicy : public DVFSPolicy {
public:
  explicit StaticDVFSPolicy(double target = 1.0) : _target(target) {}
  void Update(const PowerTelemetry & /*pwr*/, NetworkControl &net,
              int /*epoch*/) override;
  std::string GetType() const override { return "static"; }

private:
  double _target;
};

class UniformDVFSPolicy : public DVFSPolicy {
public:
  UniformDVFSPolicy(double power_cap, double min_scale = 0.5,
                   double max_scale = 1.0)
      : _cap(power_cap), _min_scale(min_scale), _max_scale(max_scale) {}
  void Update(const PowerTelemetry &pwr, NetworkControl &net, int) override;
  std::string GetType() const override { return "uniform"; }

private:
  double _cap;
  double _min_scale;
  double _max_scale;
};

class CustomDVFSPolicy : public DVFSPolicy {
public:
  explicit CustomDVFSPolicy(
      std::function<void(const PowerTelemetry &, NetworkControl &, int)> fn)
      : _fn(fn) {}
  void Update(const PowerTelemetry &pwr, NetworkControl &net,
              int epoch) override {
    if (_fn) _fn(pwr, net, epoch);
  }
  std::string GetType() const override { return "custom"; }

private:
  std::function<void(const PowerTelemetry &, NetworkControl &, int)> _fn;
};

class HWReactiveDVFSPolicy : public DVFSPolicy {
public:
  HWReactiveDVFSPolicy(double high_thresh, double low_thresh,
                       double high_scale, double low_scale,
                       int hysteresis_epochs, bool per_router,
                       std::string signal, int control_class,
                       double control_slo_cycles, double headroom_margin)
      : _high_thresh(high_thresh), _low_thresh(low_thresh),
        _high_scale(high_scale), _low_scale(low_scale),
        _hysteresis(hysteresis_epochs), _per_router(per_router),
        _signal(std::move(signal)), _control_class(control_class),
        _control_slo(control_slo_cycles), _headroom_margin(headroom_margin),
        _last_change_epoch(-1), _current_scale(1.0) {}
  void Update(const PowerTelemetry &pwr, NetworkControl &net,
              int epoch) override;
  std::string GetType() const override { return "hw_reactive"; }

private:
  double _high_thresh;
  double _low_thresh;
  double _high_scale;
  double _low_scale;
  int _hysteresis;
  bool _per_router;
  std::string _signal;
  int _control_class;
  double _control_slo;
  double _headroom_margin;
  int _last_change_epoch;
  double _current_scale;  // Track current frequency scale
};

#endif

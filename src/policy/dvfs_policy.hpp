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

class UniformDVFSPolicy : public DVFSPolicy {
public:
  explicit UniformDVFSPolicy(double target = 1.0) : _target(target) {}
  void Update(const PowerTelemetry & /*pwr*/, NetworkControl &net,
              int /*epoch*/) override;
  std::string GetType() const override { return "uniform"; }

private:
  double _target;
};

class BudgetDVFSPolicy : public DVFSPolicy {
public:
  BudgetDVFSPolicy(double power_cap, double min_scale = 0.5,
                   double max_scale = 1.0)
      : _cap(power_cap), _min_scale(min_scale), _max_scale(max_scale) {}
  void Update(const PowerTelemetry &pwr, NetworkControl &net, int) override;
  std::string GetType() const override { return "budget"; }

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

#endif

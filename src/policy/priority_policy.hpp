// Interfaces for adjusting flit/VC priority.
#ifndef _PRIORITY_POLICY_HPP_
#define _PRIORITY_POLICY_HPP_

#include <functional>

#include "class_config.hpp"
#include "flit.hpp"
#include "policy/telemetry.hpp"
#include "vc.hpp"

class PriorityPolicy {
public:
  virtual ~PriorityPolicy() {}
  virtual void OnEnqueue(Flit &f, const ClassConfig &cfg,
                         const PolicyTelemetry &t) = 0;
  virtual void OnCycle(VC &, PolicyTelemetry &) {}
};

class StaticPriorityPolicy : public PriorityPolicy {
public:
  void OnEnqueue(Flit &f, const ClassConfig &cfg,
                 const PolicyTelemetry &) override {
    f.pri = cfg.base_priority;
  }
};

class DeadlineBoostPolicy : public PriorityPolicy {
public:
  void OnEnqueue(Flit &f, const ClassConfig &cfg,
                 const PolicyTelemetry &t) override {
    // If we have per-class P99 latency info and an SLO, raise priority
    // proportionally to slack deficit.
    f.pri = cfg.base_priority;
    if (cfg.slo_cycles > 0 && f.cl < (int)t.class_latency_p99.size()) {
      double p99 = t.class_latency_p99[f.cl];
      double slack = static_cast<double>(cfg.slo_cycles) - p99;
      if (slack < 0) {
        // Boost when over SLO.
        f.pri = cfg.base_priority + static_cast<int>(cfg.priority_boost);
      }
    }
  }
};

class CustomPriorityPolicy : public PriorityPolicy {
public:
  explicit CustomPriorityPolicy(
      std::function<void(Flit &, const ClassConfig &, const PolicyTelemetry &)>
          fn,
      std::function<void(VC &, PolicyTelemetry &)> on_cycle = nullptr)
      : _fn(fn), _on_cycle(on_cycle) {}

  void OnEnqueue(Flit &f, const ClassConfig &cfg,
                 const PolicyTelemetry &t) override {
    if (_fn) {
      _fn(f, cfg, t);
    }
  }
  void OnCycle(VC &vc, PolicyTelemetry &t) override {
    if (_on_cycle) {
      _on_cycle(vc, t);
    }
  }

private:
  std::function<void(Flit &, const ClassConfig &, const PolicyTelemetry &)> _fn;
  std::function<void(VC &, PolicyTelemetry &)> _on_cycle;
};

#endif

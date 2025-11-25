// Lightweight telemetry structs used by policy hooks.
#ifndef _POLICY_TELEMETRY_HPP_
#define _POLICY_TELEMETRY_HPP_

#include <vector>

struct PolicyTelemetry {
  long long time = 0;
  // Optional: per-node queue occupancy or HOL age
  std::vector<int> queue_occupancy;
  std::vector<int> hol_age;
  // Optional per-class latency percentiles (e.g., P99)
  std::vector<double> class_latency_p99;
};

struct PowerTelemetry {
  double total_power = 0.0;
  // Optional per-router power estimates
  std::vector<double> router_power;
};

#endif

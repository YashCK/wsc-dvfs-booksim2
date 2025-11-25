// Basic per-class metadata used by policy hooks.
#ifndef _CLASS_CONFIG_HPP_
#define _CLASS_CONFIG_HPP_

#include <vector>
#include "config_utils.hpp"

struct ClassConfig {
  int id;
  int base_priority;
  // Target P99 or deadline in cycles; -1 means unused.
  int slo_cycles;
  // Optional priority boost factor for urgency-aware policies.
  double priority_boost;
  bool measure_slo;

  ClassConfig()
      : id(0),
        base_priority(0),
        slo_cycles(-1),
        priority_boost(1.0),
        measure_slo(false) {}
};

std::vector<ClassConfig> ParseClassConfig(const Configuration &config,
                                          int classes);

#endif

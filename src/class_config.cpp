// Utilities for building per-class configuration metadata from BookSim config.

#include "class_config.hpp"

#include <cassert>

std::vector<ClassConfig> ParseClassConfig(const Configuration &config,
                                          int classes) {
  std::vector<int> priorities = config.GetIntArray("class_priority");
  if (priorities.empty()) {
    priorities.push_back(config.GetInt("class_priority"));
  }
  priorities.resize(classes, priorities.back());

  std::vector<int> slo_cycles = config.GetIntArray("class_slo");
  if (slo_cycles.empty()) {
    slo_cycles.push_back(config.GetInt("class_slo"));
  }
  slo_cycles.resize(classes, slo_cycles.back());

  std::vector<double> boost = config.GetFloatArray("class_priority_boost");
  if (boost.empty()) {
    boost.push_back(config.GetFloat("class_priority_boost"));
  }
  boost.resize(classes, boost.back());

  std::vector<ClassConfig> cfg(classes);
  for (int i = 0; i < classes; ++i) {
    cfg[i].id = i;
    cfg[i].base_priority = priorities[i];
    cfg[i].slo_cycles = slo_cycles[i];
    cfg[i].priority_boost = boost[i];
    cfg[i].measure_slo = (cfg[i].slo_cycles > 0);
  }
  return cfg;
}


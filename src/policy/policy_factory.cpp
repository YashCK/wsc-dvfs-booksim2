#include "policy/policy_factory.hpp"

#include <string>

using std::string;

std::unique_ptr<ClassAssigner> MakeClassAssigner(const Configuration &config,
                                                 int classes) {
  string mode = config.GetStr("class_assigner");
  if (mode == "static" || mode.empty()) {
    return std::unique_ptr<ClassAssigner>(new StaticClassAssigner(classes));
  }
  if (mode == "custom") {
    return std::unique_ptr<ClassAssigner>(new StaticClassAssigner(classes));
  }
  return std::unique_ptr<ClassAssigner>(new StaticClassAssigner(classes));
}

std::unique_ptr<PriorityPolicy> MakePriorityPolicy(
    const Configuration &config) {
  string mode = config.GetStr("priority_policy");
  if (mode == "deadline_boost") {
    return std::unique_ptr<PriorityPolicy>(new DeadlineBoostPolicy());
  }
  if (mode == "static_class" || mode.empty()) {
    return std::unique_ptr<PriorityPolicy>(new StaticPriorityPolicy());
  }
  if (mode == "custom") {
    return std::unique_ptr<PriorityPolicy>(new StaticPriorityPolicy());
  }
  return std::unique_ptr<PriorityPolicy>(new StaticPriorityPolicy());
}

std::unique_ptr<DVFSPolicy> MakeDVFSPolicy(const Configuration &config) {
  string mode = config.GetStr("dvfs_policy");
  double cap = config.GetFloat("power_cap");
  vector<double> min_scales = config.GetFloatArray("dvfs_min_scale");
  if(min_scales.empty()) min_scales.push_back(config.GetFloat("dvfs_min_scale"));
  vector<double> max_scales = config.GetFloatArray("dvfs_max_scale");
  if(max_scales.empty()) max_scales.push_back(config.GetFloat("dvfs_max_scale"));
  double min_scale = min_scales.front();
  double max_scale = max_scales.front();
  if (mode == "budget") {
    return std::unique_ptr<DVFSPolicy>(new BudgetDVFSPolicy(cap, min_scale, max_scale));
  }
  if (mode == "hw_reactive") {
    double hi_t = config.GetFloat("hw_reactive_high_thresh");
    double lo_t = config.GetFloat("hw_reactive_low_thresh");
    double hi_s = config.GetFloat("hw_reactive_high_scale");
    double lo_s = config.GetFloat("hw_reactive_low_scale");
    int hyst = config.GetInt("hw_reactive_hysteresis_epochs");
    bool per_router = config.GetInt("hw_reactive_per_router") > 0;
    string signal = config.GetStr("hw_reactive_signal");
    return std::unique_ptr<DVFSPolicy>(
        new HWReactiveDVFSPolicy(hi_t, lo_t, hi_s, lo_s, hyst, per_router, signal));
  }
  if (mode == "uniform" || mode.empty()) {
    return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
  }
  if (mode == "custom") {
    return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
  }
  return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
}

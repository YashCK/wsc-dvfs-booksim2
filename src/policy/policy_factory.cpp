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
  if (mode == "budget") {
    return std::unique_ptr<DVFSPolicy>(new BudgetDVFSPolicy(cap));
  }
  if (mode == "uniform" || mode.empty()) {
    return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
  }
  if (mode == "custom") {
    return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
  }
  return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(1.0));
}

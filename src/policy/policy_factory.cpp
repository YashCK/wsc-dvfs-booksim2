#include "policy/policy_factory.hpp"

#include <string>

using std::string;

#include "policy/dvfs_queue_pid.hpp"
#include "policy/dvfs_perf_target.hpp"

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
  if (mode == "uniform") {
    return std::unique_ptr<DVFSPolicy>(new UniformDVFSPolicy(cap, min_scale, max_scale));
  }
  if (mode == "hw_reactive") {
    double hi_t = config.GetFloat("hw_reactive_high_thresh");
    double lo_t = config.GetFloat("hw_reactive_low_thresh");
    double hi_s = config.GetFloat("hw_reactive_high_scale");
    double lo_s = config.GetFloat("hw_reactive_low_scale");
    int hyst = config.GetInt("hw_reactive_hysteresis_epochs");
    bool per_router = config.GetInt("hw_reactive_per_router") > 0;
    string signal = config.GetStr("hw_reactive_signal");
    int control_class = config.GetInt("control_class_id");
    double control_slo = config.GetFloat("control_slo_cycles");
    double headroom_margin = config.GetFloat("hw_reactive_headroom_margin");
    return std::unique_ptr<DVFSPolicy>(
        new HWReactiveDVFSPolicy(hi_t, lo_t, hi_s, lo_s, hyst, per_router, signal,
                                 control_class, control_slo, headroom_margin));
  }
  if (mode == "queue_pid") {
    double target = config.GetFloat("queue_pid_target");
    double kp = config.GetFloat("queue_pid_kp");
    double ki = config.GetFloat("queue_pid_ki");
    double kd = config.GetFloat("queue_pid_kd");
    bool per_router = config.GetInt("queue_pid_per_router") > 0;
    double headroom_margin = config.GetFloat("queue_pid_headroom_margin");
    int control_class = config.GetInt("control_class_id");
    double control_slo = config.GetFloat("control_slo_cycles");
    return std::unique_ptr<DVFSPolicy>(
        new QueuePIDPolicy(target, kp, ki, kd, min_scale, max_scale, per_router, headroom_margin,
                          control_class, control_slo));
  }
  if (mode == "perf_target") {
    string metric = config.GetStr("perf_target_metric");
    double target = config.GetFloat("perf_target_value");
    int tclass = config.GetInt("perf_target_class");
    double kp = config.GetFloat("perf_target_kp");
    bool per_router = config.GetInt("perf_target_per_router") > 0;
    double headroom_margin = config.GetFloat("perf_target_headroom_margin");
    return std::unique_ptr<DVFSPolicy>(
        new PerfTargetDVFSPolicy(metric, target, tclass, kp, min_scale, max_scale, per_router, headroom_margin));
  }
  if (mode == "static" || mode.empty()) {
    return std::unique_ptr<DVFSPolicy>(new StaticDVFSPolicy(1.0));
  }
  if (mode == "custom") {
    return std::unique_ptr<DVFSPolicy>(new StaticDVFSPolicy(1.0));
  }
  return std::unique_ptr<DVFSPolicy>(new StaticDVFSPolicy(1.0));
}

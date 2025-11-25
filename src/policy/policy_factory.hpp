// Factory helpers for policy objects driven by config.
#ifndef _POLICY_FACTORY_HPP_
#define _POLICY_FACTORY_HPP_

#include <memory>

#include "booksim_config.hpp"
#include "class_config.hpp"
#include "policy/class_assigner.hpp"
#include "policy/dvfs_policy.hpp"
#include "policy/priority_policy.hpp"

std::unique_ptr<ClassAssigner> MakeClassAssigner(const Configuration &config,
                                                 int classes);
std::unique_ptr<PriorityPolicy> MakePriorityPolicy(
    const Configuration &config);
std::unique_ptr<DVFSPolicy> MakeDVFSPolicy(const Configuration &config);

#endif

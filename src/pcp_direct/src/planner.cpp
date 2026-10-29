// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_direct/planner.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <yaml-cpp/yaml.h>

namespace pcp_direct
{

PlannerSettings loadMoveItOmplSettings(const std::string & yaml_path, const std::string & group)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node g = root[group];
  if (!g) {
    throw std::runtime_error(yaml_path + ": no group '" + group + "'");
  }
  PlannerSettings s;
  s.planner_config = g["default_planner_config"].as<std::string>("");
  const YAML::Node cfg = root["planner_configs"][s.planner_config];
  if (s.planner_config.empty() || !cfg) {
    throw std::runtime_error(yaml_path + ": default_planner_config of '" + group + "' not found");
  }
  s.planner_type = cfg["type"].as<std::string>("");
  if (s.planner_type != "geometric::RRTConnect") {
    throw std::runtime_error(
      "the direct pipeline implements RRTConnect only, MoveIt's default is " + s.planner_type);
  }
  if (cfg["range"]) {
    s.range = cfg["range"].as<double>();
    s.range_set_in_yaml = true;
  }
  if (g["longest_valid_segment_fraction"]) {
    s.longest_valid_segment_fraction = g["longest_valid_segment_fraction"].as<double>();
    s.lvsf_set_in_yaml = true;
  }
  return s;
}

double JointSpaceL1::distance(const ompl::base::State * a, const ompl::base::State * b) const
{
  const double * x = a->as<StateType>()->values;
  const double * y = b->as<StateType>()->values;
  double d = 0.0;
  for (unsigned int i = 0; i < dimension_; ++i) {
    d += std::abs(x[i] - y[i]);
  }
  return d;
}

double JointSpaceL1::getMaximumExtent() const
{
  double e = 0.0;
  for (unsigned int i = 0; i < dimension_; ++i) {
    e += bounds_.high[i] - bounds_.low[i];
  }
  return e;
}

DirectPlanner::DirectPlanner(CollisionChecker & checker, const PlannerSettings & settings)
: checker_(checker), settings_(settings)
{
  const std::size_t n = checker_.jointNames().size();
  space_ = std::make_shared<JointSpaceL1>(static_cast<unsigned int>(n));
  ompl::base::RealVectorBounds bounds(static_cast<unsigned int>(n));
  for (std::size_t j = 0; j < n; ++j) {
    bounds.setLow(static_cast<unsigned int>(j), checker_.lowerLimits()[j]);
    bounds.setHigh(static_cast<unsigned int>(j), checker_.upperLimits()[j]);
    space_->setDimensionName(static_cast<unsigned int>(j), checker_.jointNames()[j]);
  }
  space_->setBounds(bounds);
  space_->setLongestValidSegmentFraction(settings_.longest_valid_segment_fraction);
  space_->setup();
  effective_range_ =
    settings_.range > 0.0 ? settings_.range : 0.2 * space_->getMaximumExtent();  // OMPL default
}

double DirectPlanner::maximumExtent() const { return space_->getMaximumExtent(); }
double DirectPlanner::longestValidSegment() const { return space_->getLongestValidSegmentLength(); }
double DirectPlanner::effectiveRange() const { return effective_range_; }

PlanResult DirectPlanner::plan(
  const std::vector<double> & start, const std::vector<double> & goal,
  const ompl::base::ConstraintPtr & constraint)
{
  const std::size_t n = checker_.jointNames().size();
  if (start.size() != n || goal.size() != n) {
    throw std::invalid_argument("start/goal size does not match the joint count");
  }
  // A fresh SimpleSetup per request, like MoveIt's planning context per request. With a path
  // constraint, plan in OMPL's ProjectedStateSpace over the same JointSpaceL1 (MoveIt's
  // enforce_constrained_state_space does the same with OMPL's default delta/lambda/tolerance).
  ompl::base::StateSpacePtr space = space_;
  ompl::base::SpaceInformationPtr si;
  if (constraint) {
    auto css = std::make_shared<ompl::base::ProjectedStateSpace>(space_, constraint);
    space = css;
    si = std::make_shared<ompl::base::ConstrainedSpaceInformation>(css);
  } else {
    si = std::make_shared<ompl::base::SpaceInformation>(space_);
  }
  const bool constrained = static_cast<bool>(constraint);
  auto values = [constrained](const ompl::base::State * state) -> const double * {
    const ompl::base::State * raw =
      constrained ? state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState() : state;
    return raw->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  };
  ompl::geometric::SimpleSetup ss(si);
  CollisionChecker * checker = &checker_;
  ss.setStateValidityChecker([si, checker, values](const ompl::base::State * state) {
    if (!si->satisfiesBounds(state)) {
      return false;
    }
    return checker->isValid(values(state));
  });
  si->setStateValidityCheckingResolution(settings_.longest_valid_segment_fraction);
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  if (settings_.range > 0.0) {
    planner->setRange(settings_.range);
  }
  ss.setPlanner(planner);
  ompl::base::ScopedState<> s(space);
  ompl::base::ScopedState<> g(space);
  if (constrained) {
    s->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(
      Eigen::Map<const Eigen::VectorXd>(start.data(), static_cast<Eigen::Index>(n)));
    g->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(
      Eigen::Map<const Eigen::VectorXd>(goal.data(), static_cast<Eigen::Index>(n)));
  } else {
    for (std::size_t j = 0; j < n; ++j) {
      s[j] = start[j];
      g[j] = goal[j];
    }
  }
  ss.setStartAndGoalStates(s, g);
  ss.setup();

  PlanResult r;
  checker_.resetStatistics();
  const ompl::base::PlannerStatus status = ss.solve(settings_.timeout_s);
  r.status = status.asString();
  r.plan_time_s = ss.getLastPlanComputationTime();
  r.checks_plan = checker_.calls();
  r.success = status == ompl::base::PlannerStatus::EXACT_SOLUTION;
  if (r.success) {
    r.raw_states = ss.getSolutionPath().getStateCount();
    // MoveIt: simplifySolution(allowed_planning_time - plan time), with at least one full pass.
    r.simplify_budget_s = std::max(0.0, settings_.timeout_s - r.plan_time_s);
    ss.simplifySolution(ompl::base::timedPlannerTerminationCondition(r.simplify_budget_s));
    r.simplify_time_s = ss.getLastSimplificationTime();
    const auto & path = ss.getSolutionPath();
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const double * v = values(path.getState(i));
      r.path.emplace_back(v, v + n);
    }
  }
  r.checks_total = checker_.calls();
  r.check_time_us_median = checker_.medianCallTimeUs();
  r.check_time_total_s = checker_.totalCallTimeS();
  return r;
}

}  // namespace pcp_direct

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Direct planner: OMPL RRTConnect in a 6-D real-vector joint space, with the Pinocchio/coal
// checker as the state validity checker and OMPL's own path simplifier. No MoveIt code.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "pcp_direct/collision.hpp"

namespace pcp_direct
{

using JointPath = std::vector<std::vector<double>>;

/// The OMPL settings the MoveIt pipeline uses for a group, read from the same
/// ompl_planning.yaml so that the two pipelines cannot drift apart.
struct PlannerSettings
{
  std::string planner_config;                    ///< e.g. RRTConnectRange2
  std::string planner_type;                      ///< must be geometric::RRTConnect
  double range = 0.0;                            ///< 0 = OMPL default (0.2 x extent)
  double longest_valid_segment_fraction = 0.01;  ///< MoveIt's default if the YAML has none
  bool range_set_in_yaml = false;
  bool lvsf_set_in_yaml = false;
  double timeout_s = 10.0;
  unsigned int ompl_seed = 1;
};

/// Read `<group>.default_planner_config`, that config's `type` and `range`, and
/// `<group>.longest_valid_segment_fraction` from MoveIt's ompl_planning.yaml.
PlannerSettings loadMoveItOmplSettings(const std::string & yaml_path, const std::string & group);

/// RealVectorStateSpace with MoveIt's joint-space metric: the distance is the SUM of per-joint
/// distances (MoveIt's JointModelGroup::distance for bounded revolute joints) and the maximum
/// extent is the SUM of the joint ranges. With these two overrides, OMPL's range,
/// longest_valid_segment_fraction and validSegmentCount mean exactly what they mean inside MoveIt.
/// Sampling and (linear) interpolation are RealVectorStateSpace's, which equal MoveIt's.
class JointSpaceL1 : public ompl::base::RealVectorStateSpace
{
public:
  explicit JointSpaceL1(unsigned int dim) : RealVectorStateSpace(dim) { setName("JointSpaceL1"); }
  double distance(const ompl::base::State * a, const ompl::base::State * b) const override;
  double getMaximumExtent() const override;
};

struct PlanResult
{
  bool success = false;
  std::string status;                 ///< OMPL planner status string
  double plan_time_s = 0.0;           ///< SimpleSetup::getLastPlanComputationTime
  double simplify_budget_s = 0.0;     ///< timeout - plan time, as MoveIt does
  double simplify_time_s = 0.0;       ///< SimpleSetup::getLastSimplificationTime
  std::size_t checks_plan = 0;        ///< validity checks during solve()
  std::size_t checks_total = 0;       ///< solve() + simplification
  double check_time_us_median = 0.0;  ///< median over all checks of this request
  double check_time_total_s = 0.0;    ///< summed time inside the checker
  std::size_t raw_states = 0;         ///< states in the unsimplified path
  JointPath path;                     ///< simplified path, start and goal included
};

class DirectPlanner
{
public:
  DirectPlanner(CollisionChecker & checker, const PlannerSettings & settings);

  /// Plan start -> goal (joint order of the checker). The checker's statistics are reset first.
  PlanResult plan(const std::vector<double> & start, const std::vector<double> & goal);

  double maximumExtent() const;
  double longestValidSegment() const;  ///< rad (L1) between checks along a motion
  double effectiveRange() const;       ///< rad (L1) per RRT extension after setup
  const PlannerSettings & settings() const { return settings_; }

private:
  CollisionChecker & checker_;
  PlannerSettings settings_;
  std::shared_ptr<JointSpaceL1> space_;
  double effective_range_ = 0.0;
};

}  // namespace pcp_direct

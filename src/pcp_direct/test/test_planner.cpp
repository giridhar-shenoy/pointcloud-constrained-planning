// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// The direct planner uses MoveIt's settings and MoveIt's joint-space metric, and its paths are
// collision-free for both checkers.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/util/RandomNumbers.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <nlohmann/json.hpp>

#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"
#include "pcp_benchmark/shelf_scene.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/moveit_reference.hpp"
#include "pcp_direct/planner.hpp"

namespace pcp_direct
{
namespace
{
const char * const kGroup = "ur_manipulator";

class DirectPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { ompl::RNG::setSeed(1); }

  void SetUp() override
  {
    model_ = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
    jmg_ = model_->getJointModelGroup(kGroup);
    checker_ = std::make_unique<CollisionChecker>(
      readTextFile(PCP_TEST_URDF), readTextFile(PCP_TEST_SRDF), loadBoxes(PCP_SHELF_YAML),
      jmg_->getVariableNames());
    settings_ = loadMoveItOmplSettings(PCP_OMPL_YAML, kGroup);
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * jmg_ = nullptr;
  std::unique_ptr<CollisionChecker> checker_;
  PlannerSettings settings_;
};

TEST_F(DirectPlannerTest, ReadsMoveItsPlannerSettings)
{
  EXPECT_EQ(settings_.planner_config, "RRTConnectRange2");
  EXPECT_EQ(settings_.planner_type, "geometric::RRTConnect");
  EXPECT_TRUE(settings_.range_set_in_yaml);
  EXPECT_DOUBLE_EQ(settings_.range, 2.0);
  EXPECT_TRUE(settings_.lvsf_set_in_yaml);
  EXPECT_DOUBLE_EQ(settings_.longest_valid_segment_fraction, 0.0003);
}

TEST_F(DirectPlannerTest, MetricAndExtentEqualMoveIts)
{
  DirectPlanner planner(*checker_, settings_);
  double moveit_extent = 0.0;
  for (const auto * joint : jmg_->getActiveJointModels()) {
    moveit_extent += joint->getMaximumExtent();
  }
  EXPECT_NEAR(planner.maximumExtent(), moveit_extent, 1e-12);
  EXPECT_NEAR(planner.maximumExtent(), 22.0 * M_PI, 1e-9);  // 5 x 4 pi + 2 pi
  EXPECT_NEAR(planner.longestValidSegment(), 0.0003 * 22.0 * M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(planner.effectiveRange(), 2.0);

  auto space = std::make_shared<JointSpaceL1>(6);
  ompl::base::RealVectorBounds b(6);
  b.setLow(-10);
  b.setHigh(10);
  space->setBounds(b);
  pcp_benchmark::DeterministicRng rng(3);
  for (int k = 0; k < 20; ++k) {
    std::vector<double> qa(6);
    std::vector<double> qb(6);
    for (int j = 0; j < 6; ++j) {
      qa[j] = rng.uniform(checker_->lowerLimits()[j], checker_->upperLimits()[j]);
      qb[j] = rng.uniform(checker_->lowerLimits()[j], checker_->upperLimits()[j]);
    }
    ompl::base::ScopedState<> sa(space);
    ompl::base::ScopedState<> sb(space);
    for (int j = 0; j < 6; ++j) {
      sa[j] = qa[j];
      sb[j] = qb[j];
    }
    EXPECT_NEAR(space->distance(sa.get(), sb.get()), jmg_->distance(qa.data(), qb.data()), 1e-12);
  }
}

TEST_F(DirectPlannerTest, PlansACommittedQueryCollisionFreeForBothCheckers)
{
  const auto set = pcp_benchmark::queriesFromJson(
    nlohmann::json::parse(pcp_benchmark::readFile(PCP_COMMITTED_BOX_QUERIES)));
  const auto & q = set.queries.at(0);
  DirectPlanner planner(*checker_, settings_);
  const PlanResult r = planner.plan(q.start, q.goal);
  ASSERT_TRUE(r.success) << r.status;
  EXPECT_GT(r.plan_time_s, 0.0);
  EXPECT_GE(r.simplify_time_s, 0.0);
  EXPECT_NEAR(r.simplify_budget_s, settings_.timeout_s - r.plan_time_s, 1e-9);
  EXPECT_GT(r.checks_plan, 0u);
  EXPECT_GE(r.checks_total, r.checks_plan);
  EXPECT_GT(r.check_time_us_median, 0.0);
  ASSERT_GE(r.path.size(), 2u);
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_DOUBLE_EQ(r.path.front()[j], q.start[j]);
    EXPECT_DOUBLE_EQ(r.path.back()[j], q.goal[j]);
  }
  // Dense re-check of the straight-line segments at 0.01 rad: direct checker and MoveIt's scene.
  planning_scene::PlanningScene scene(model_);
  pcp_benchmark::addShelfToPlanningScene(pcp_benchmark::loadShelfScene(PCP_SHELF_YAML), scene, 0.0);
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  std::size_t direct_hits = 0;
  std::size_t moveit_hits = 0;
  for (const auto & qd : pcp_benchmark::densify(r.path, 0.01)) {
    direct_hits += checker_->isColliding(qd) ? 1 : 0;
    state.setJointGroupPositions(jmg_, qd);
    state.update();
    moveit_hits += moveit_reference::isColliding(scene, state, kGroup) ? 1 : 0;
  }
  EXPECT_EQ(direct_hits, 0u);
  EXPECT_EQ(moveit_hits, 0u);
}

TEST_F(DirectPlannerTest, RejectsAStartInCollision)
{
  DirectPlanner planner(*checker_, settings_);
  const std::vector<double> folded = {0.0, -1.5707, 3.0, 0.0, 0.0, 0.0};
  const std::vector<double> home = {0.0, -1.5707, 0.0, 0.0, 0.0, 0.0};
  const PlanResult r = planner.plan(folded, home);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

}  // namespace
}  // namespace pcp_direct

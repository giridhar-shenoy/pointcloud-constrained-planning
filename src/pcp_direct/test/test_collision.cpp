// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Direct checker (Pinocchio + coal) against MoveIt's planning scene on hand-picked
// configurations, and the pair bookkeeping that makes the two comparable.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometric_shapes/shapes.h>
#include <octomap/OcTree.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"
#include "pcp_benchmark/shelf_scene.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/moveit_reference.hpp"

namespace pcp_direct
{
namespace
{
const char * const kGroup = "ur_manipulator";

class DirectCollision : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
    jmg_ = model_->getJointModelGroup(kGroup);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    pcp_benchmark::addShelfToPlanningScene(
      pcp_benchmark::loadShelfScene(PCP_SHELF_YAML), *scene_, 0.0);
    direct_ = std::make_unique<CollisionChecker>(
      readTextFile(PCP_TEST_URDF), readTextFile(PCP_TEST_SRDF), loadBoxes(PCP_SHELF_YAML),
      jmg_->getVariableNames());
  }

  bool moveitColliding(const std::vector<double> & q)
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg_, q);
    state.update();
    return moveit_reference::isColliding(*scene_, state, kGroup);
  }

  std::vector<NamedPair> moveitPairs(const std::vector<double> & q)
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg_, q);
    state.update();
    return moveit_reference::collidingPairs(*scene_, state, kGroup);
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * jmg_ = nullptr;
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<CollisionChecker> direct_;
};

TEST_F(DirectCollision, LoadsTheSevenShelfBoxes)
{
  const auto boxes = loadBoxes(PCP_SHELF_YAML);
  ASSERT_EQ(boxes.size(), 7u);
  EXPECT_EQ(boxes.front().name, "board_bottom");
  // 8 URDF collision geometries (7 arm links + the ground plane) + 7 boxes.
  EXPECT_EQ(direct_->geometryCount(), 15u);
}

TEST_F(DirectCollision, JointBoundsMatchMoveIt)
{
  const auto & names = jmg_->getVariableNames();
  for (std::size_t j = 0; j < names.size(); ++j) {
    const auto & b = model_->getVariableBounds(names[j]);
    EXPECT_DOUBLE_EQ(direct_->lowerLimits()[j], b.min_position_) << names[j];
    EXPECT_DOUBLE_EQ(direct_->upperLimits()[j], b.max_position_) << names[j];
  }
}

// The SRDF disables 11 link pairs; 4 of them involve links without collision geometry
// (base_link, tool0), so 7 geometry pairs are removed.
TEST_F(DirectCollision, SrdfRemovesTheDisabledGeometryPairs)
{
  std::size_t expected = 0;
  for (const auto & dc : model_->getSRDF()->getDisabledCollisionPairs()) {
    const auto * a = model_->getLinkModel(dc.link1_);
    const auto * b = model_->getLinkModel(dc.link2_);
    if (a && b && !a->getShapes().empty() && !b->getShapes().empty()) {
      ++expected;
    }
  }
  EXPECT_EQ(expected, 7u);
  EXPECT_EQ(direct_->srdfRemovedPairCount(), expected);
}

TEST_F(DirectCollision, ActivePairsAreExactlyThePairsMoveItChecks)
{
  const auto moveit_pairs = moveit_reference::checkedPairs(*scene_, kGroup);
  const auto direct_pairs = direct_->activePairs();
  EXPECT_EQ(direct_pairs.size(), moveit_pairs.size());
  EXPECT_EQ(direct_pairs, moveit_pairs);
}

TEST_F(DirectCollision, HomeConfigurationIsFree)
{
  const std::vector<double> home = {0.0, -1.5707, 0.0, 0.0, 0.0, 0.0};  // SRDF "home"
  EXPECT_FALSE(moveitColliding(home));
  EXPECT_FALSE(direct_->isColliding(home));
  EXPECT_TRUE(direct_->collidingPairs(home).empty());
  EXPECT_GT(direct_->minimumDistance(home).distance, 0.0);
}

TEST_F(DirectCollision, FoldedElbowSelfCollides)
{
  // Arm up, elbow folded back to 3.0 rad: the wrist comes down onto the upper arm / shoulder.
  const std::vector<double> folded = {0.0, -1.5707, 3.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(moveitColliding(folded));
  EXPECT_TRUE(direct_->isColliding(folded));
  const auto d = direct_->collidingPairs(folded);
  const auto m = moveitPairs(folded);
  EXPECT_FALSE(d.empty());
  EXPECT_EQ(d, m);
  for (const auto & p : d) {  // self-collision only: no shelf box involved
    EXPECT_EQ(p.first.find("board"), std::string::npos);
    EXPECT_EQ(p.second.find("wall"), std::string::npos);
  }
}

TEST_F(DirectCollision, ToolInsideTheMiddleBoardCollidesWithIt)
{
  // Tool-down IK solution with tool0 at the centre height of board_middle (z = 0.49).
  pcp_benchmark::GeneratorConfig cfg;
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  const Eigen::Vector3d target(0.60, 0.30, 0.49);
  state.setJointGroupPositions(
    jmg_, std::vector<double>{std::atan2(0.30, 0.60), -1.2, 1.6, -M_PI / 2 - 0.4, -M_PI / 2, 0.0});
  ASSERT_TRUE(
    pcp_benchmark::solveIkDls(
      state, jmg_, model_->getLinkModel("tool0"), pcp_benchmark::toolDownPose(target, 0.0), cfg));
  std::vector<double> q;
  state.copyJointGroupPositions(jmg_, q);

  ASSERT_TRUE(moveitColliding(q));
  ASSERT_TRUE(direct_->isColliding(q));
  const auto d = direct_->collidingPairs(q);
  const auto has_board = [](const std::vector<NamedPair> & pairs) {
    return std::any_of(pairs.begin(), pairs.end(), [](const NamedPair & p) {
      return p.first == "board_middle" || p.second == "board_middle";
    });
  };
  EXPECT_TRUE(has_board(d));
  EXPECT_TRUE(has_board(moveitPairs(q)));
  EXPECT_LE(direct_->pairDistance(q, {"board_middle", "wrist_3_link"}), 0.0);
}

// SHOULD (shared octomap): the same octree in MoveIt's scene (as its "<octomap>" object) and in the
// direct checker (one coal::OcTree) gives the same verdicts and names the same pair.
TEST_F(DirectCollision, OctreeGivesTheSameVerdictsAsMoveIt)
{
  auto tree = std::make_shared<octomap::OcTree>(0.02);
  for (double x = 0.41; x < 0.80; x += 0.02) {  // fill board_middle's volume with 2 cm voxels
    for (double y = -0.61; y < 0.62; y += 0.02) {
      tree->updateNode(octomap::point3d(x, y, 0.49), true);
    }
  }
  tree->updateInnerOccupancy();
  planning_scene::PlanningScene scene(model_);
  scene.getWorldNonConst()->addToObject(
    planning_scene::PlanningScene::OCTOMAP_NS, std::make_shared<const shapes::OcTree>(tree),
    Eigen::Isometry3d::Identity());
  CollisionChecker direct(
    readTextFile(PCP_TEST_URDF), readTextFile(PCP_TEST_SRDF), {}, jmg_->getVariableNames(), {},
    tree);
  const std::vector<double> home = {0.0, -1.5707, 0.0, 0.0, 0.0, 0.0};
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg_, home);
  state.update();
  EXPECT_FALSE(moveit_reference::isColliding(scene, state, kGroup));
  EXPECT_FALSE(direct.isColliding(home));

  pcp_benchmark::GeneratorConfig cfg;
  state.setJointGroupPositions(
    jmg_, std::vector<double>{std::atan2(0.30, 0.60), -1.2, 1.6, -M_PI / 2 - 0.4, -M_PI / 2, 0.0});
  ASSERT_TRUE(
    pcp_benchmark::solveIkDls(
      state, jmg_, model_->getLinkModel("tool0"),
      pcp_benchmark::toolDownPose(Eigen::Vector3d(0.60, 0.30, 0.49), 0.0), cfg));
  std::vector<double> q;
  state.copyJointGroupPositions(jmg_, q);
  state.update();
  EXPECT_TRUE(moveit_reference::isColliding(scene, state, kGroup));
  EXPECT_TRUE(direct.isColliding(q));
  const auto d = direct.collidingPairs(q);
  const auto m = moveit_reference::collidingPairs(scene, state, kGroup);
  EXPECT_FALSE(d.empty());
  EXPECT_EQ(d, m);
}

TEST_F(DirectCollision, IsValidCountsAndTimesCalls)
{
  const std::vector<double> home = {0.0, -1.5707, 0.0, 0.0, 0.0, 0.0};
  direct_->resetStatistics();
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(direct_->isValid(home));
  }
  EXPECT_EQ(direct_->calls(), 10u);
  EXPECT_GT(direct_->medianCallTimeUs(), 0.0);
  EXPECT_GT(direct_->totalCallTimeS(), 0.0);
  direct_->resetStatistics();
  EXPECT_EQ(direct_->calls(), 0u);
}

}  // namespace
}  // namespace pcp_direct

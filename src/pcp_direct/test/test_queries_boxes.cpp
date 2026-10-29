// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// results/queries_boxes.json (the v0.2 query set, box shelf scene, seed 0) regenerates byte for
// byte, and every start and goal is collision-free for BOTH checkers.
#include <gtest/gtest.h>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <nlohmann/json.hpp>

#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"
#include "pcp_benchmark/shelf_scene.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/moveit_reference.hpp"

namespace pcp_direct
{
namespace
{
TEST(QueriesBoxes, CommittedFileRegeneratesExactly)
{
  const auto model = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
  const auto committed = nlohmann::json::parse(pcp_benchmark::readFile(PCP_COMMITTED_BOX_QUERIES));
  pcp_benchmark::GeneratorConfig cfg;
  cfg.seed = committed.at("seed").get<std::uint64_t>();
  cfg.count = committed.at("count").get<std::size_t>();
  ASSERT_EQ(cfg.seed, 0u);
  ASSERT_EQ(cfg.count, 100u);
  const auto regenerated = pcp_benchmark::toJson(
    pcp_benchmark::generateQueries(model, pcp_benchmark::loadShelfScene(PCP_SHELF_YAML), cfg));
  EXPECT_EQ(regenerated.dump(1), committed.dump(1));
}

TEST(QueriesBoxes, EveryStartAndGoalIsValidForBothCheckers)
{
  const auto model = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
  const auto set = pcp_benchmark::queriesFromJson(
    nlohmann::json::parse(pcp_benchmark::readFile(PCP_COMMITTED_BOX_QUERIES)));
  const auto * jmg = model->getJointModelGroup(set.config.group);
  planning_scene::PlanningScene scene(model);
  pcp_benchmark::addShelfToPlanningScene(pcp_benchmark::loadShelfScene(PCP_SHELF_YAML), scene, 0.0);
  CollisionChecker direct(
    readTextFile(PCP_TEST_URDF), readTextFile(PCP_TEST_SRDF), loadBoxes(PCP_SHELF_YAML),
    jmg->getVariableNames());
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const auto & q : set.queries) {
    for (const auto * joints : {&q.start, &q.goal}) {
      state.setJointGroupPositions(jmg, *joints);
      state.update();
      EXPECT_FALSE(moveit_reference::isColliding(scene, state, set.config.group)) << q.id;
      EXPECT_FALSE(direct.isColliding(*joints)) << q.id;
    }
  }
}
}  // namespace
}  // namespace pcp_direct

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"

namespace pcp_benchmark
{
namespace
{
class QueryDeterminism : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
    shelf_ = loadShelfScene(PCP_SHELF_YAML);
  }
  moveit::core::RobotModelPtr model_;
  ShelfScene shelf_;
};

TEST_F(QueryDeterminism, SameSeedSameQueries)
{
  GeneratorConfig cfg;
  cfg.count = 5;
  const auto a = toJson(generateQueries(model_, shelf_, cfg)).dump();
  const auto b = toJson(generateQueries(model_, shelf_, cfg)).dump();
  EXPECT_EQ(a, b);
  cfg.seed = 1;
  EXPECT_NE(a, toJson(generateQueries(model_, shelf_, cfg)).dump());
}

// The committed results/queries.json must be exactly what the generator produces from its
// recorded seed and count. If this fails, the benchmark CSV no longer matches its inputs.
TEST_F(QueryDeterminism, CommittedQueriesRegenerateExactly)
{
  const auto committed = nlohmann::json::parse(readFile(PCP_COMMITTED_QUERIES));
  GeneratorConfig cfg;
  cfg.seed = committed.at("seed").get<std::uint64_t>();
  cfg.count = committed.at("count").get<std::size_t>();
  ASSERT_EQ(cfg.count, 100u);
  ASSERT_EQ(cfg.seed, 0u);
  const auto regenerated = toJson(generateQueries(model_, shelf_, cfg));
  EXPECT_EQ(regenerated.dump(1), committed.dump(1));
}

TEST_F(QueryDeterminism, CommittedQueriesAreValid)
{
  const auto set = queriesFromJson(nlohmann::json::parse(readFile(PCP_COMMITTED_QUERIES)));
  planning_scene::PlanningScene scene(model_);
  addShelfToPlanningScene(shelf_, scene, 0.0);
  const auto * jmg = model_->getJointModelGroup(set.config.group);
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  for (const auto & q : set.queries) {
    for (const auto * joints : {&q.start, &q.goal}) {
      state.setJointGroupPositions(jmg, *joints);
      state.update();
      EXPECT_TRUE(state.satisfiesBounds(jmg)) << "query " << q.id;
      EXPECT_FALSE(scene.isStateColliding(state, set.config.group)) << "query " << q.id;
      // Tool points down (inside the 5 deg cone with a wide margin) at both ends.
      EXPECT_LT(tiltFromDownDeg(state.getGlobalLinkTransform(set.config.tip_link).linear()), 0.01)
        << "query " << q.id;
    }
    EXPECT_GE((q.goal_tcp.head<3>() - q.start_tcp.head<3>()).norm(), 0.10) << "query " << q.id;
  }
}
}  // namespace
}  // namespace pcp_benchmark

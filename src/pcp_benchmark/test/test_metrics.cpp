// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <cmath>

#include <moveit/robot_state/robot_state.hpp>

#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/robot_model_files.hpp"

namespace pcp_benchmark
{
namespace
{
constexpr double kDeg = M_PI / 180.0;
const std::vector<double> kToolDown = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};

TEST(Densify, RespectsStepAndKeepsEndpoints)
{
  const JointPath path = {{0.0, 0.0}, {0.1, -0.05}, {0.1, 0.25}};
  const JointPath dense = densify(path, 0.01);
  EXPECT_EQ(dense.front(), path.front());
  EXPECT_EQ(dense.back(), path.back());
  EXPECT_EQ(dense.size(), 1u + 10u + 30u);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    for (std::size_t j = 0; j < 2; ++j) {
      EXPECT_LE(std::abs(dense[i][j] - dense[i - 1][j]), 0.01 + 1e-12);
    }
  }
  EXPECT_THROW(densify(path, 0.0), std::invalid_argument);
}

TEST(JointPathLength, SumOfSegmentNorms)
{
  EXPECT_DOUBLE_EQ(jointPathLength({}), 0.0);
  EXPECT_DOUBLE_EQ(jointPathLength({{1.0, 2.0}}), 0.0);
  EXPECT_DOUBLE_EQ(jointPathLength({{0.0, 0.0}, {3.0, 4.0}, {3.0, 0.0}}), 5.0 + 4.0);
  // Densifying a straight segment does not change its length.
  const JointPath straight = {{0.0, 0.0, 0.0}, {0.3, -0.2, 0.6}};
  EXPECT_NEAR(jointPathLength(densify(straight, 0.01)), jointPathLength(straight), 1e-12);
}

TEST(Tilt, Basics)
{
  Eigen::Matrix3d down = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_NEAR(tiltFromDownDeg(down), 0.0, 1e-9);
  EXPECT_NEAR(tiltFromDownDeg(Eigen::Matrix3d::Identity()), 180.0, 1e-9);
  Eigen::Matrix3d tilted = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) * down;
  EXPECT_NEAR(tiltFromDownDeg(tilted), 0.3 / kDeg, 1e-9);
}

class PathMetricsTest : public ::testing::Test
{
protected:
  void SetUp() override { model_ = loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF); }
  moveit::core::RobotModelPtr model_;
};

TEST_F(PathMetricsTest, BaseRotationGivesArcLength)
{
  // Rotating only the base joint by d moves tool0 on a circle of radius r around world z.
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();
  state.setJointGroupPositions("ur_manipulator", kToolDown);
  state.update();
  const Eigen::Vector3d p = state.getGlobalLinkTransform("tool0").translation();
  const double radius = std::hypot(p.x(), p.y());
  auto end = kToolDown;
  end[0] += 0.8;
  const auto m = computePathMetrics(model_, "ur_manipulator", "tool0", {kToolDown, end}, 0.001);
  EXPECT_NEAR(m.joint_path_rad, 0.8, 1e-12);
  EXPECT_NEAR(m.tcp_path_m, radius * 0.8, 1e-6);
  EXPECT_LT(m.max_tilt_deg, 1e-3);  // base rotation keeps the tool vertical
  EXPECT_EQ(m.n_waypoints, 2u);
  EXPECT_EQ(m.n_dense_states, 801u);
}

TEST_F(PathMetricsTest, MaxTiltSeesTheMiddleOfASegment)
{
  // Wrist 2 goes out by 20 deg and back: the endpoints are vertical, the middle is not.
  auto mid = kToolDown;
  mid[4] += 20.0 * kDeg;
  const auto m = computePathMetrics(model_, "ur_manipulator", "tool0", {kToolDown, mid, kToolDown});
  EXPECT_NEAR(m.max_tilt_deg, 20.0, 1e-3);
  EXPECT_NEAR(m.joint_path_rad, 40.0 * kDeg, 1e-9);
}
}  // namespace
}  // namespace pcp_benchmark

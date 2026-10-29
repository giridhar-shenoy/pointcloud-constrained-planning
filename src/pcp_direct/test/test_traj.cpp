// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pcp_benchmark/robot_model_files.hpp"
#include "pcp_direct/traj.hpp"
#include "pcp_direct/traj/trapezoidal.hpp"

namespace pcp_direct
{
namespace
{
double seconds(const builtin_interfaces::msg::Duration & d) { return d.sec + 1e-9 * d.nanosec; }

const std::vector<std::string> kJoints = {"shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint",        "wrist_1_joint",
                                          "wrist_2_joint",      "wrist_3_joint"};

// The direct limits are the ones MoveIt's TOTG uses: URDF velocity, ur_moveit_config acceleration.
TEST(Traj, LimitsAreMoveItsTimeParameterisationLimits)
{
  const auto limits = loadJointLimits(PCP_JOINT_LIMITS_YAML, kJoints);
  const auto model = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
  const YAML::Node moveit_limits = YAML::LoadFile(
    ament_index_cpp::get_package_share_directory("ur_moveit_config") +
    "/config/joint_limits.yaml")["joint_limits"];
  for (std::size_t j = 0; j < kJoints.size(); ++j) {
    EXPECT_DOUBLE_EQ(limits.max_velocity[j], model->getVariableBounds(kJoints[j]).max_velocity_);
    EXPECT_DOUBLE_EQ(
      limits.max_acceleration[j], moveit_limits[kJoints[j]]["max_acceleration"].as<double>());
  }
}

TEST(Traj, RestToRestSegmentsRespectLimitsAndHitWaypoints)
{
  const auto limits = loadJointLimits(PCP_JOINT_LIMITS_YAML, kJoints);
  const JointPath path = {
    {0.0, -1.5, 1.0, 0.0, 0.0, 0.0},
    {1.0, -1.2, 0.5, 0.3, -0.2, 2.0},
    {1.05, -1.2, 0.5, 0.3, -0.2, 2.0},  // a short segment: acceleration-limited
  };
  const auto out = parameterise(path, limits, 0.1);
  ASSERT_EQ(out.msg.joint_names, kJoints);
  double expected = 0.0;
  for (std::size_t k = 1; k < path.size(); ++k) {
    double t = 0.0;
    for (std::size_t j = 0; j < 6; ++j) {
      t = std::max(
        t, profiles::min_duration(
             path[k][j] - path[k - 1][j], limits.max_velocity[j], limits.max_acceleration[j]));
    }
    expected += t;
  }
  EXPECT_NEAR(out.duration_s, expected, 1e-9);
  EXPECT_NEAR(seconds(out.msg.points.back().time_from_start), expected, 1e-6);
  EXPECT_GE(out.param_time_s, 0.0);

  double last_t = -1.0;
  for (const auto & p : out.msg.points) {
    const double t = seconds(p.time_from_start);
    EXPECT_GT(t, last_t);
    last_t = t;
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_LE(std::abs(p.velocities[j]), limits.max_velocity[j] + 1e-9);
      EXPECT_LE(std::abs(p.accelerations[j]), limits.max_acceleration[j] + 1e-9);
    }
  }
  const auto & first = out.msg.points.front();
  const auto & last = out.msg.points.back();
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_DOUBLE_EQ(first.positions[j], path.front()[j]);
    EXPECT_DOUBLE_EQ(first.velocities[j], 0.0);
    EXPECT_DOUBLE_EQ(last.positions[j], path.back()[j]);
    EXPECT_DOUBLE_EQ(last.velocities[j], 0.0);
  }
  // The middle waypoint is reached at rest, exactly at the end of the first segment.
  bool found = false;
  for (const auto & p : out.msg.points) {
    if (std::abs(p.positions[0] - 1.0) < 1e-12 && std::abs(p.positions[5] - 2.0) < 1e-12) {
      found = true;
      for (std::size_t j = 0; j < 6; ++j) {
        EXPECT_DOUBLE_EQ(p.velocities[j], 0.0);
      }
    }
  }
  EXPECT_TRUE(found);
}

TEST(Traj, RejectsDegenerateInput)
{
  const auto limits = loadJointLimits(PCP_JOINT_LIMITS_YAML, kJoints);
  EXPECT_THROW(parameterise({{0, 0, 0, 0, 0, 0}}, limits), std::invalid_argument);
  EXPECT_THROW(
    parameterise({{0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}}, limits, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace pcp_direct

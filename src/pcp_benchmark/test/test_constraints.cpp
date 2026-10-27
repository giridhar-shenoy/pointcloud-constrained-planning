// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <cmath>

#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/transforms/transforms.hpp>

#include "pcp_benchmark/constraints.hpp"
#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"

namespace pcp_benchmark
{
namespace
{
constexpr double kDeg = M_PI / 180.0;
// Canonical UR5e tool-down configuration (tool0 z along world -z).
const std::vector<double> kToolDown = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};

class ConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
    jmg_ = model_->getJointModelGroup("ur_manipulator");
    ASSERT_NE(jmg_, nullptr);
  }

  bool satisfied(const moveit_msgs::msg::Constraints & msg, const std::vector<double> & q)
  {
    kinematic_constraints::OrientationConstraint oc(model_);
    moveit::core::Transforms tf(model_->getModelFrame());
    EXPECT_TRUE(oc.configure(msg.orientation_constraints.at(0), tf));
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg_, q);
    state.update();
    return oc.decide(state).satisfied;
  }

  double tilt(const std::vector<double> & q)
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg_, q);
    state.update();
    return tiltFromDownDeg(state.getGlobalLinkTransform("tool0").linear());
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * jmg_ = nullptr;
};

TEST(ConstraintBuilder, NoneIsEmpty)
{
  const auto c = buildPathConstraints(ConstraintType::kNone);
  EXPECT_TRUE(c.orientation_constraints.empty());
  EXPECT_TRUE(c.position_constraints.empty());
  EXPECT_TRUE(c.joint_constraints.empty());
  EXPECT_TRUE(c.visibility_constraints.empty());
}

TEST(ConstraintBuilder, OrientationMessageFields)
{
  const auto c = buildPathConstraints(ConstraintType::kOrientation);
  ASSERT_EQ(c.orientation_constraints.size(), 1u);
  EXPECT_TRUE(c.position_constraints.empty());
  const auto & oc = c.orientation_constraints[0];
  EXPECT_EQ(oc.header.frame_id, "world");
  EXPECT_EQ(oc.link_name, "tool0");
  EXPECT_EQ(oc.parameterization, moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR);
  EXPECT_DOUBLE_EQ(oc.orientation.x, 1.0);
  EXPECT_DOUBLE_EQ(oc.orientation.w, 0.0);
  EXPECT_NEAR(oc.absolute_x_axis_tolerance, 5.0 * kDeg / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(oc.absolute_y_axis_tolerance, 5.0 * kDeg / std::sqrt(2.0), 1e-12);
  EXPECT_DOUBLE_EQ(oc.absolute_z_axis_tolerance, M_PI);
  EXPECT_DOUBLE_EQ(oc.weight, 1.0);
}

TEST(ConstraintBuilder, ParsesNames)
{
  EXPECT_EQ(constraintTypeFromString("none"), ConstraintType::kNone);
  EXPECT_EQ(constraintTypeFromString("orientation"), ConstraintType::kOrientation);
  EXPECT_EQ(toString(ConstraintType::kOrientation), "orientation");
  EXPECT_THROW(constraintTypeFromString("line"), std::invalid_argument);
}

TEST_F(ConstraintTest, ToolDownSatisfiesAtAnyYaw)
{
  const auto c = buildPathConstraints(ConstraintType::kOrientation);
  for (double yaw : {-3.0, -1.5, 0.0, 0.7, 2.9}) {
    auto q = kToolDown;
    q[5] = yaw;  // wrist 3 spins the tool about its (vertical) axis
    EXPECT_LT(tilt(q), 1e-3);
    EXPECT_TRUE(satisfied(c, q)) << "yaw " << yaw;
  }
}

TEST_F(ConstraintTest, SmallTiltAcceptedLargeTiltRejected)
{
  const auto c = buildPathConstraints(ConstraintType::kOrientation);
  auto q = kToolDown;
  q[4] += 3.0 * kDeg;  // wrist 2 tilts the tool about a horizontal axis
  EXPECT_NEAR(tilt(q), 3.0, 1e-3);
  EXPECT_TRUE(satisfied(c, q));
  q = kToolDown;
  q[4] += 6.0 * kDeg;
  EXPECT_NEAR(tilt(q), 6.0, 1e-3);
  EXPECT_FALSE(satisfied(c, q));
}

// Soundness of the inscribed-square tolerance: whatever MoveIt accepts tilts at most 5 deg.
TEST_F(ConstraintTest, EverySatisfyingStateIsWithinTheCone)
{
  const auto c = buildPathConstraints(ConstraintType::kOrientation);
  DeterministicRng rng(42);
  int accepted = 0;
  for (int i = 0; i < 2000; ++i) {
    auto q = kToolDown;
    q[0] += rng.uniform(-1.0, 1.0);
    q[3] += rng.uniform(-8.0, 8.0) * kDeg;
    q[4] += rng.uniform(-8.0, 8.0) * kDeg;
    q[5] += rng.uniform(-M_PI, M_PI);
    if (satisfied(c, q)) {
      ++accepted;
      EXPECT_LE(tilt(q), 5.0 + 1e-6);
    }
  }
  EXPECT_GT(accepted, 100);  // the check above must not be vacuous
}
}  // namespace
}  // namespace pcp_benchmark

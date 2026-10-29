// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Keep-upright constraint for the direct planner: the same rotation-vector dead band as v0.1's
// MoveIt constraint, and a constrained plan that keeps the tool inside the 5 deg cone.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <ompl/util/RandomNumbers.h>
#include <nlohmann/json.hpp>

#include "pcp_benchmark/constraints.hpp"
#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/constraint.hpp"
#include "pcp_direct/planner.hpp"

namespace pcp_direct
{
namespace
{
const std::vector<std::string> kJoints = {"shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint",        "wrist_1_joint",
                                          "wrist_2_joint",      "wrist_3_joint"};

Eigen::Matrix3d toolDown(double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

class KeepUpright : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { ompl::RNG::setSeed(1); }
  void SetUp() override
  {
    fk_ = std::make_shared<FrameKinematics>(readTextFile(PCP_TEST_URDF), kJoints, "tool0");
    set_ = pcp_benchmark::queriesFromJson(
      nlohmann::json::parse(pcp_benchmark::readFile(PCP_COMMITTED_BOX_QUERIES)));
    tol_ = pcp_benchmark::perAxisToleranceRad(5.0);
  }
  std::shared_ptr<FrameKinematics> fk_;
  pcp_benchmark::QuerySet set_;
  double tol_ = 0.0;
};

// Every committed query start is tool-down within 0.01 deg, so the dead band is zero there for a
// reference yaw at the query's own yaw, and the error's x/y components are tiny.
TEST_F(KeepUpright, QueryStartsSatisfyTheConstraint)
{
  for (const auto & q : set_.queries) {
    const double yaw = pcp_benchmark::circularMean(q.start_tcp[3], q.goal_tcp[3]);
    KeepUprightConstraint c(fk_, toolDown(yaw), Eigen::Vector3d(tol_, tol_, M_PI));
    const Eigen::Map<const Eigen::VectorXd> x(q.start.data(), 6);
    Eigen::VectorXd f(3);
    c.function(x, f);
    EXPECT_LT(f.norm(), 1e-9) << "query " << q.id;
    const Eigen::Vector3d e = c.error(x);
    EXPECT_LT(std::hypot(e.x(), e.y()), 1e-3) << "query " << q.id;
  }
}

TEST_F(KeepUpright, DeadBandIsZeroInsideAndLinearOutside)
{
  const auto & q = set_.queries.at(0);
  const Eigen::Map<const Eigen::VectorXd> x(q.start.data(), 6);
  // Reference tilted by 3 deg about x: inside the per-axis bound 5/sqrt(2) = 3.54 deg? No: 3 deg
  // about the tool x axis puts |r_x| = 3 deg < 3.54 deg -> zero. 6 deg -> 2.46 deg outside.
  const Eigen::Matrix3d base = fk_->pose(x).linear();
  for (const double tilt_deg : {3.0, 6.0}) {
    const Eigen::Matrix3d target =
      base *
      Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    KeepUprightConstraint c(fk_, target, Eigen::Vector3d(tol_, tol_, M_PI));
    Eigen::VectorXd f(3);
    c.function(x, f);
    const double expected = std::max(0.0, tilt_deg * M_PI / 180.0 - tol_);
    EXPECT_NEAR(f[0], expected, 1e-9) << tilt_deg;
    EXPECT_NEAR(f[1], 0.0, 1e-9);
    EXPECT_NEAR(f[2], 0.0, 1e-9);
  }
}

TEST_F(KeepUpright, JacobianMatchesFiniteDifferencesOutsideTheBand)
{
  const auto & q = set_.queries.at(1);
  Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(q.start.data(), 6);
  x[4] += 0.3;  // tilt the tool well outside the band
  KeepUprightConstraint c(fk_, toolDown(q.start_tcp[3]), Eigen::Vector3d(tol_, tol_, M_PI));
  Eigen::MatrixXd J(3, 6);
  c.jacobian(x, J);
  Eigen::VectorXd fp(3);
  Eigen::VectorXd fm(3);
  for (int j = 0; j < 6; ++j) {
    Eigen::VectorXd xp = x;
    Eigen::VectorXd xm = x;
    xp[j] += 1e-5;
    xm[j] -= 1e-5;
    c.function(xp, fp);
    c.function(xm, fm);
    EXPECT_LT(((fp - fm) / 2e-5 - J.col(j)).norm(), 1e-4) << "column " << j;
  }
}

TEST_F(KeepUpright, ConstrainedPlanKeepsTheToolUpright)
{
  const auto model = pcp_benchmark::loadRobotModel(PCP_TEST_URDF, PCP_TEST_SRDF);
  CollisionChecker checker(
    readTextFile(PCP_TEST_URDF), readTextFile(PCP_TEST_SRDF), loadBoxes(PCP_SHELF_YAML), kJoints);
  DirectPlanner planner(checker, loadMoveItOmplSettings(PCP_OMPL_YAML, "ur_manipulator"));
  // Query 3: start and goal in the same compartment (a short constrained motion).
  const auto & q = set_.queries.at(3);
  const double yaw = pcp_benchmark::circularMean(q.start_tcp[3], q.goal_tcp[3]);
  auto c =
    std::make_shared<KeepUprightConstraint>(fk_, toolDown(yaw), Eigen::Vector3d(tol_, tol_, M_PI));
  const auto r = planner.plan(q.start, q.goal, c);
  ASSERT_TRUE(r.success) << r.status;
  const auto metrics = pcp_benchmark::computePathMetrics(model, "ur_manipulator", "tool0", r.path);
  // Projection stops at a 1e-4 residual, so allow a hair over the 5 deg cone on the dense path.
  EXPECT_LT(metrics.max_tilt_deg, 5.1);
}

}  // namespace
}  // namespace pcp_direct

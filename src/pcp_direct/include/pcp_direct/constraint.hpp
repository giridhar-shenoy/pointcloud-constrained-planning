// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Keep-upright path constraint for the direct planner, in the same formulation MoveIt's OMPL
// constrained state space uses for v0.1's orientation constraint: the rotation vector
// r = log(R_tool^T * R_target) with per-axis bounds (x, y: 5 deg / sqrt(2); z: pi, i.e. yaw
// free), turned into an equality constraint F(q) = 0 by a dead-band penalty (zero inside the
// bounds, the distance to the bound outside). Co-dimension 3. No MoveIt code.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ompl/base/Constraint.h>
#include <Eigen/Geometry>

namespace pcp_direct
{

/// Forward kinematics of one frame with Pinocchio (joint order as in the CollisionChecker).
class FrameKinematics
{
public:
  FrameKinematics(
    const std::string & urdf_xml, const std::vector<std::string> & joint_names,
    const std::string & frame);
  ~FrameKinematics();
  Eigen::Isometry3d pose(const Eigen::Ref<const Eigen::VectorXd> & q) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class KeepUprightConstraint : public ompl::base::Constraint
{
public:
  /// @param target       reference tool orientation (tool z along world -z, reference yaw)
  /// @param tolerance    per-axis bounds on the rotation vector (rad), symmetric
  KeepUprightConstraint(
    std::shared_ptr<const FrameKinematics> fk, const Eigen::Matrix3d & target,
    const Eigen::Vector3d & tolerance);

  /// Rotation vector of R_tool^T * R_target (what the bounds apply to).
  Eigen::Vector3d error(const Eigen::Ref<const Eigen::VectorXd> & q) const;

  void function(
    const Eigen::Ref<const Eigen::VectorXd> & q, Eigen::Ref<Eigen::VectorXd> out) const override;
  /// Dead-band derivative (-1, 0, +1 per axis) times d(error)/dq by central differences.
  void jacobian(
    const Eigen::Ref<const Eigen::VectorXd> & q, Eigen::Ref<Eigen::MatrixXd> out) const override;

private:
  std::shared_ptr<const FrameKinematics> fk_;
  Eigen::Matrix3d target_;
  Eigen::Vector3d tolerance_;
};

}  // namespace pcp_direct

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_benchmark/constraints.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/orientation_constraint.hpp>

namespace pcp_benchmark
{

ConstraintType constraintTypeFromString(const std::string & name)
{
  if (name == "none") {
    return ConstraintType::kNone;
  }
  if (name == "orientation") {
    return ConstraintType::kOrientation;
  }
  throw std::invalid_argument("unknown constraint type '" + name + "' (expected none|orientation)");
}

std::string toString(ConstraintType type)
{
  switch (type) {
    case ConstraintType::kNone:
      return "none";
    case ConstraintType::kOrientation:
      return "orientation";
  }
  return "unknown";
}

double circularMean(double a, double b)
{
  return std::atan2(std::sin(a) + std::sin(b), std::cos(a) + std::cos(b));
}

double perAxisToleranceRad(double max_tilt_deg)
{
  return max_tilt_deg * M_PI / 180.0 / std::sqrt(2.0);
}

moveit_msgs::msg::Constraints buildPathConstraints(ConstraintType type, const ConstraintSpec & spec)
{
  moveit_msgs::msg::Constraints constraints;
  if (type == ConstraintType::kNone) {
    return constraints;
  }
  constraints.name = "keep_upright";
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = spec.frame_id;
  oc.link_name = spec.link_name;
  // Tool z along world -z with the given yaw: Rz(yaw) * Rx(pi). For yaw 0 the quaternion
  // (x, y, z, w) is (1, 0, 0, 0).
  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(spec.target_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  oc.orientation.x = q.x();
  oc.orientation.y = q.y();
  oc.orientation.z = q.z();
  oc.orientation.w = q.w();
  const double tol = perAxisToleranceRad(spec.max_tilt_deg);
  oc.absolute_x_axis_tolerance = tol;
  oc.absolute_y_axis_tolerance = tol;
  oc.absolute_z_axis_tolerance = M_PI;
  oc.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
  oc.weight = 1.0;
  constraints.orientation_constraints.push_back(oc);
  return constraints;
}

}  // namespace pcp_benchmark

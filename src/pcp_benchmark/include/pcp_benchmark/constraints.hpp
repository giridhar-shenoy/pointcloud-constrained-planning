// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#pragma once

#include <string>

#include <moveit_msgs/msg/constraints.hpp>

namespace pcp_benchmark
{

/// Path-constraint families the benchmark knows about. Only these two are in v0.1; line and box
/// position constraints are listed under "What's next" in the README.
enum class ConstraintType
{
  kNone,
  kOrientation,
};

ConstraintType constraintTypeFromString(const std::string & name);
std::string toString(ConstraintType type);

struct ConstraintSpec
{
  std::string frame_id = "world";   ///< frame the target orientation is expressed in
  std::string link_name = "tool0";  ///< constrained link
  /// Maximum angle between the link's z axis and world -z ("keep a cup upright"), in degrees.
  double max_tilt_deg = 5.0;
};

/// Per-axis rotation-vector tolerance (rad) that guarantees a tilt of at most `max_tilt_deg`.
///
/// MoveIt (both OMPL's constrained state space and kinematic_constraints with ROTATION_VECTOR)
/// bounds each component of the rotation vector r of R_target^-1 * R_link independently. The tilt
/// of the link z axis is at most |(r_x, r_y)|, so |r_x|, |r_y| <= max_tilt / sqrt(2) keeps the tilt
/// inside the cone. The feasible set is the square inscribed in the cone: sound, slightly
/// conservative (a pure 4.9 deg tilt about x alone is rejected).
double perAxisToleranceRad(double max_tilt_deg);

/// Build the path constraints for one benchmark cell. kNone returns an empty message.
/// kOrientation: target = link z along world -z with yaw 0, x/y tolerance from
/// perAxisToleranceRad(), z (yaw) tolerance pi, i.e. yaw is free.
moveit_msgs::msg::Constraints buildPathConstraints(
  ConstraintType type, const ConstraintSpec & spec = ConstraintSpec());

}  // namespace pcp_benchmark

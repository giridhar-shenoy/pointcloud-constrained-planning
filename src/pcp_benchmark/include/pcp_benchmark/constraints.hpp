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
  /// Yaw (rad, about world z) of the reference orientation. Yaw itself is free, but the rotation
  /// vector couples yaw and tilt: at a yaw offset psi from the reference, |r_xy| ~= tilt *
  /// (psi/2) / sin(psi/2), so the admissible tilt shrinks (to ~2.25 deg at psi = pi). The runner
  /// sets this to the circular mean of the query's start and goal yaws (|psi| <= pi/2 at both
  /// ends, factor <= 1.11).
  double target_yaw = 0.0;
};

/// Circular mean of two angles (rad), in (-pi, pi].
double circularMean(double a, double b);

/// Per-axis rotation-vector tolerance (rad) that guarantees a tilt of at most `max_tilt_deg`.
///
/// MoveIt (both OMPL's constrained state space and kinematic_constraints with ROTATION_VECTOR)
/// bounds each component of the rotation vector r of R_target^-1 * R_link independently. The tilt
/// of the link z axis is at most |(r_x, r_y)|, so |r_x|, |r_y| <= max_tilt / sqrt(2) keeps the tilt
/// inside the cone. The feasible set is the square inscribed in the cone: sound, slightly
/// conservative (a pure 4.9 deg tilt about x alone is rejected).
double perAxisToleranceRad(double max_tilt_deg);

/// Build the path constraints for one benchmark cell. kNone returns an empty message.
/// kOrientation: target = link z along world -z with yaw `spec.target_yaw`, x/y tolerance from
/// perAxisToleranceRad(), z (yaw) tolerance pi, i.e. yaw is free.
moveit_msgs::msg::Constraints buildPathConstraints(
  ConstraintType type, const ConstraintSpec & spec = ConstraintSpec());

}  // namespace pcp_benchmark

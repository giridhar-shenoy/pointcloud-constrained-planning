// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#pragma once

#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>

namespace pcp_benchmark
{

using JointPath = std::vector<std::vector<double>>;  ///< waypoints, each in group variable order

struct PathMetrics
{
  double joint_path_rad = 0.0;  ///< sum of L2 norms of consecutive joint-space segments
  double tcp_path_m = 0.0;      ///< Cartesian length of the tip-link trajectory
  double max_tilt_deg = 0.0;    ///< max angle between tip z and world -z along the path
  std::size_t n_waypoints = 0;
  std::size_t n_dense_states = 0;
};

/// Insert linearly interpolated states so that no joint moves more than `max_step_rad` between
/// consecutive states. This is what a joint trajectory controller would execute between
/// waypoints, so Cartesian metrics on the dense path are what the robot actually does.
JointPath densify(const JointPath & path, double max_step_rad);

/// Joint-space length: sum over segments of the L2 norm of q_{i+1} - q_i (rad).
double jointPathLength(const JointPath & path);

/// Angle (deg) between the z axis of `rotation` and world -z.
double tiltFromDownDeg(const Eigen::Matrix3d & rotation);

/// Metrics for a joint path of `group`, evaluated on the path densified to `max_step_rad`.
PathMetrics computePathMetrics(
  const moveit::core::RobotModelConstPtr & model, const std::string & group,
  const std::string & tip_link, const JointPath & path, double max_step_rad = 0.01);

}  // namespace pcp_benchmark

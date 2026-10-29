// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Time parameterisation of a joint path with the joint-synchronised, rest-to-rest trapezoidal
// profiles of traj/trapezoidal.hpp (copied from R1), sampled into a JointTrajectory message.
#pragma once

#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "pcp_direct/planner.hpp"

namespace pcp_direct
{

struct JointLimits
{
  std::vector<std::string> joint_names;
  std::vector<double> max_velocity;      ///< rad/s
  std::vector<double> max_acceleration;  ///< rad/s^2
};

/// Read `joint_limits.<joint>.{max_velocity,max_acceleration}` for `joint_names`, in that order.
JointLimits loadJointLimits(
  const std::string & yaml_path, const std::vector<std::string> & joint_names);

struct ParameterisedTrajectory
{
  trajectory_msgs::msg::JointTrajectory msg;
  double duration_s = 0.0;    ///< time_from_start of the last point
  double param_time_s = 0.0;  ///< wall time of profile construction + sampling
};

/// Every path segment becomes one synchronised rest-to-rest trapezoid at the shortest duration
/// the limits allow. Each segment is sampled at 3*m equal steps (m = ceil(T / (3 dt))) so the
/// 1/3-rule phase boundaries are sample points; points carry positions, velocities and
/// accelerations (the controller then interpolates them with quintic splines).
ParameterisedTrajectory parameterise(
  const JointPath & path, const JointLimits & limits, double sample_dt = 0.1);

}  // namespace pcp_direct

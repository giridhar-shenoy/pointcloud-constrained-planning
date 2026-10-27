// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#pragma once

#include <string>

#include <moveit/robot_model/robot_model.hpp>

namespace pcp_benchmark
{

std::string readFile(const std::string & path);

/// Build a RobotModel straight from URDF and SRDF files (no ROS parameters, no kinematics
/// plugins). Used by the query generator and the unit tests.
moveit::core::RobotModelPtr loadRobotModel(
  const std::string & urdf_path, const std::string & srdf_path);

}  // namespace pcp_benchmark

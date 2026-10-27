// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>

namespace pcp_benchmark
{

struct Box
{
  std::string name;
  Eigen::Vector3d size;      ///< full extents (m)
  Eigen::Vector3d position;  ///< centre in the scene frame (m)
};

struct Region
{
  std::string name;
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

/// The shelf as described by pcp_scenes/config/shelf.yaml (the same file the Gazebo world is
/// generated from).
struct ShelfScene
{
  std::string frame = "world";
  std::vector<Box> boxes;
  std::vector<Region> target_regions;
};

ShelfScene loadShelfScene(const std::string & yaml_path);

/// Add every shelf box, inflated by `padding` metres on each side, as a world collision object.
void addShelfToPlanningScene(
  const ShelfScene & shelf, planning_scene::PlanningScene & scene, double padding);

}  // namespace pcp_benchmark

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_benchmark/shelf_scene.hpp"

#include <stdexcept>

#include <geometric_shapes/shapes.h>
#include <yaml-cpp/yaml.h>

namespace pcp_benchmark
{
namespace
{
Eigen::Vector3d vec3(const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("shelf.yaml: expected a 3-element list");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}
}  // namespace

ShelfScene loadShelfScene(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  ShelfScene shelf;
  shelf.frame = root["frame"].as<std::string>("world");
  for (const auto & b : root["boxes"]) {
    shelf.boxes.push_back({b["name"].as<std::string>(), vec3(b["size"]), vec3(b["position"])});
  }
  for (const auto & r : root["target_regions"]) {
    shelf.target_regions.push_back({r["name"].as<std::string>(), vec3(r["min"]), vec3(r["max"])});
  }
  if (shelf.boxes.empty() || shelf.target_regions.empty()) {
    throw std::runtime_error("shelf.yaml: needs at least one box and one target region");
  }
  return shelf;
}

void addShelfToPlanningScene(
  const ShelfScene & shelf, planning_scene::PlanningScene & scene, double padding)
{
  auto world = scene.getWorldNonConst();
  for (const auto & box : shelf.boxes) {
    const Eigen::Vector3d size = box.size + Eigen::Vector3d::Constant(2.0 * padding);
    auto shape = std::make_shared<shapes::Box>(size.x(), size.y(), size.z());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = box.position;
    world->addToObject(box.name, shape, pose);
  }
}

}  // namespace pcp_benchmark

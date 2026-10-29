// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Direct collision checker: Pinocchio + coal on the same URDF, SRDF and shelf boxes that MoveIt
// uses. No MoveIt code is linked into this library.
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace octomap
{
class OcTree;
}

namespace pcp_direct
{

/// One axis-aligned box of the shelf, as in pcp_scenes/config/shelf.yaml.
struct BoxSpec
{
  std::string name;
  Eigen::Vector3d size;      ///< full extents (m)
  Eigen::Vector3d position;  ///< centre in the world frame (m)
};

/// Read the `boxes:` list of a shelf YAML (the file the Gazebo world is generated from).
std::vector<BoxSpec> loadBoxes(const std::string & yaml_path);

/// A colliding or closest pair, named the way MoveIt names it: link names for robot geometry,
/// the box name for world boxes. `first` < `second` lexicographically.
using NamedPair = std::pair<std::string, std::string>;

struct PairDistance
{
  NamedPair pair;
  double distance = 0.0;  ///< coal's distance (m); <= 0 when the pair is in contact
};

/// Pinocchio model + GeometryModel of the robot's collision meshes and the shelf boxes.
///
/// Pairs: `addAllCollisionPairs()` (every pair of geometries on different joints, so geometries
/// fixed to the universe — base, ground plane, boxes — are never paired with each other), then the
/// SRDF's `<disable_collisions>` pairs are removed. That is the pair set MoveIt's OMPL validity
/// checker tests with `group_name` = the arm group (FCL skips pairs where neither link moves with
/// the group). No padding, no security margin.
class CollisionChecker
{
public:
  /// @param urdf_xml     the robot_description string (meshes as file:// or package:// URIs)
  /// @param srdf_xml     the robot_description_semantic string
  /// @param boxes        world boxes, attached to the universe
  /// @param joint_names  the order in which configurations are passed to this class
  /// @param package_dirs directories searched for package:// meshes (share/.. of each package)
  /// @param octree       optional occupancy map (e.g. the one MoveIt built), added as one
  ///                     coal::OcTree on the universe; named "<octomap>" like MoveIt's object
  CollisionChecker(
    const std::string & urdf_xml, const std::string & srdf_xml, const std::vector<BoxSpec> & boxes,
    const std::vector<std::string> & joint_names,
    const std::vector<std::string> & package_dirs = {},
    std::shared_ptr<const octomap::OcTree> octree = nullptr);
  ~CollisionChecker();
  CollisionChecker(const CollisionChecker &) = delete;
  CollisionChecker & operator=(const CollisionChecker &) = delete;

  /// True if no active pair collides (`computeCollisions`, stop at the first collision). Counts
  /// the call and records its wall time. This is the planner's validity check.
  bool isValid(const double * q);
  bool isValid(const std::vector<double> & q) { return isValid(q.data()); }

  /// Same test without touching the statistics (for tests and cross-checks).
  bool isColliding(const std::vector<double> & q) const;

  /// Every active pair in collision (no early stop).
  std::vector<NamedPair> collidingPairs(const std::vector<double> & q) const;

  /// Minimum coal distance over all active pairs and the pair that attains it.
  PairDistance minimumDistance(const std::vector<double> & q) const;

  /// Distance of one named pair (throws if the pair is not active).
  double pairDistance(const std::vector<double> & q, const NamedPair & pair) const;

  std::size_t activePairCount() const;
  std::size_t srdfRemovedPairCount() const { return srdf_removed_; }
  std::vector<NamedPair> activePairs() const;
  std::size_t geometryCount() const;

  const std::vector<std::string> & jointNames() const { return joint_names_; }
  const std::vector<double> & lowerLimits() const { return lower_; }
  const std::vector<double> & upperLimits() const { return upper_; }

  // Statistics of isValid() since the last reset.
  std::size_t calls() const { return call_times_us_.size(); }
  double medianCallTimeUs() const;
  double totalCallTimeS() const;
  void resetStatistics() { call_times_us_.clear(); }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::size_t srdf_removed_ = 0;
  std::vector<float> call_times_us_;
};

/// Read a whole file into a string (throws if it cannot be opened).
std::string readTextFile(const std::string & path);

}  // namespace pcp_direct

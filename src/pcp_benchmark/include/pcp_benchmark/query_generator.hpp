// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <nlohmann/json.hpp>

#include "pcp_benchmark/shelf_scene.hpp"

namespace pcp_benchmark
{

/// Platform-independent RNG: std::mt19937_64's output sequence is fixed by the C++ standard, but
/// std::uniform_real_distribution is not, so the conversions to doubles are done here.
class DeterministicRng
{
public:
  explicit DeterministicRng(std::uint64_t seed) : engine_(seed) {}
  double uniform();  ///< [0, 1) with 53 random bits
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  std::size_t index(std::size_t n);  ///< uniform in [0, n)

private:
  std::mt19937_64 engine_;
};

struct GeneratorConfig
{
  std::uint64_t seed = 0;
  std::size_t count = 100;
  std::string group = "ur_manipulator";
  std::string tip_link = "tool0";
  /// Shelf boxes are inflated by this much for the validity check, so that start and goal keep
  /// clearance from the octomap (2 cm voxels can stick out of the true surface by one voxel).
  double collision_padding = 0.03;
  /// Start and goal tool positions at least this far apart (m), so no query is trivial.
  double min_tcp_distance = 0.10;
  int ik_seeds = 8;             ///< IK seeds tried per sampled pose
  int ik_max_iterations = 300;  ///< fixed budget per seed; no wall-clock timeouts
  double ik_damping = 0.05;     ///< damped-least-squares lambda
  double ik_tolerance = 1e-6;   ///< on position error (m) and rotation error (rad)
  int max_pose_samples = 500;   ///< per state, before giving up
};

struct Query
{
  std::size_t id = 0;
  std::string start_region;
  std::string goal_region;
  std::vector<double> start;  ///< joint positions, group variable order
  std::vector<double> goal;
  Eigen::Vector4d start_tcp;  ///< x, y, z (m), yaw (rad) of the sampled tool pose
  Eigen::Vector4d goal_tcp;
};

struct QuerySet
{
  GeneratorConfig config;
  std::string scene_name = "shelf";
  std::vector<std::string> joint_names;
  std::vector<Query> queries;
  std::size_t pose_samples = 0;       ///< tool poses drawn
  std::size_t ik_failures = 0;        ///< poses with no converged IK solution in limits
  std::size_t collision_rejects = 0;  ///< converged solutions in collision (padded shelf, self)
};

/// Damped-least-squares IK on MoveIt's FK/Jacobian from the state's current joint values.
/// Deterministic: fixed iteration budget, no random restarts. Returns true if both position and
/// orientation errors fall below `cfg.ik_tolerance`.
bool solveIkDls(
  moveit::core::RobotState & state, const moveit::core::JointModelGroup * group,
  const moveit::core::LinkModel * tip, const Eigen::Isometry3d & target,
  const GeneratorConfig & cfg);

/// Tool-down pose (tool z along world -z) at `position` with the given yaw about world z.
Eigen::Isometry3d toolDownPose(const Eigen::Vector3d & position, double yaw);

/// Generate `cfg.count` queries: start and goal are IK solutions for tool-down poses sampled
/// uniformly inside the shelf's target regions, collision-free against the padded shelf and
/// self-collision. Same model + scene + config => identical output.
QuerySet generateQueries(
  const moveit::core::RobotModelConstPtr & model, const ShelfScene & shelf,
  const GeneratorConfig & cfg);

nlohmann::json toJson(const QuerySet & set);
QuerySet queriesFromJson(const nlohmann::json & json);

/// Round to 1e-6 so the JSON is stable and diffable.
double round6(double value);

}  // namespace pcp_benchmark

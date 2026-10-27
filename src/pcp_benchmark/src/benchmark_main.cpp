// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// pcp_benchmark: plans every (query x constraint type x planner x repetition) cell with
// MoveItCpp + PlanningComponent against ONE frozen snapshot of the planning scene (octomap from
// the simulated depth camera, as published by move_group on /monitored_planning_scene), and
// writes one CSV row per cell. Parameters: config/benchmark.yaml (all under `bench.`).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometric_shapes/shapes.h>
#include <ompl/util/RandomNumbers.h>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pcp_benchmark/constraints.hpp"
#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"

namespace
{
const rclcpp::Logger kLog = rclcpp::get_logger("pcp_benchmark");

struct BenchParams
{
  std::string queries_file;
  std::string output_csv;
  std::string commit = "unknown";
  std::string pipeline = "ompl";
  std::string group = "ur_manipulator";
  std::string tip_link = "tool0";
  std::string mode = "constrained_state_space";
  std::vector<std::string> planners = {"RRTConnectkConfigDefault"};
  std::vector<std::string> constraints = {"none", "orientation"};
  double planning_time = 10.0;
  double max_tilt_deg = 5.0;
  double octomap_timeout = 180.0;
  double octomap_settle = 3.0;
  int64_t first_query = 0;
  int64_t max_queries = -1;
  int64_t repetitions = 1;
  int64_t ompl_seed = 0;
};

BenchParams loadParams(const rclcpp::Node::SharedPtr & node)
{
  BenchParams p;
  node->get_parameter("bench.queries_file", p.queries_file);
  node->get_parameter("bench.output_csv", p.output_csv);
  node->get_parameter("bench.commit", p.commit);
  node->get_parameter("bench.pipeline", p.pipeline);
  node->get_parameter("bench.group", p.group);
  node->get_parameter("bench.tip_link", p.tip_link);
  node->get_parameter("bench.mode", p.mode);
  node->get_parameter("bench.planners", p.planners);
  node->get_parameter("bench.constraints", p.constraints);
  node->get_parameter("bench.planning_time", p.planning_time);
  node->get_parameter("bench.max_tilt_deg", p.max_tilt_deg);
  node->get_parameter("bench.octomap_timeout", p.octomap_timeout);
  node->get_parameter("bench.octomap_settle", p.octomap_settle);
  node->get_parameter("bench.first_query", p.first_query);
  node->get_parameter("bench.max_queries", p.max_queries);
  node->get_parameter("bench.repetitions", p.repetitions);
  node->get_parameter("bench.ompl_seed", p.ompl_seed);
  if (p.queries_file.empty() || p.output_csv.empty()) {
    throw std::runtime_error("bench.queries_file and bench.output_csv are required");
  }
  return p;
}

/// Occupied voxels at the finest resolution (pruned inner leaves count as 8^levels voxels).
std::size_t occupiedVoxels(const planning_scene::PlanningScene & scene)
{
  const auto object = scene.getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
  if (!object || object->shapes_.empty()) {
    return 0;
  }
  const auto shape = std::dynamic_pointer_cast<const shapes::OcTree>(object->shapes_[0]);
  if (!shape || !shape->octree) {
    return 0;
  }
  const auto & tree = *shape->octree;
  const unsigned max_depth = tree.getTreeDepth();
  std::size_t count = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (tree.isNodeOccupied(*it)) {
      count += std::size_t{1} << (3 * (max_depth - it.getDepth()));
    }
  }
  return count;
}

std::string utcNow()
{
  const std::time_t t = std::time(nullptr);
  std::ostringstream ss;
  ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
}

std::string fixedStr(double v, int precision = 6)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << v;
  return ss.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pcp_benchmark", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int exit_code = 0;
  // Outlives the try block so it is never destroyed while the executor spins its callbacks.
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit;

  try {
    const BenchParams p = loadParams(node);
    // Seed OMPL before MoveItCpp loads the planner plugin. Runs are still not bit-identical:
    // planners stop on wall-clock time and MoveIt may plan in worker threads.
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(p.ompl_seed));

    const auto query_set = pcp_benchmark::queriesFromJson(
      nlohmann::json::parse(pcp_benchmark::readFile(p.queries_file)));

    moveit = std::make_shared<moveit_cpp::MoveItCpp>(node);
    auto psm = moveit->getPlanningSceneMonitorNonConst();
    const auto model = moveit->getRobotModel();
    const auto * jmg = model->getJointModelGroup(p.group);
    if (jmg == nullptr || jmg->getVariableNames() != query_set.joint_names) {
      throw std::runtime_error("group variables do not match the joint order in queries.json");
    }

    // Wait for move_group's octomap to arrive and stop changing, then freeze one copy.
    RCLCPP_INFO(kLog, "waiting for a settled octomap on /monitored_planning_scene ...");
    const auto wait_start = std::chrono::steady_clock::now();
    std::size_t voxels = 0;
    std::size_t last = 0;
    auto stable_since = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      {
        planning_scene_monitor::LockedPlanningSceneRO locked(psm);
        const planning_scene::PlanningSceneConstPtr & scene = locked;
        voxels = occupiedVoxels(*scene);
      }
      const auto now = std::chrono::steady_clock::now();
      if (voxels != last) {
        last = voxels;
        stable_since = now;
      }
      const double stable_s = std::chrono::duration<double>(now - stable_since).count();
      if (voxels > 0 && stable_s >= p.octomap_settle) {
        break;
      }
      if (std::chrono::duration<double>(now - wait_start).count() > p.octomap_timeout) {
        throw std::runtime_error("no settled octomap within bench.octomap_timeout");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    planning_scene::PlanningScenePtr snapshot;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm);
      snapshot = planning_scene::PlanningScene::clone(scene);
    }
    voxels = occupiedVoxels(*snapshot);
    RCLCPP_INFO(kLog, "frozen planning scene: octomap with %zu occupied voxels", voxels);

    std::ofstream csv(p.output_csv);
    csv << "commit,query_seed,ompl_seed,query_id,start_region,goal_region,planner,constraint,mode,"
           "rep,success,error_code,planning_time_s,wall_time_s,joint_path_rad,tcp_path_m,"
           "max_tilt_deg,n_waypoints,start_valid,goal_valid,octomap_voxels\n";

    moveit_cpp::PlanningComponent planning(p.group, moveit);
    pcp_benchmark::ConstraintSpec spec;
    spec.link_name = p.tip_link;
    spec.max_tilt_deg = p.max_tilt_deg;

    const std::size_t first = static_cast<std::size_t>(std::max<int64_t>(0, p.first_query));
    std::size_t last_query = query_set.queries.size();
    if (p.max_queries >= 0) {
      last_query = std::min(last_query, first + static_cast<std::size_t>(p.max_queries));
    }
    const auto bench_start = std::chrono::steady_clock::now();
    std::size_t cells = 0;
    std::size_t successes = 0;
    for (std::size_t qi = first; qi < last_query && rclcpp::ok(); ++qi) {
      const auto & q = query_set.queries[qi];
      moveit::core::RobotState start(model);
      start.setToDefaultValues();
      start.setJointGroupPositions(jmg, q.start);
      start.update();
      moveit::core::RobotState goal(start);
      goal.setJointGroupPositions(jmg, q.goal);
      goal.update();
      const bool start_valid = !snapshot->isStateColliding(start, p.group);
      const bool goal_valid = !snapshot->isStateColliding(goal, p.group);

      for (const auto & constraint_name : p.constraints) {
        const auto type = pcp_benchmark::constraintTypeFromString(constraint_name);
        for (const auto & planner : p.planners) {
          for (int64_t rep = 0; rep < p.repetitions; ++rep) {
            planning.setStartState(start);
            planning.setGoal(goal);
            planning.setPathConstraints(pcp_benchmark::buildPathConstraints(type, spec));
            moveit_cpp::PlanningComponent::PlanRequestParameters params;
            params.planner_id = planner;
            params.planning_pipeline = p.pipeline;
            params.planning_attempts = 1;
            params.planning_time = p.planning_time;
            params.max_velocity_scaling_factor = 1.0;
            params.max_acceleration_scaling_factor = 1.0;

            const auto t0 = std::chrono::steady_clock::now();
            const auto response = planning.plan(params, snapshot);
            const double wall =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

            const bool success = static_cast<bool>(response.error_code) && response.trajectory;
            pcp_benchmark::PathMetrics metrics;
            if (success) {
              pcp_benchmark::JointPath path;
              for (std::size_t w = 0; w < response.trajectory->getWayPointCount(); ++w) {
                std::vector<double> qw;
                response.trajectory->getWayPoint(w).copyJointGroupPositions(jmg, qw);
                path.push_back(std::move(qw));
              }
              metrics = pcp_benchmark::computePathMetrics(model, p.group, p.tip_link, path);
              ++successes;
            }
            ++cells;
            csv << p.commit << ',' << query_set.config.seed << ',' << p.ompl_seed << ',' << q.id
                << ',' << q.start_region << ',' << q.goal_region << ',' << planner << ','
                << constraint_name << ',' << p.mode << ',' << rep << ',' << (success ? 1 : 0) << ','
                << response.error_code.val << ',' << fixedStr(response.planning_time) << ','
                << fixedStr(wall) << ',' << (success ? fixedStr(metrics.joint_path_rad) : "") << ','
                << (success ? fixedStr(metrics.tcp_path_m) : "") << ','
                << (success ? fixedStr(metrics.max_tilt_deg, 3) : "") << ','
                << (success ? std::to_string(metrics.n_waypoints) : "") << ','
                << (start_valid ? 1 : 0) << ',' << (goal_valid ? 1 : 0) << ',' << voxels << '\n';
            csv.flush();
            RCLCPP_INFO(
              kLog, "query %zu %s %s rep %ld: %s in %.3f s", q.id, constraint_name.c_str(),
              planner.c_str(), static_cast<long>(rep), success ? "OK" : "FAIL", wall);
          }
        }
      }
    }
    const double total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - bench_start).count();
    RCLCPP_INFO(kLog, "done: %zu cells, %zu successes, %.1f s total", cells, successes, total);

    // Side-car metadata so every CSV can be traced to its inputs.
    nlohmann::json meta = {
      {"commit", p.commit},
      {"finished_utc", utcNow()},
      {"queries_file", p.queries_file},
      {"query_seed", query_set.config.seed},
      {"ompl_seed", p.ompl_seed},
      {"planners", p.planners},
      {"constraints", p.constraints},
      {"mode", p.mode},
      {"planning_time_s", p.planning_time},
      {"repetitions", p.repetitions},
      {"first_query", first},
      {"queries_run", last_query - first},
      {"cells", cells},
      {"successes", successes},
      {"total_wall_time_s", total},
      {"octomap_voxels", voxels},
      {"octomap_wait_s", std::chrono::duration<double>(bench_start - wait_start).count()},
      {"orientation_per_axis_tolerance_rad", pcp_benchmark::perAxisToleranceRad(p.max_tilt_deg)},
      {"hardware_threads", std::thread::hardware_concurrency()},
    };
    std::ofstream(p.output_csv + ".meta.json") << meta.dump(2) << "\n";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLog, "%s", e.what());
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  // MoveIt 2.15.2's ~MoveItCpp segfaults here (exit code -11, after "Deleting MoveItCpp"), even
  // with the executor stopped. The CSV and metadata files are closed by now, so skip the
  // destructor instead of turning a finished benchmark into a crashed process.
  std::fflush(nullptr);
  std::_Exit(exit_code);
}

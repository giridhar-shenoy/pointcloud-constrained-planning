// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// bench_direct: the same queries, the same box shelf, the same planner settings and (optionally)
// the same controller, planned by two pipelines:
//   direct — pcp_direct: OMPL RRTConnect on the Pinocchio/coal checker, OMPL's simplifier,
//            R1's trapezoidal parameterisation, a FollowJointTrajectory client;
//   moveit — MoveItCpp PlanningComponent::plan() on a frozen planning scene (OMPL pipeline with
//            its request/response adapters, TOTG), MoveItCpp::execute() (trajectory execution
//            manager).
// Every path is densely re-checked by the OTHER pipeline's checker (cross-validation). With
// execution on, the direct planner moves the arm to each run's start first.
//
// Parameters: config/bench_direct.yaml (all under `bench.`).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include <ompl/util/RandomNumbers.h>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/ompl_interface/detail/state_validity_checker.hpp>
#include <moveit/ompl_interface/model_based_planning_context.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pcp_benchmark/constraints.hpp"
#include "pcp_benchmark/metrics.hpp"
#include "pcp_benchmark/query_generator.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/constraint.hpp"
#include "pcp_direct/exec.hpp"
#include "pcp_direct/moveit_reference.hpp"
#include "pcp_direct/planner.hpp"
#include "pcp_direct/traj.hpp"

namespace
{
const rclcpp::Logger kLog = rclcpp::get_logger("bench_direct");

struct Params
{
  std::string queries_file;
  std::string shelf_yaml;
  std::string ompl_yaml;
  std::string joint_limits_yaml;
  std::string output_csv;
  std::string commit = "unknown";
  std::string group = "ur_manipulator";
  std::string tip_link = "tool0";
  std::string pipeline = "ompl";
  std::string action_name = "/joint_trajectory_controller/follow_joint_trajectory";
  std::string controller_state_topic = "/joint_trajectory_controller/controller_state";
  bool execute = false;
  double planning_time = 10.0;
  double sample_dt = 0.1;
  double goal_tolerance = 0.01;
  double dense_step = 0.01;
  double exec_timeout_margin = 30.0;
  double scene_timeout = 60.0;
  int64_t first_query = 0;
  int64_t max_queries = -1;
  int64_t ompl_seed = 1;
  int64_t probe_queries = 0;        ///< queries for the MoveIt validity-check probe (0 = off)
  std::string constraint = "none";  ///< none | orientation (keep-upright, v0.1's definition)
  double max_tilt_deg = 5.0;
};

Params loadParams(const rclcpp::Node::SharedPtr & node)
{
  Params p;
  auto get = [&](const std::string & name, auto & value) {
    node->get_parameter("bench." + name, value);
  };
  get("queries_file", p.queries_file);
  get("shelf_yaml", p.shelf_yaml);
  get("ompl_yaml", p.ompl_yaml);
  get("joint_limits_yaml", p.joint_limits_yaml);
  get("output_csv", p.output_csv);
  get("commit", p.commit);
  get("group", p.group);
  get("tip_link", p.tip_link);
  get("pipeline", p.pipeline);
  get("action_name", p.action_name);
  get("controller_state_topic", p.controller_state_topic);
  get("execute", p.execute);
  get("planning_time", p.planning_time);
  get("sample_dt", p.sample_dt);
  get("goal_tolerance", p.goal_tolerance);
  get("dense_step", p.dense_step);
  get("exec_timeout_margin", p.exec_timeout_margin);
  get("scene_timeout", p.scene_timeout);
  get("first_query", p.first_query);
  get("max_queries", p.max_queries);
  get("ompl_seed", p.ompl_seed);
  get("probe_queries", p.probe_queries);
  get("constraint", p.constraint);
  get("max_tilt_deg", p.max_tilt_deg);
  for (const auto * v :
       {&p.queries_file, &p.shelf_yaml, &p.ompl_yaml, &p.joint_limits_yaml, &p.output_csv}) {
    if (v->empty()) {
      throw std::runtime_error(
        "bench.queries_file, shelf_yaml, ompl_yaml, joint_limits_yaml and output_csv are "
        "required");
    }
  }
  return p;
}

std::string fx(double v, int precision = 6)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << v;
  return ss.str();
}

std::string utcNow()
{
  const std::time_t t = std::time(nullptr);
  std::ostringstream ss;
  ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
}

double maxAbsDiff(const std::vector<double> & a, const std::vector<double> & b)
{
  double d = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {
    d = std::max(d, std::abs(a[j] - b[j]));
  }
  return d;
}

/// What MoveIt's OMPL pipeline actually configures for a request: read from the planning context
/// it builds (state-space extent, collision-check spacing, RRTConnect range, validator), so the
/// "same settings" claim is measured rather than assumed.
nlohmann::json inspectMoveItOmplContext(
  const std::shared_ptr<moveit_cpp::MoveItCpp> & moveit,
  const planning_scene::PlanningScenePtr & scene, const std::string & pipeline,
  const std::string & group, const std::string & planner_id, double planning_time,
  const moveit::core::RobotState & start, const moveit::core::RobotState & goal)
{
  const auto & pipelines = moveit->getPlanningPipelines();
  const auto it = pipelines.find(pipeline);
  if (it == pipelines.end()) {
    throw std::runtime_error("no planning pipeline " + pipeline);
  }
  const auto manager = it->second->getPlannerManager("ompl_interface/OMPLPlanner");
  if (!manager) {
    throw std::runtime_error("no OMPL planner manager in pipeline " + pipeline);
  }
  planning_interface::MotionPlanRequest req;
  req.group_name = group;
  req.pipeline_id = pipeline;
  req.planner_id = planner_id;
  req.allowed_planning_time = planning_time;
  req.num_planning_attempts = 1;
  moveit::core::robotStateToRobotStateMsg(start, req.start_state);
  req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(
      goal, goal.getJointModelGroup(group), std::numeric_limits<double>::epsilon()));
  moveit_msgs::msg::MoveItErrorCodes ec;
  const auto ctx = manager->getPlanningContext(scene, req, ec);
  const auto mbpc = std::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(ctx);
  if (!mbpc) {
    throw std::runtime_error("MoveIt's OMPL planning context could not be inspected");
  }
  const auto & ss = mbpc->getOMPLSimpleSetup();
  const auto si = ss->getSpaceInformation();
  const auto space = si->getStateSpace();
  nlohmann::json j = {
    {"state_space", space->getName()},
    {"max_extent_rad", space->getMaximumExtent()},
    {"longest_valid_segment_fraction", space->getLongestValidSegmentFraction()},
    {"longest_valid_segment_rad", space->getLongestValidSegmentLength()},
    {"state_validity_checking_resolution", si->getStateValidityCheckingResolution()},
    {"motion_validator", si->getMotionValidator() ? typeid(*si->getMotionValidator()).name() : ""},
  };
  if (ss->getPlanner()) {
    std::string range;
    ss->getPlanner()->params().getParam("range", range);
    j["planner"] = ss->getPlanner()->getName();
    j["range"] = range;
  }
  return j;
}

/// MoveIt's own OMPL state validity checker, counting and timing its calls (probe only). It must
/// BE MoveIt's checker type: MoveIt static-casts the space's checker to it (a generic wrapper
/// crashed with SIGSEGV).
class CountingValidityChecker : public ompl_interface::StateValidityChecker
{
public:
  explicit CountingValidityChecker(const ompl_interface::ModelBasedPlanningContext * ctx)
  : ompl_interface::StateValidityChecker(ctx)
  {
  }
  bool isValid(const ompl::base::State * state, bool verbose) const override
  {
    const auto t0 = std::chrono::steady_clock::now();
    const bool v = ompl_interface::StateValidityChecker::isValid(state, verbose);
    times_us_.push_back(
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    return v;
  }
  std::size_t calls() const { return times_us_.size(); }
  double medianUs() const
  {
    if (times_us_.empty()) {
      return 0.0;
    }
    auto t = times_us_;
    std::nth_element(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(t.size() / 2), t.end());
    return t[t.size() / 2];
  }
  double totalS() const
  {
    double sum = 0.0;
    for (double v : times_us_) {
      sum += v;
    }
    return sum * 1e-6;
  }

private:
  mutable std::vector<double> times_us_;
};

/// Probe: solve one query in MoveIt's OMPL planning context (the one plan() would build) with its
/// validity checker wrapped, and report how many checks MoveIt's planner made and what they cost.
/// This bypasses the planning pipeline's adapters on purpose; it measures the planner core only.
nlohmann::json probeMoveItChecks(
  const std::shared_ptr<moveit_cpp::MoveItCpp> & moveit,
  const planning_scene::PlanningScenePtr & scene, const std::string & pipeline,
  const std::string & group, const std::string & planner_id, double planning_time,
  const moveit::core::RobotState & start, const moveit::core::RobotState & goal)
{
  const auto manager =
    moveit->getPlanningPipelines().at(pipeline)->getPlannerManager("ompl_interface/OMPLPlanner");
  planning_interface::MotionPlanRequest req;
  req.group_name = group;
  req.pipeline_id = pipeline;
  req.planner_id = planner_id;
  req.allowed_planning_time = planning_time;
  req.num_planning_attempts = 1;
  moveit::core::robotStateToRobotStateMsg(start, req.start_state);
  req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(
      goal, goal.getJointModelGroup(group), std::numeric_limits<double>::epsilon()));
  moveit_msgs::msg::MoveItErrorCodes ec;
  const auto ctx = std::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(
    manager->getPlanningContext(scene, req, ec));
  if (!ctx) {
    return {{"error", "no ModelBasedPlanningContext"}};
  }
  auto & ss = ctx->getOMPLSimpleSetup();
  // MoveIt caches planning contexts and its goal sampler runs in a background thread, so a
  // counter freed after this call could still be referenced (the first version crashed after a
  // dozen probes). Restore MoveIt's own checker afterwards and keep every counter alive.
  static std::vector<std::shared_ptr<CountingValidityChecker>> keep_alive;
  const auto original = ss->getStateValidityChecker();
  auto counting = std::make_shared<CountingValidityChecker>(ctx.get());
  keep_alive.push_back(counting);
  ss->setStateValidityChecker(counting);
  planning_interface::MotionPlanResponse res;
  const auto t0 = std::chrono::steady_clock::now();
  ctx->solve(res);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ss->setStateValidityChecker(original);
  return {
    {"success", static_cast<bool>(res.error_code)},
    {"planning_time_s", res.planning_time},
    {"plan_time_s", ctx->getLastPlanTime()},
    {"simplify_time_s", ctx->getLastSimplifyTime()},
    {"solve_wall_s", wall},
    {"validity_checks", counting->calls()},
    {"check_time_us_median", counting->medianUs()},
    {"check_time_total_s", counting->totalS()},
  };
}

/// One row of results/bench_direct.csv. Empty strings are "not applicable / not measured".
struct Row
{
  std::size_t query = 0;
  std::string pipeline;
  std::string success, plan_time_s, simplify_time_s, param_time_s, time_to_trajectory_s,
    exec_time_s, path_len_joint_rad, path_len_tcp_m, validity_checks, check_time_us_median,
    max_track_err_rad, crossval_ok;
  // extras (appended after the columns the brief lists)
  std::string status, plan_success, exec_success, traj_duration_s, n_waypoints, own_dense_ok,
    crossval_hits, moveit_error_code, final_err_rad, at_start, order, start_region, goal_region,
    checks_plan, simplify_budget_s, raw_states, exec_error, max_tilt_deg;
};

const char * kHeader =
  "query,pipeline,success,plan_time_s,simplify_time_s,param_time_s,time_to_trajectory_s,"
  "exec_time_s,path_len_joint_rad,path_len_tcp_m,validity_checks,check_time_us_median,"
  "max_track_err_rad,crossval_ok,commit,seed,"
  "status,plan_success,exec_success,traj_duration_s,n_waypoints,own_dense_ok,crossval_hits,"
  "moveit_error_code,final_err_rad,at_start,order,start_region,goal_region,checks_plan,"
  "simplify_budget_s,raw_states,exec_error,ompl_seed,executed,constraint,max_tilt_deg\n";

std::string csvSafe(std::string s)
{
  std::replace(s.begin(), s.end(), ',', ';');
  std::replace(s.begin(), s.end(), '\n', ' ');
  return s;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("bench_direct", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit;
  int exit_code = 0;

  try {
    const Params p = loadParams(node);
    // Seeds OMPL's RNG for the direct planner. MoveIt's state sampler uses its own
    // random_numbers generator, so MoveIt plans are not seed-reproducible (NOTES.md).
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(p.ompl_seed));

    const auto query_set = pcp_benchmark::queriesFromJson(
      nlohmann::json::parse(pcp_direct::readTextFile(p.queries_file)));
    const auto boxes = pcp_direct::loadBoxes(p.shelf_yaml);

    moveit = std::make_shared<moveit_cpp::MoveItCpp>(node);
    auto psm = moveit->getPlanningSceneMonitorNonConst();
    const auto model = moveit->getRobotModel();
    const auto * jmg = model->getJointModelGroup(p.group);
    if (jmg == nullptr || jmg->getVariableNames() != query_set.joint_names) {
      throw std::runtime_error("group variables do not match the joint order of the queries");
    }
    const auto & joint_names = query_set.joint_names;

    // Freeze MoveIt's scene once every shelf box has arrived from shelf_boxes_loader.
    planning_scene::PlanningScenePtr scene;
    const auto wait_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !scene) {
      {
        planning_scene_monitor::LockedPlanningSceneRO locked(psm);
        const planning_scene::PlanningSceneConstPtr & live = locked;
        std::size_t present = 0;
        for (const auto & b : boxes) {
          present += live->getWorld()->hasObject(b.name) ? 1 : 0;
        }
        if (present == boxes.size()) {
          scene = planning_scene::PlanningScene::clone(live);
        }
      }
      if (
        !scene &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() >
          p.scene_timeout) {
        throw std::runtime_error("shelf boxes did not arrive within bench.scene_timeout");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (scene->getWorld()->hasObject(planning_scene::PlanningScene::OCTOMAP_NS)) {
      throw std::runtime_error("the frozen scene contains an octomap; v0.2 runs on boxes only");
    }
    RCLCPP_INFO(
      kLog, "frozen MoveIt scene: %zu world objects", scene->getWorld()->getObjectIds().size());

    std::string urdf;
    std::string srdf;
    node->get_parameter("robot_description", urdf);
    node->get_parameter("robot_description_semantic", srdf);
    pcp_direct::CollisionChecker checker(urdf, srdf, boxes, joint_names);
    pcp_direct::PlannerSettings settings = pcp_direct::loadMoveItOmplSettings(p.ompl_yaml, p.group);
    settings.timeout_s = p.planning_time;
    settings.ompl_seed = static_cast<unsigned int>(p.ompl_seed);
    // The MoveIt pipeline loaded the same file through MoveItConfigsBuilder: confirm.
    double moveit_lvsf = -1.0;
    node->get_parameter(
      p.pipeline + "." + p.group + ".longest_valid_segment_fraction", moveit_lvsf);
    std::string moveit_default_cfg;
    node->get_parameter(p.pipeline + "." + p.group + ".default_planner_config", moveit_default_cfg);
    if (
      std::abs(moveit_lvsf - settings.longest_valid_segment_fraction) > 1e-15 ||
      moveit_default_cfg != settings.planner_config) {
      throw std::runtime_error("MoveIt's loaded OMPL parameters differ from " + p.ompl_yaml);
    }
    pcp_direct::DirectPlanner planner(checker, settings);
    nlohmann::json moveit_ompl = {{"error", "not inspected"}};
    {
      const auto & q0 = query_set.queries.at(0);
      moveit::core::RobotState s0(model);
      s0.setToDefaultValues();
      s0.setJointGroupPositions(jmg, q0.start);
      s0.update();
      moveit::core::RobotState g0(s0);
      g0.setJointGroupPositions(jmg, q0.goal);
      g0.update();
      try {
        moveit_ompl = inspectMoveItOmplContext(
          moveit, scene, p.pipeline, p.group, settings.planner_config, p.planning_time, s0, g0);
      } catch (const std::exception & e) {
        moveit_ompl = {{"error", e.what()}};
      }
      RCLCPP_INFO(kLog, "MoveIt's OMPL context: %s", moveit_ompl.dump().c_str());
    }
    const auto limits = pcp_direct::loadJointLimits(p.joint_limits_yaml, joint_names);
    const auto constraint_type = pcp_benchmark::constraintTypeFromString(p.constraint);
    pcp_benchmark::ConstraintSpec spec;
    spec.link_name = p.tip_link;
    spec.max_tilt_deg = p.max_tilt_deg;
    const auto fk = std::make_shared<pcp_direct::FrameKinematics>(urdf, joint_names, p.tip_link);
    RCLCPP_INFO(
      kLog, "direct planner: %s, range %.3f rad, extent %.3f rad, check every %.4f rad",
      settings.planner_config.c_str(), planner.effectiveRange(), planner.maximumExtent(),
      planner.longestValidSegment());

    // Optional probe (before the benchmark, results only in the .meta.json): MoveIt's planner
    // core with a counting validity checker vs the direct planner on the same queries.
    nlohmann::json probe = nlohmann::json::array();
    // Probe queries start at bench.first_query; scripts/check_probe.sh runs one per process,
    // because reusing MoveIt's cached planning context after swapping its checker crashed.
    for (int64_t i = std::max<int64_t>(0, p.first_query);
         i < std::max<int64_t>(0, p.first_query) + p.probe_queries &&
         i < static_cast<int64_t>(query_set.queries.size());
         ++i) {
      const auto & qq = query_set.queries[static_cast<std::size_t>(i)];
      moveit::core::RobotState ps(model);
      ps.setToDefaultValues();
      ps.setJointGroupPositions(jmg, qq.start);
      ps.update();
      moveit::core::RobotState pg(ps);
      pg.setJointGroupPositions(jmg, qq.goal);
      pg.update();
      auto m = probeMoveItChecks(
        moveit, scene, p.pipeline, p.group, settings.planner_config, p.planning_time, ps, pg);
      const auto d = planner.plan(qq.start, qq.goal);
      nlohmann::json entry = {
        {"query", qq.id},
        {"moveit", m},
        {"direct",
         {{"success", d.success},
          {"plan_time_s", d.plan_time_s},
          {"simplify_time_s", d.simplify_time_s},
          {"validity_checks", d.checks_total},
          {"validity_checks_plan", d.checks_plan},
          {"check_time_us_median", d.check_time_us_median},
          {"check_time_total_s", d.check_time_total_s}}}};
      RCLCPP_INFO(kLog, "probe %s", entry.dump().c_str());
      probe.push_back(entry);
    }

    std::unique_ptr<pcp_direct::TrajectoryExecutor> exec;
    if (p.execute) {
      exec = std::make_unique<pcp_direct::TrajectoryExecutor>(
        node, joint_names, p.action_name, p.controller_state_topic);
      if (!exec->waitForServer(30.0) || !exec->waitForJointStates(30.0)) {
        throw std::runtime_error("no trajectory action server or joint states");
      }
    }

    moveit_cpp::PlanningComponent planning(p.group, moveit);
    std::ofstream csv(p.output_csv);
    csv << kHeader;

    std::size_t skipped = 0;
    std::size_t returns_attempted = 0;
    std::size_t return_failures = 0;
    std::size_t rows = 0;
    const auto bench_start = std::chrono::steady_clock::now();
    const std::size_t first = static_cast<std::size_t>(std::max<int64_t>(0, p.first_query));
    std::size_t last = query_set.queries.size();
    if (p.max_queries >= 0) {
      last = std::min(last, first + static_cast<std::size_t>(p.max_queries));
    }

    // Recovery (harness, not part of either pipeline): an aborted execution can leave the arm
    // stopped against the shelf, in a configuration the planner rejects as a start. The first
    // full run cascaded from one such abort (NOTES.md). Escape: sample boxes of growing size
    // around the measured configuration, take the closest collision-free sample with >= 5 mm
    // clearance, move there in a straight joint-space line, then plan as usual.
    pcp_benchmark::DeterministicRng escape_rng(static_cast<std::uint64_t>(p.ompl_seed));
    std::size_t escapes = 0;
    std::size_t escape_failures = 0;
    auto escape = [&](const std::vector<double> & from) -> bool {
      ++escapes;
      for (const double radius : {0.02, 0.05, 0.1, 0.2, 0.4}) {
        std::vector<double> best;
        double best_d = std::numeric_limits<double>::infinity();
        for (int k = 0; k < 400; ++k) {
          std::vector<double> c(from);
          for (std::size_t j = 0; j < c.size(); ++j) {
            c[j] = std::clamp(
              from[j] + escape_rng.uniform(-radius, radius), checker.lowerLimits()[j],
              checker.upperLimits()[j]);
          }
          const double d = maxAbsDiff(c, from);
          if (
            d >= best_d || checker.isColliding(c) || checker.minimumDistance(c).distance < 0.005) {
            continue;
          }
          best = c;
          best_d = d;
        }
        if (best.empty()) {
          continue;
        }
        RCLCPP_WARN(kLog, "escape: moving %.3f rad to a collision-free configuration", best_d);
        const auto traj = pcp_direct::parameterise({from, best}, limits, p.sample_dt);
        exec->execute(traj.msg, traj.duration_s + p.exec_timeout_margin);
        exec->waitUntilAt(best, 0.02, 5.0);
        const auto now = exec->currentPositions();
        if (!now.empty() && !checker.isColliding(now)) {
          return true;
        }
      }
      ++escape_failures;
      return false;
    };

    // Moves the arm (direct planner + trapezoid + action) to `target`; true when there.
    auto reposition = [&](const std::vector<double> & target) -> bool {
      auto q = exec->currentPositions();
      if (q.empty()) {
        return false;
      }
      if (maxAbsDiff(q, target) <= p.goal_tolerance) {
        return true;
      }
      ++returns_attempted;
      bool ok = false;
      for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
        q = exec->currentPositions();
        if (checker.isColliding(q)) {
          RCLCPP_WARN(kLog, "reposition: the arm is in collision, escaping first");
          if (!escape(q)) {
            continue;
          }
          q = exec->currentPositions();
        }
        const auto r = planner.plan(q, target);
        if (!r.success) {
          RCLCPP_WARN(kLog, "reposition: plan failed (%s)", r.status.c_str());
          continue;
        }
        const auto traj = pcp_direct::parameterise(r.path, limits, p.sample_dt);
        const auto e = exec->execute(traj.msg, traj.duration_s + p.exec_timeout_margin);
        ok = exec->waitUntilAt(target, p.goal_tolerance, 5.0);
        if (!ok) {
          RCLCPP_WARN(
            kLog, "reposition: not at target (action %s, %s)", e.succeeded ? "ok" : "failed",
            e.error_string.c_str());
        }
      }
      return_failures += ok ? 0 : 1;
      return ok;
    };

    for (std::size_t qi = first; qi < last && rclcpp::ok(); ++qi) {
      const auto & q = query_set.queries[qi];
      moveit::core::RobotState start_state(model);
      start_state.setToDefaultValues();
      start_state.setJointGroupPositions(jmg, q.start);
      start_state.update();
      moveit::core::RobotState goal_state(start_state);
      goal_state.setJointGroupPositions(jmg, q.goal);
      goal_state.update();

      // A query rejected by EITHER checker at its start or goal is skipped and counted.
      std::string reject;
      if (pcp_direct::moveit_reference::isColliding(*scene, start_state, p.group)) {
        reject += "moveit_start;";
      }
      if (pcp_direct::moveit_reference::isColliding(*scene, goal_state, p.group)) {
        reject += "moveit_goal;";
      }
      if (checker.isColliding(q.start)) {
        reject += "direct_start;";
      }
      if (checker.isColliding(q.goal)) {
        reject += "direct_goal;";
      }

      // Keep-upright (SHOULD): v0.1's definition on both sides. Reference yaw halfway between
      // the start and goal yaw; per-axis rotation-vector tolerance 5 deg / sqrt(2), yaw free.
      spec.target_yaw = pcp_benchmark::circularMean(q.start_tcp[3], q.goal_tcp[3]);
      ompl::base::ConstraintPtr direct_constraint;
      if (constraint_type == pcp_benchmark::ConstraintType::kOrientation) {
        const double tol = pcp_benchmark::perAxisToleranceRad(spec.max_tilt_deg);
        const Eigen::Matrix3d target =
          (Eigen::AngleAxisd(spec.target_yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
        direct_constraint = std::make_shared<pcp_direct::KeepUprightConstraint>(
          fk, target, Eigen::Vector3d(tol, tol, M_PI));
      }

      const bool direct_first = q.id % 2 == 0;  // alternate the order to cancel order effects
      const std::vector<std::string> order = direct_first
                                               ? std::vector<std::string>{"direct", "moveit"}
                                               : std::vector<std::string>{"moveit", "direct"};
      for (std::size_t k = 0; k < order.size(); ++k) {
        const std::string & pipeline = order[k];
        Row row;
        row.query = q.id;
        row.pipeline = pipeline;
        row.order = std::to_string(k);
        row.start_region = q.start_region;
        row.goal_region = q.goal_region;
        if (!reject.empty()) {
          row.status = "skipped:" + reject;
          row.success = "0";
          if (k == 0) {
            ++skipped;
          }
        } else {
          bool at_start = true;
          if (exec) {
            at_start = reposition(q.start);
            row.at_start = at_start ? "1" : "0";
          }
          pcp_direct::JointPath path;
          bool plan_ok = false;
          trajectory_msgs::msg::JointTrajectory direct_traj;
          double direct_traj_duration = 0.0;
          robot_trajectory::RobotTrajectoryPtr moveit_traj;

          if (pipeline == "direct") {
            const auto t0 = std::chrono::steady_clock::now();
            const auto r = planner.plan(q.start, q.goal, direct_constraint);
            plan_ok = r.success;
            row.plan_time_s = fx(r.plan_time_s);
            row.checks_plan = std::to_string(r.checks_plan);
            row.validity_checks = std::to_string(r.checks_total);
            row.check_time_us_median = fx(r.check_time_us_median, 3);
            row.status = r.status;
            if (plan_ok) {
              row.simplify_time_s = fx(r.simplify_time_s);
              row.simplify_budget_s = fx(r.simplify_budget_s);
              row.raw_states = std::to_string(r.raw_states);
              const auto traj = pcp_direct::parameterise(r.path, limits, p.sample_dt);
              const auto t1 = std::chrono::steady_clock::now();
              row.param_time_s = fx(traj.param_time_s);
              row.time_to_trajectory_s = fx(std::chrono::duration<double>(t1 - t0).count());
              row.traj_duration_s = fx(traj.duration_s, 4);
              direct_traj = traj.msg;
              direct_traj_duration = traj.duration_s;
              path = r.path;
            }
          } else {
            planning.setStartState(start_state);
            planning.setGoal(goal_state);
            planning.setPathConstraints(pcp_benchmark::buildPathConstraints(constraint_type, spec));
            moveit_cpp::PlanningComponent::PlanRequestParameters params;
            params.planner_id = settings.planner_config;
            params.planning_pipeline = p.pipeline;
            params.planning_attempts = 1;  // the loaded default would be 5 parallel attempts
            params.planning_time = p.planning_time;
            params.max_velocity_scaling_factor = 1.0;
            params.max_acceleration_scaling_factor = 1.0;
            const auto t0 = std::chrono::steady_clock::now();
            const auto response = planning.plan(params, scene);
            const auto t1 = std::chrono::steady_clock::now();
            plan_ok = static_cast<bool>(response.error_code) && response.trajectory;
            row.moveit_error_code = std::to_string(response.error_code.val);
            row.status = plan_ok ? "Exact solution" : "error " + row.moveit_error_code;
            if (plan_ok) {
              // MoveIt's planning_time = OMPL solve + simplification (no adapters, no TOTG).
              row.plan_time_s = fx(response.planning_time);
              row.time_to_trajectory_s = fx(std::chrono::duration<double>(t1 - t0).count());
              moveit_traj = response.trajectory;
              row.traj_duration_s = fx(moveit_traj->getDuration(), 4);
              for (std::size_t w = 0; w < moveit_traj->getWayPointCount(); ++w) {
                std::vector<double> qw;
                moveit_traj->getWayPoint(w).copyJointGroupPositions(jmg, qw);
                path.push_back(std::move(qw));
              }
            }
          }
          row.plan_success = plan_ok ? "1" : "0";

          if (plan_ok) {
            const auto metrics =
              pcp_benchmark::computePathMetrics(model, p.group, p.tip_link, path, p.dense_step);
            row.path_len_joint_rad = fx(metrics.joint_path_rad);
            row.path_len_tcp_m = fx(metrics.tcp_path_m);
            row.max_tilt_deg = fx(metrics.max_tilt_deg, 3);
            row.n_waypoints = std::to_string(path.size());
            // Dense re-check at bench.dense_step by both checkers; crossval = the OTHER one.
            std::size_t direct_hits = 0;
            std::size_t moveit_hits = 0;
            moveit::core::RobotState dense(start_state);
            for (const auto & qd : pcp_benchmark::densify(path, p.dense_step)) {
              direct_hits += checker.isColliding(qd) ? 1 : 0;
              dense.setJointGroupPositions(jmg, qd);
              dense.update();
              moveit_hits +=
                pcp_direct::moveit_reference::isColliding(*scene, dense, p.group) ? 1 : 0;
            }
            const std::size_t other = pipeline == "direct" ? moveit_hits : direct_hits;
            const std::size_t own = pipeline == "direct" ? direct_hits : moveit_hits;
            row.crossval_ok = other == 0 ? "1" : "0";
            row.crossval_hits = std::to_string(other);
            row.own_dense_ok = own == 0 ? "1" : "0";
          }

          bool exec_ok = !p.execute;
          if (plan_ok && exec && at_start) {
            if (pipeline == "direct") {
              const auto e =
                exec->execute(direct_traj, direct_traj_duration + p.exec_timeout_margin);
              const bool there = exec->waitUntilAt(q.goal, p.goal_tolerance, 5.0);
              exec_ok = e.succeeded && there;
              row.exec_time_s = fx(e.exec_time_s, 4);
              row.max_track_err_rad = fx(e.tracking.max_error_rad, 5);
              row.exec_error = csvSafe(e.error_string);
            } else {
              exec->startMonitoring();
              const double t0 = exec->simNow();
              const auto status = moveit->execute(moveit_traj);
              const double t1 = exec->simNow();
              const auto tracking = exec->stopMonitoring();
              const bool there = exec->waitUntilAt(q.goal, p.goal_tolerance, 5.0);
              exec_ok = status == moveit_controller_manager::ExecutionStatus::SUCCEEDED && there;
              row.exec_time_s = fx(t1 - t0, 4);
              row.max_track_err_rad = fx(tracking.max_error_rad, 5);
              row.exec_error = csvSafe(status.asString());
            }
            const auto qf = exec->currentPositions();
            row.final_err_rad = fx(maxAbsDiff(qf, q.goal), 5);
            row.exec_success = exec_ok ? "1" : "0";
          } else if (plan_ok && exec) {
            row.exec_success = "0";
            row.exec_error = "arm not at the query start";
          }
          row.success = plan_ok && exec_ok ? "1" : "0";
        }

        csv << row.query << ',' << row.pipeline << ',' << row.success << ',' << row.plan_time_s
            << ',' << row.simplify_time_s << ',' << row.param_time_s << ','
            << row.time_to_trajectory_s << ',' << row.exec_time_s << ',' << row.path_len_joint_rad
            << ',' << row.path_len_tcp_m << ',' << row.validity_checks << ','
            << row.check_time_us_median << ',' << row.max_track_err_rad << ',' << row.crossval_ok
            << ',' << p.commit << ',' << query_set.config.seed << ',' << csvSafe(row.status) << ','
            << row.plan_success << ',' << row.exec_success << ',' << row.traj_duration_s << ','
            << row.n_waypoints << ',' << row.own_dense_ok << ',' << row.crossval_hits << ','
            << row.moveit_error_code << ',' << row.final_err_rad << ',' << row.at_start << ','
            << row.order << ',' << row.start_region << ',' << row.goal_region << ','
            << row.checks_plan << ',' << row.simplify_budget_s << ',' << row.raw_states << ','
            << row.exec_error << ',' << p.ompl_seed << ',' << (p.execute ? 1 : 0) << ','
            << p.constraint << ',' << row.max_tilt_deg << '\n';
        csv.flush();
        ++rows;
        RCLCPP_INFO(
          kLog, "query %zu %-6s plan %s %s exec %s (%s s plan, %s s to trajectory)", q.id,
          pipeline.c_str(), row.plan_success.c_str(), row.status.c_str(),
          row.exec_success.empty() ? "-" : row.exec_success.c_str(), row.plan_time_s.c_str(),
          row.time_to_trajectory_s.c_str());
      }
    }
    const double total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - bench_start).count();
    RCLCPP_INFO(
      kLog, "done: %zu rows, %zu skipped queries, %zu/%zu repositioning failures, %.1f s", rows,
      skipped, return_failures, returns_attempted, total);

    const nlohmann::json meta = {
      {"commit", p.commit},
      {"finished_utc", utcNow()},
      {"queries_file", p.queries_file},
      {"query_seed", query_set.config.seed},
      {"ompl_seed", p.ompl_seed},
      {"first_query", first},
      {"queries_run", last - first},
      {"rows", rows},
      {"skipped_queries", skipped},
      {"execute", p.execute},
      {"repositions_attempted", returns_attempted},
      {"reposition_failures", return_failures},
      {"escapes", escapes},
      {"escape_failures", escape_failures},
      {"total_wall_time_s", total},
      {"planner_config", settings.planner_config},
      {"range_rad", planner.effectiveRange()},
      {"range_from_yaml", settings.range_set_in_yaml},
      {"longest_valid_segment_fraction", settings.longest_valid_segment_fraction},
      {"max_extent_rad", planner.maximumExtent()},
      {"longest_valid_segment_rad", planner.longestValidSegment()},
      {"planning_time_s", p.planning_time},
      {"constraint", p.constraint},
      {"max_tilt_deg", p.max_tilt_deg},
      {"sample_dt_s", p.sample_dt},
      {"goal_tolerance_rad", p.goal_tolerance},
      {"dense_step_rad", p.dense_step},
      {"joint_limits",
       {{"max_velocity", limits.max_velocity}, {"max_acceleration", limits.max_acceleration}}},
      {"moveit_ompl_context", moveit_ompl},
      {"moveit_check_probe", probe},
      {"moveit_collision_detector", scene->getCollisionDetectorName()},
      {"world_objects", scene->getWorld()->getObjectIds()},
      {"direct_active_pairs", checker.activePairCount()},
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
  // MoveIt 2.15.2's ~MoveItCpp segfaults on shutdown (see pcp_benchmark); outputs are closed.
  std::fflush(nullptr);
  std::_Exit(exit_code);
}

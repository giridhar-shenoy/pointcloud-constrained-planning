// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// collision_agreement: do two independent collision checkers agree on the same scene?
//
// MoveIt side: MoveItCpp's planning-scene monitor, no 3-D sensors configured (so no octomap
// updater), receiving the shelf boxes from shelf_boxes_loader. Direct side: pcp_direct's
// Pinocchio/coal checker built from the same robot_description, robot_description_semantic and
// shelf YAML. Both are asked about the same seeded, uniformly random joint configurations with
// the request MoveIt's OMPL validity checker uses (arm group, no padding). Nothing is tuned.
//
// Parameters (all under `agree.`): shelf_yaml, output_csv, count (500), seed (0),
// group (ur_manipulator), scene_timeout_s (60).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pcp_benchmark/query_generator.hpp"
#include "pcp_direct/collision.hpp"
#include "pcp_direct/moveit_reference.hpp"

namespace
{
const rclcpp::Logger kLog = rclcpp::get_logger("collision_agreement");

std::string joinPairs(const std::vector<pcp_direct::NamedPair> & pairs)
{
  std::string out;
  for (const auto & p : pairs) {
    if (!out.empty()) {
      out += ';';
    }
    out += p.first + '|' + p.second;
  }
  return out;
}

std::string fixed(double v, int precision)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << v;
  return ss.str();
}

double median(std::vector<double> v)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("collision_agreement", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit;
  int exit_code = 0;

  try {
    std::string shelf_yaml;
    std::string output_csv;
    int64_t count = 500;
    int64_t seed = 0;
    std::string group = "ur_manipulator";
    double scene_timeout = 60.0;
    std::string commit = "unknown";
    node->get_parameter("agree.shelf_yaml", shelf_yaml);
    node->get_parameter("agree.output_csv", output_csv);
    node->get_parameter("agree.count", count);
    node->get_parameter("agree.seed", seed);
    node->get_parameter("agree.group", group);
    node->get_parameter("agree.scene_timeout_s", scene_timeout);
    node->get_parameter("agree.commit", commit);
    if (shelf_yaml.empty() || output_csv.empty()) {
      throw std::runtime_error("agree.shelf_yaml and agree.output_csv are required");
    }
    const auto boxes = pcp_direct::loadBoxes(shelf_yaml);

    moveit = std::make_shared<moveit_cpp::MoveItCpp>(node);
    auto psm = moveit->getPlanningSceneMonitorNonConst();
    const auto model = moveit->getRobotModel();
    const auto * jmg = model->getJointModelGroup(group);
    if (jmg == nullptr) {
      throw std::runtime_error("unknown group " + group);
    }

    // Wait until every box from the YAML is in MoveIt's scene, then freeze one copy.
    RCLCPP_INFO(kLog, "waiting for %zu shelf boxes in the planning scene ...", boxes.size());
    const auto wait_start = std::chrono::steady_clock::now();
    planning_scene::PlanningScenePtr scene;
    while (rclcpp::ok()) {
      std::size_t present = 0;
      {
        planning_scene_monitor::LockedPlanningSceneRO locked(psm);
        const planning_scene::PlanningSceneConstPtr & live = locked;
        for (const auto & b : boxes) {
          present += live->getWorld()->hasObject(b.name) ? 1 : 0;
        }
        if (present == boxes.size()) {
          scene = planning_scene::PlanningScene::clone(live);
        }
      }
      if (scene) {
        break;
      }
      if (
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() >
        scene_timeout) {
        throw std::runtime_error("shelf boxes did not arrive within agree.scene_timeout_s");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (scene->getWorld()->hasObject(planning_scene::PlanningScene::OCTOMAP_NS)) {
      throw std::runtime_error("the planning scene has an octomap; this test must run without one");
    }
    const auto world_ids = scene->getWorld()->getObjectIds();

    std::string urdf;
    std::string srdf;
    node->get_parameter("robot_description", urdf);
    node->get_parameter("robot_description_semantic", srdf);
    const auto joint_names = jmg->getVariableNames();
    pcp_direct::CollisionChecker direct(urdf, srdf, boxes, joint_names);

    // Same bounds on both sides (MoveIt's come from the URDF + joint_limits.yaml).
    std::vector<double> lo;
    std::vector<double> hi;
    bool bounds_equal = true;
    for (std::size_t j = 0; j < joint_names.size(); ++j) {
      const auto & b = model->getVariableBounds(joint_names[j]);
      lo.push_back(b.min_position_);
      hi.push_back(b.max_position_);
      bounds_equal = bounds_equal && std::abs(b.min_position_ - direct.lowerLimits()[j]) < 1e-12 &&
                     std::abs(b.max_position_ - direct.upperLimits()[j]) < 1e-12;
    }
    const auto moveit_pairs = pcp_direct::moveit_reference::checkedPairs(*scene, group);
    const auto direct_pairs = direct.activePairs();

    std::ofstream csv(output_csv);
    csv << "index,seed";
    for (const auto & j : joint_names) {
      csv << ',' << j;
    }
    csv << ",moveit_collision,direct_collision,agree,moveit_pairs,direct_pairs,"
           "moveit_min_distance_m,moveit_min_pair,direct_min_distance_m,direct_min_pair,"
           "direct_distance_of_moveit_pairs_m,moveit_check_us,direct_check_us\n";

    pcp_benchmark::DeterministicRng rng(static_cast<std::uint64_t>(seed));
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::size_t agree = 0;
    std::size_t moveit_hits = 0;
    std::size_t direct_hits = 0;
    std::vector<double> moveit_us;
    std::vector<double> direct_us;
    nlohmann::json disagreements = nlohmann::json::array();
    std::vector<std::vector<double>> samples;
    std::vector<bool> moveit_verdicts;
    for (int64_t i = 0; i < count && rclcpp::ok(); ++i) {
      std::vector<double> q(joint_names.size());
      for (std::size_t j = 0; j < q.size(); ++j) {
        q[j] = pcp_benchmark::round6(rng.uniform(lo[j], hi[j]));
      }
      // Both timings include forward kinematics: MoveIt's OMPL checker also sets the state and
      // updates its transforms inside every check, and the direct checker runs Pinocchio's FK
      // + geometry placement inside isValid().
      const auto t0 = std::chrono::steady_clock::now();
      state.setJointGroupPositions(jmg, q);
      state.update();
      const bool m_col = pcp_direct::moveit_reference::isColliding(*scene, state, group);
      const auto t1 = std::chrono::steady_clock::now();
      const bool d_col = !direct.isValid(q);
      const auto t2 = std::chrono::steady_clock::now();
      moveit_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
      direct_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());

      const auto m_pairs = pcp_direct::moveit_reference::collidingPairs(*scene, state, group);
      const auto d_pairs = direct.collidingPairs(q);
      const auto m_min = pcp_direct::moveit_reference::minimumDistance(*scene, state, group);
      const auto d_min = direct.minimumDistance(q);
      std::string d_of_m;
      if (m_col != d_col) {
        for (const auto & p : m_pairs) {
          if (!d_of_m.empty()) {
            d_of_m += ';';
          }
          d_of_m += fixed(direct.pairDistance(q, p), 6);
        }
      }
      samples.push_back(q);
      moveit_verdicts.push_back(m_col);
      moveit_hits += m_col ? 1 : 0;
      direct_hits += d_col ? 1 : 0;
      agree += m_col == d_col ? 1 : 0;
      if (m_col != d_col) {
        disagreements.push_back(
          {{"index", i},
           {"q", q},
           {"moveit_pairs", joinPairs(m_pairs)},
           {"direct_pairs", joinPairs(d_pairs)},
           {"moveit_min_distance_m", m_min.distance},
           {"direct_min_distance_m", d_min.distance}});
        RCLCPP_WARN(
          kLog,
          "disagreement at %ld: moveit=%d [%s] direct=%d [%s] min dist moveit %.6f direct %.6f",
          static_cast<long>(i), m_col, joinPairs(m_pairs).c_str(), d_col,
          joinPairs(d_pairs).c_str(), m_min.distance, d_min.distance);
      }
      csv << i << ',' << seed;
      for (double v : q) {
        csv << ',' << fixed(v, 6);
      }
      csv << ',' << m_col << ',' << d_col << ',' << (m_col == d_col) << ',' << joinPairs(m_pairs)
          << ',' << joinPairs(d_pairs) << ',' << fixed(m_min.distance, 6) << ',' << m_min.pair.first
          << '|' << m_min.pair.second << ',' << fixed(d_min.distance, 6) << ',' << d_min.pair.first
          << '|' << d_min.pair.second << ',' << d_of_m << ',' << fixed(moveit_us.back(), 2) << ','
          << fixed(direct_us.back(), 2) << '\n';
    }
    csv.close();

    // Boundary probe (supplement; the 500 uniform samples above are the headline). Uniform
    // samples almost never land within millimetres of contact, which is where two libraries could
    // differ. For every pair of consecutive samples on which MoveIt's verdict changes, bisect the
    // joint-space segment with MoveIt's checker until the bracket is shorter than 1e-7 rad in
    // every joint, then ask the direct checker about both ends of the bracket.
    const std::string boundary_csv_path =
      output_csv.substr(0, output_csv.rfind('.')) + "_boundary.csv";
    std::ofstream bcsv(boundary_csv_path);
    bcsv << "segment_start_index,free_side_direct_collision,colliding_side_direct_collision,agree,"
            "free_side_direct_min_distance_m,free_side_moveit_min_distance_m,"
            "colliding_side_moveit_pairs,colliding_side_direct_pairs,bracket_rad";
    for (const auto & j : joint_names) {
      bcsv << ",free_" << j;
    }
    bcsv << '\n';
    std::size_t boundary_total = 0;
    std::size_t boundary_agree = 0;
    auto moveit_at = [&](const std::vector<double> & q) {
      state.setJointGroupPositions(jmg, q);
      state.update();
      return pcp_direct::moveit_reference::isColliding(*scene, state, group);
    };
    for (std::size_t i = 0; i + 1 < samples.size() && rclcpp::ok(); ++i) {
      if (moveit_verdicts[i] == moveit_verdicts[i + 1]) {
        continue;
      }
      std::vector<double> free_q = moveit_verdicts[i] ? samples[i + 1] : samples[i];
      std::vector<double> hit_q = moveit_verdicts[i] ? samples[i] : samples[i + 1];
      double bracket = 0.0;
      for (int it = 0; it < 200; ++it) {
        bracket = 0.0;
        std::vector<double> mid(free_q.size());
        for (std::size_t j = 0; j < mid.size(); ++j) {
          mid[j] = 0.5 * (free_q[j] + hit_q[j]);
          bracket = std::max(bracket, std::abs(hit_q[j] - free_q[j]));
        }
        if (bracket < 1e-7) {
          break;
        }
        (moveit_at(mid) ? hit_q : free_q) = mid;
      }
      const bool d_free_side = direct.isColliding(free_q);
      const bool d_hit_side = direct.isColliding(hit_q);
      const bool ok = !d_free_side && d_hit_side;
      ++boundary_total;
      boundary_agree += ok ? 1 : 0;
      state.setJointGroupPositions(jmg, free_q);
      state.update();
      const auto m_free = pcp_direct::moveit_reference::minimumDistance(*scene, state, group);
      state.setJointGroupPositions(jmg, hit_q);
      state.update();
      const auto m_pairs_hit = pcp_direct::moveit_reference::collidingPairs(*scene, state, group);
      bcsv << i << ',' << d_free_side << ',' << d_hit_side << ',' << ok << ','
           << fixed(direct.minimumDistance(free_q).distance, 9) << ',' << fixed(m_free.distance, 9)
           << ',' << joinPairs(m_pairs_hit) << ',' << joinPairs(direct.collidingPairs(hit_q)) << ','
           << fixed(bracket, 9);
      for (double v : free_q) {
        bcsv << ',' << fixed(v, 9);
      }
      bcsv << '\n';
    }
    bcsv.close();
    const double boundary_pct = boundary_total > 0 ? 100.0 * static_cast<double>(boundary_agree) /
                                                       static_cast<double>(boundary_total)
                                                   : 0.0;
    RCLCPP_INFO(
      kLog, "boundary probe: %zu/%zu brackets agree (%.2f %%) -> %s", boundary_agree,
      boundary_total, boundary_pct, boundary_csv_path.c_str());

    const double pct =
      count > 0 ? 100.0 * static_cast<double>(agree) / static_cast<double>(count) : 0.0;
    RCLCPP_INFO(
      kLog,
      "agreement %zu/%ld = %.2f %% (MoveIt colliding %zu, direct colliding %zu); median check "
      "MoveIt %.1f us, direct %.1f us",
      agree, static_cast<long>(count), pct, moveit_hits, direct_hits, median(moveit_us),
      median(direct_us));

    const nlohmann::json meta = {
      {"commit", commit},
      {"count", count},
      {"seed", seed},
      {"group", group},
      {"sampling", "uniform within the URDF joint bounds, rounded to 1e-6 rad"},
      {"agree", agree},
      {"agreement_pct", pct},
      {"moveit_colliding", moveit_hits},
      {"direct_colliding", direct_hits},
      {"disagreements", disagreements},
      {"moveit_collision_detector", scene->getCollisionDetectorName()},
      {"moveit_world_objects", world_ids},
      {"moveit_checked_pairs", moveit_pairs.size()},
      {"direct_active_pairs", direct_pairs.size()},
      {"pair_sets_equal", moveit_pairs == direct_pairs},
      {"direct_srdf_removed_pairs", direct.srdfRemovedPairCount()},
      {"bounds_equal", bounds_equal},
      {"moveit_check_us_median", median(moveit_us)},
      {"direct_check_us_median", median(direct_us)},
      {"timing_note",
       "one call per configuration, wall clock, same process, after the scene "
       "was frozen, both including forward kinematics; MoveIt: setJointGroupPositions "
       "+ update + PlanningScene::checkCollision with the OMPL checker's request; "
       "direct: CollisionChecker::isValid"},
      {"boundary_brackets", boundary_total},
      {"boundary_agree", boundary_agree},
      {"boundary_agreement_pct", boundary_pct},
      {"boundary_method",
       "bisection on MoveIt's verdict between consecutive samples with "
       "different verdicts, bracket < 1e-7 rad per joint; direct checker "
       "asked at both bracket ends"},
      {"hardware_threads", std::thread::hardware_concurrency()},
    };
    std::ofstream(output_csv + ".meta.json") << meta.dump(2) << "\n";
    if (moveit_pairs != direct_pairs) {
      RCLCPP_ERROR(
        kLog, "pair sets differ: MoveIt %zu, direct %zu", moveit_pairs.size(), direct_pairs.size());
    }
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

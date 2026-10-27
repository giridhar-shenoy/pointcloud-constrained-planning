// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_benchmark/query_generator.hpp"

#include <cmath>
#include <stdexcept>

#include <moveit/planning_scene/planning_scene.hpp>

namespace pcp_benchmark
{
namespace
{
double wrapToPi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

Eigen::Vector4d tcpVector(const Eigen::Vector3d & p, double yaw)
{
  return {p.x(), p.y(), p.z(), yaw};
}

std::vector<double> rounded(const std::vector<double> & v)
{
  std::vector<double> out(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) {
    out[i] = round6(v[i]);
  }
  return out;
}
}  // namespace

double DeterministicRng::uniform() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }

std::size_t DeterministicRng::index(std::size_t n)
{
  return std::min(n - 1, static_cast<std::size_t>(uniform() * static_cast<double>(n)));
}

double round6(double value)
{
  const double r = std::round(value * 1e6) / 1e6;
  return r == 0.0 ? 0.0 : r;  // no "-0"
}

Eigen::Isometry3d toolDownPose(const Eigen::Vector3d & position, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  pose.translation() = position;
  return pose;
}

bool solveIkDls(
  moveit::core::RobotState & state, const moveit::core::JointModelGroup * group,
  const moveit::core::LinkModel * tip, const Eigen::Isometry3d & target,
  const GeneratorConfig & cfg)
{
  const Eigen::Matrix<double, 6, 6> damping =
    cfg.ik_damping * cfg.ik_damping * Eigen::Matrix<double, 6, 6>::Identity();
  std::vector<double> q;
  Eigen::MatrixXd jacobian;
  for (int it = 0; it < cfg.ik_max_iterations; ++it) {
    state.updateLinkTransforms();
    const Eigen::Isometry3d & current = state.getGlobalLinkTransform(tip);
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target.translation() - current.translation();
    const Eigen::AngleAxisd rot_err(target.linear() * current.linear().transpose());
    error.tail<3>() = rot_err.angle() * rot_err.axis();
    if (error.head<3>().norm() < cfg.ik_tolerance && error.tail<3>().norm() < cfg.ik_tolerance) {
      return true;
    }
    // Rows 0-2 linear, 3-5 angular velocity, both in the model (world) frame.
    state.getJacobian(group, tip, Eigen::Vector3d::Zero(), jacobian);
    const Eigen::VectorXd dq =
      jacobian.transpose() * (jacobian * jacobian.transpose() + damping).ldlt().solve(error);
    const double max_abs = dq.cwiseAbs().maxCoeff();
    const double scale = max_abs > 0.2 ? 0.2 / max_abs : 1.0;  // trust region of 0.2 rad
    state.copyJointGroupPositions(group, q);
    for (std::size_t j = 0; j < q.size(); ++j) {
      q[j] += scale * dq[static_cast<Eigen::Index>(j)];
    }
    state.setJointGroupPositions(group, q);
    state.enforceBounds(group);
  }
  return false;
}

QuerySet generateQueries(
  const moveit::core::RobotModelConstPtr & model, const ShelfScene & shelf,
  const GeneratorConfig & cfg)
{
  const auto * group = model->getJointModelGroup(cfg.group);
  const auto * tip = model->getLinkModel(cfg.tip_link);
  if (group == nullptr || tip == nullptr) {
    throw std::invalid_argument("unknown group '" + cfg.group + "' or tip '" + cfg.tip_link + "'");
  }
  planning_scene::PlanningScene scene(model);
  addShelfToPlanningScene(shelf, scene, cfg.collision_padding);

  QuerySet set;
  set.config = cfg;
  set.joint_names = group->getVariableNames();
  DeterministicRng rng(cfg.seed);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  // Draw tool-down poses in `region` until one has a valid IK solution; return it (rounded to
  // 1e-6 rad, re-checked after rounding) and the pose that produced it.
  auto sample_state = [&](const Region & region, Eigen::Vector4d & tcp) -> std::vector<double> {
    for (int attempt = 0; attempt < cfg.max_pose_samples; ++attempt) {
      ++set.pose_samples;
      Eigen::Vector3d p;
      for (int k = 0; k < 3; ++k) {
        p[k] = rng.uniform(region.min[k], region.max[k]);
      }
      const double yaw = rng.uniform(-M_PI, M_PI);
      const Eigen::Isometry3d target = toolDownPose(p, yaw);
      bool converged_any = false;
      for (int s = 0; s < cfg.ik_seeds; ++s) {
        // Elbow-up, tool-down seeds: q4 = -pi/2 - q2 - q3 and q5 = -pi/2 point the tool down.
        const double q1 = std::atan2(p.y(), p.x()) + rng.uniform(-0.3, 0.3);
        const double q2 = rng.uniform(-2.2, -0.4);
        const double q3 = rng.uniform(0.4, 2.6);
        const double q6 = rng.uniform(-M_PI, M_PI);
        std::vector<double> seed = {q1, q2, q3, -M_PI / 2.0 - q2 - q3, -M_PI / 2.0, q6};
        state.setJointGroupPositions(group, seed);
        if (!solveIkDls(state, group, tip, target, cfg)) {
          continue;
        }
        converged_any = true;
        std::vector<double> q;
        state.copyJointGroupPositions(group, q);
        for (double & v : q) {
          v = wrapToPi(v);
        }
        q = rounded(q);
        state.setJointGroupPositions(group, q);
        state.updateCollisionBodyTransforms();
        if (!state.satisfiesBounds(group) || scene.isStateColliding(state, cfg.group)) {
          ++set.collision_rejects;
          continue;
        }
        tcp = tcpVector(p, yaw);
        return q;
      }
      if (!converged_any) {
        ++set.ik_failures;
      }
    }
    throw std::runtime_error("no valid IK solution in region '" + region.name + "'");
  };

  const auto & regions = shelf.target_regions;
  for (std::size_t id = 0; id < cfg.count; ++id) {
    Query query;
    query.id = id;
    const Region & start_region = regions[rng.index(regions.size())];
    const Region & goal_region = regions[rng.index(regions.size())];
    query.start_region = start_region.name;
    query.goal_region = goal_region.name;
    query.start = sample_state(start_region, query.start_tcp);
    do {
      query.goal = sample_state(goal_region, query.goal_tcp);
    } while ((query.goal_tcp.head<3>() - query.start_tcp.head<3>()).norm() < cfg.min_tcp_distance);
    set.queries.push_back(std::move(query));
  }
  return set;
}

nlohmann::json toJson(const QuerySet & set)
{
  nlohmann::json j;
  j["scene"] = set.scene_name;
  j["seed"] = set.config.seed;
  j["count"] = set.queries.size();
  j["group"] = set.config.group;
  j["tip_link"] = set.config.tip_link;
  j["joint_names"] = set.joint_names;
  j["generator"] = {
    {"collision_padding_m", set.config.collision_padding},
    {"min_tcp_distance_m", set.config.min_tcp_distance},
    {"ik", "damped least squares, fixed iteration budget"},
    {"ik_seeds", set.config.ik_seeds},
    {"ik_max_iterations", set.config.ik_max_iterations},
    {"ik_damping", set.config.ik_damping},
    {"ik_tolerance", set.config.ik_tolerance},
    {"tool_orientation", "z axis along world -z, yaw uniform in [-pi, pi)"},
    {"pose_samples", set.pose_samples},
    {"ik_failures", set.ik_failures},
    {"collision_rejects", set.collision_rejects},
  };
  nlohmann::json queries = nlohmann::json::array();
  for (const auto & q : set.queries) {
    auto tcp = [](const Eigen::Vector4d & v) {
      return std::vector<double>{round6(v[0]), round6(v[1]), round6(v[2]), round6(v[3])};
    };
    queries.push_back({
      {"id", q.id},
      {"start_region", q.start_region},
      {"goal_region", q.goal_region},
      {"start", q.start},
      {"goal", q.goal},
      {"start_tcp_xyz_yaw", tcp(q.start_tcp)},
      {"goal_tcp_xyz_yaw", tcp(q.goal_tcp)},
    });
  }
  j["queries"] = queries;
  return j;
}

QuerySet queriesFromJson(const nlohmann::json & json)
{
  QuerySet set;
  set.scene_name = json.at("scene").get<std::string>();
  set.config.seed = json.at("seed").get<std::uint64_t>();
  set.config.group = json.at("group").get<std::string>();
  set.config.tip_link = json.at("tip_link").get<std::string>();
  set.joint_names = json.at("joint_names").get<std::vector<std::string>>();
  for (const auto & jq : json.at("queries")) {
    Query q;
    q.id = jq.at("id").get<std::size_t>();
    q.start_region = jq.at("start_region").get<std::string>();
    q.goal_region = jq.at("goal_region").get<std::string>();
    q.start = jq.at("start").get<std::vector<double>>();
    q.goal = jq.at("goal").get<std::vector<double>>();
    const auto s = jq.at("start_tcp_xyz_yaw").get<std::vector<double>>();
    const auto g = jq.at("goal_tcp_xyz_yaw").get<std::vector<double>>();
    q.start_tcp = Eigen::Vector4d(s[0], s[1], s[2], s[3]);
    q.goal_tcp = Eigen::Vector4d(g[0], g[1], g[2], g[3]);
    set.queries.push_back(std::move(q));
  }
  set.config.count = set.queries.size();
  return set;
}

}  // namespace pcp_benchmark

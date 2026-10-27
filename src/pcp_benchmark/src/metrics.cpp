// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_benchmark/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <moveit/robot_state/robot_state.hpp>

namespace pcp_benchmark
{

JointPath densify(const JointPath & path, double max_step_rad)
{
  if (max_step_rad <= 0.0) {
    throw std::invalid_argument("max_step_rad must be positive");
  }
  if (path.size() < 2) {
    return path;
  }
  JointPath dense;
  dense.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    double max_delta = 0.0;
    for (std::size_t j = 0; j < a.size(); ++j) {
      max_delta = std::max(max_delta, std::abs(b[j] - a[j]));
    }
    const auto steps =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(max_delta / max_step_rad)));
    for (std::size_t s = 1; s <= steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      std::vector<double> q(a.size());
      for (std::size_t j = 0; j < a.size(); ++j) {
        q[j] = a[j] + t * (b[j] - a[j]);
      }
      dense.push_back(std::move(q));
    }
  }
  return dense;
}

double jointPathLength(const JointPath & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    double sq = 0.0;
    for (std::size_t j = 0; j < path[i].size(); ++j) {
      const double d = path[i][j] - path[i - 1][j];
      sq += d * d;
    }
    length += std::sqrt(sq);
  }
  return length;
}

double tiltFromDownDeg(const Eigen::Matrix3d & rotation)
{
  const double c = std::clamp(-rotation.col(2).z(), -1.0, 1.0);  // z_tool . (-z_world)
  return std::acos(c) * 180.0 / M_PI;
}

PathMetrics computePathMetrics(
  const moveit::core::RobotModelConstPtr & model, const std::string & group,
  const std::string & tip_link, const JointPath & path, double max_step_rad)
{
  PathMetrics m;
  m.n_waypoints = path.size();
  m.joint_path_rad = jointPathLength(path);
  if (path.empty()) {
    return m;
  }
  const auto * jmg = model->getJointModelGroup(group);
  const auto * tip = model->getLinkModel(tip_link);
  if (jmg == nullptr || tip == nullptr) {
    throw std::invalid_argument("unknown group or tip link");
  }
  const JointPath dense = densify(path, max_step_rad);
  m.n_dense_states = dense.size();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  Eigen::Vector3d prev = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < dense.size(); ++i) {
    state.setJointGroupPositions(jmg, dense[i]);
    state.updateLinkTransforms();
    const Eigen::Isometry3d & pose = state.getGlobalLinkTransform(tip);
    if (i > 0) {
      m.tcp_path_m += (pose.translation() - prev).norm();
    }
    prev = pose.translation();
    m.max_tilt_deg = std::max(m.max_tilt_deg, tiltFromDownDeg(pose.linear()));
  }
  return m;
}

}  // namespace pcp_benchmark

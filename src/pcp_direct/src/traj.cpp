// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_direct/traj.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "pcp_direct/traj/trapezoidal.hpp"

namespace pcp_direct
{

JointLimits loadJointLimits(
  const std::string & yaml_path, const std::vector<std::string> & joint_names)
{
  const YAML::Node root = YAML::LoadFile(yaml_path)["joint_limits"];
  JointLimits limits;
  limits.joint_names = joint_names;
  for (const auto & name : joint_names) {
    const YAML::Node j = root[name];
    if (!j || !j["max_velocity"] || !j["max_acceleration"]) {
      throw std::runtime_error(yaml_path + ": no velocity/acceleration limit for " + name);
    }
    limits.max_velocity.push_back(j["max_velocity"].as<double>());
    limits.max_acceleration.push_back(j["max_acceleration"].as<double>());
  }
  return limits;
}

namespace
{
builtin_interfaces::msg::Duration toDuration(double t)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(t));
  d.nanosec = static_cast<uint32_t>(std::llround((t - std::floor(t)) * 1e9));
  if (d.nanosec >= 1000000000u) {
    d.sec += 1;
    d.nanosec -= 1000000000u;
  }
  return d;
}

trajectory_msgs::msg::JointTrajectoryPoint toPoint(
  const std::vector<profiles::JointSample> & s, double t)
{
  trajectory_msgs::msg::JointTrajectoryPoint p;
  for (const auto & j : s) {
    p.positions.push_back(j.position);
    p.velocities.push_back(j.velocity);
    p.accelerations.push_back(j.acceleration);
  }
  p.time_from_start = toDuration(t);
  return p;
}
}  // namespace

ParameterisedTrajectory parameterise(
  const JointPath & path, const JointLimits & limits, double sample_dt)
{
  if (path.size() < 2) {
    throw std::invalid_argument("parameterise: a path needs at least two waypoints");
  }
  if (sample_dt <= 0.0) {
    throw std::invalid_argument("parameterise: sample_dt must be positive");
  }
  const auto t0 = std::chrono::steady_clock::now();
  const JointPath waypoints(path.begin() + 1, path.end());
  // Requested times of 0: every segment takes the shortest duration its limits allow.
  const std::vector<double> times(waypoints.size(), 0.0);
  const profiles::TrapezoidalTrajectory profile(
    path.front(), waypoints, times, limits.max_velocity, limits.max_acceleration);

  ParameterisedTrajectory out;
  out.msg.joint_names = limits.joint_names;
  std::vector<profiles::JointSample> s;
  profile.sample(0.0, s);
  out.msg.points.push_back(toPoint(s, 0.0));
  double seg_start = 0.0;
  for (std::size_t k = 0; k < profile.segment_count(); ++k) {
    const double T = profile.segment_duration(k);
    const auto m = static_cast<std::size_t>(std::max(1.0, std::ceil(T / (3.0 * sample_dt))));
    const std::size_t steps = 3 * m;
    for (std::size_t i = 1; i <= steps; ++i) {
      // The last sample of a segment is its end at rest (sample() at the boundary would return
      // the next segment's start, which is the same position at rest as well).
      const double t = seg_start + T * static_cast<double>(i) / static_cast<double>(steps);
      profile.sample(i == steps ? seg_start + T - 1e-12 : t, s);
      if (i == steps) {
        for (std::size_t j = 0; j < s.size(); ++j) {
          s[j] = profiles::JointSample{waypoints[k][j], 0.0, 0.0};
        }
      }
      out.msg.points.push_back(toPoint(s, t));
    }
    seg_start += T;
  }
  out.duration_s = profile.duration();
  out.param_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

}  // namespace pcp_direct

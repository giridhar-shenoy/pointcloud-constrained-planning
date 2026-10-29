// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_direct/exec.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

namespace pcp_direct
{

TrajectoryExecutor::TrajectoryExecutor(
  const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & joint_names,
  const std::string & action_name, const std::string & state_topic,
  const std::string & joint_states_topic)
: node_(node), joint_names_(joint_names)
{
  client_ = rclcpp_action::create_client<FJT>(node_, action_name);
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS().keep_last(100),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      auto q = reorder(msg->name, msg->position);
      auto v = reorder(msg->name, msg->velocity);
      if (q.empty()) {
        return;
      }
      const double t = rclcpp::Time(msg->header.stamp).seconds();
      std::lock_guard<std::mutex> lock(mutex_);
      latest_q_ = q;
      latest_v_ = v;
      if (monitoring_) {
        js_samples_.push_back({t, std::move(q)});
      }
    });
  cs_sub_ = node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    state_topic, rclcpp::QoS(100),
    [this](const control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr msg) {
      auto ref = reorder(msg->joint_names, msg->reference.positions);
      if (ref.empty()) {
        return;
      }
      const double t = rclcpp::Time(msg->header.stamp).seconds();
      std::lock_guard<std::mutex> lock(mutex_);
      if (monitoring_) {
        ref_samples_.push_back({t, std::move(ref)});
      }
    });
}

std::vector<double> TrajectoryExecutor::reorder(
  const std::vector<std::string> & names, const std::vector<double> & values) const
{
  std::vector<double> out(joint_names_.size());
  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    const auto it = std::find(names.begin(), names.end(), joint_names_[j]);
    if (it == names.end()) {
      return {};
    }
    const auto k = static_cast<std::size_t>(it - names.begin());
    if (k >= values.size()) {
      return {};
    }
    out[j] = values[k];
  }
  return out;
}

bool TrajectoryExecutor::waitForServer(double timeout_s)
{
  return client_->wait_for_action_server(std::chrono::duration<double>(timeout_s));
}

std::vector<double> TrajectoryExecutor::currentPositions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_q_;
}

bool TrajectoryExecutor::waitForJointStates(double timeout_s) const
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (!currentPositions().empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool TrajectoryExecutor::waitUntilAt(
  const std::vector<double> & target, double tolerance, double timeout_s) const
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    std::vector<double> q;
    std::vector<double> v;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      q = latest_q_;
      v = latest_v_;
    }
    if (q.size() == target.size()) {
      double err = 0.0;
      double vel = 0.0;
      for (std::size_t j = 0; j < q.size(); ++j) {
        err = std::max(err, std::abs(q[j] - target[j]));
        vel = std::max(vel, j < v.size() ? std::abs(v[j]) : 0.0);
      }
      // "At rest": below 0.05 rad/s on every joint (simulated position control jitters around
      // zero velocity); the position tolerance is the caller's.
      if (err <= tolerance && vel < 0.05) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

void TrajectoryExecutor::startMonitoring()
{
  std::lock_guard<std::mutex> lock(mutex_);
  js_samples_.clear();
  ref_samples_.clear();
  monitoring_ = true;
}

TrackingResult TrajectoryExecutor::stopMonitoring()
{
  std::vector<Sample> js;
  std::vector<Sample> ref;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    monitoring_ = false;
    js.swap(js_samples_);
    ref.swap(ref_samples_);
  }
  TrackingResult r;
  if (ref.empty()) {
    return r;
  }
  std::sort(ref.begin(), ref.end(), [](const Sample & a, const Sample & b) { return a.t < b.t; });
  for (const auto & s : js) {
    // Reference with the nearest stamp, used only if within 5 ms (2.5 controller periods).
    auto it = std::lower_bound(
      ref.begin(), ref.end(), s.t, [](const Sample & a, double t) { return a.t < t; });
    const Sample * best = nullptr;
    double best_dt = 0.005;
    for (auto c : {it, it == ref.begin() ? it : it - 1}) {
      if (c != ref.end() && std::abs(c->t - s.t) <= best_dt) {
        best_dt = std::abs(c->t - s.t);
        best = &*c;
      }
    }
    if (best == nullptr) {
      continue;
    }
    ++r.samples;
    for (std::size_t j = 0; j < s.q.size(); ++j) {
      r.max_error_rad = std::max(r.max_error_rad, std::abs(s.q[j] - best->q[j]));
    }
  }
  return r;
}

ExecResult TrajectoryExecutor::execute(
  const trajectory_msgs::msg::JointTrajectory & trajectory, double timeout_s)
{
  ExecResult out;
  FJT::Goal goal;
  goal.trajectory = trajectory;
  goal.trajectory.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  startMonitoring();
  const double t0 = simNow();
  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    out.tracking = stopMonitoring();
    out.error_string = "goal not answered within 5 s";
    return out;
  }
  const auto handle = goal_future.get();
  if (!handle) {
    out.tracking = stopMonitoring();
    out.error_string = "goal rejected";
    return out;
  }
  out.accepted = true;
  auto result_future = client_->async_get_result(handle);
  if (
    result_future.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) {
    client_->async_cancel_goal(handle);
    out.tracking = stopMonitoring();
    out.error_string = "no result within the timeout";
    return out;
  }
  const auto wrapped = result_future.get();
  out.exec_time_s = simNow() - t0;
  out.tracking = stopMonitoring();
  out.succeeded = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
                  wrapped.result->error_code == FJT::Result::SUCCESSFUL;
  if (wrapped.result) {
    out.error_code = wrapped.result->error_code;
    out.error_string = wrapped.result->error_string;
  }
  const auto q = currentPositions();
  const auto & last = trajectory.points.back().positions;
  for (std::size_t j = 0; j < q.size() && j < last.size(); ++j) {
    out.final_error_rad = std::max(out.final_error_rad, std::abs(q[j] - last[j]));
  }
  return out;
}

}  // namespace pcp_direct

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Execution on the joint_trajectory_controller that the simulation already runs: a
// FollowJointTrajectory action client for the direct pipeline, and a tracking monitor that
// measures BOTH pipelines the same way (/joint_states against the controller's own reference
// from its controller_state topic, matched by stamp). No MoveIt code.
#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace pcp_direct
{

struct TrackingResult
{
  double max_error_rad = 0.0;  ///< max over samples and joints of |q - reference|
  std::size_t samples = 0;     ///< /joint_states samples matched to a reference
};

struct ExecResult
{
  bool accepted = false;
  bool succeeded = false;  ///< action result SUCCESSFUL
  int error_code = 0;      ///< FollowJointTrajectory error_code
  std::string error_string;
  double exec_time_s = 0.0;      ///< sim time, goal sent -> result received
  double final_error_rad = 0.0;  ///< max |q - last point| after the result
  TrackingResult tracking;
};

class TrajectoryExecutor
{
public:
  /// @param action_name   e.g. /joint_trajectory_controller/follow_joint_trajectory
  /// @param state_topic   e.g. /joint_trajectory_controller/controller_state
  TrajectoryExecutor(
    const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & joint_names,
    const std::string & action_name, const std::string & state_topic,
    const std::string & joint_states_topic = "/joint_states");

  bool waitForServer(double timeout_s);
  /// Latest /joint_states positions in joint_names order (empty if none received yet).
  std::vector<double> currentPositions() const;
  /// Block until joint states arrive (true) or the timeout passes.
  bool waitForJointStates(double timeout_s) const;
  /// Block until every joint is within `tolerance` of `target` and nearly at rest.
  bool waitUntilAt(const std::vector<double> & target, double tolerance, double timeout_s) const;

  /// Start/stop recording /joint_states and controller references (for either pipeline).
  void startMonitoring();
  TrackingResult stopMonitoring();

  /// Send `trajectory` (header stamp 0: the controller starts on receipt), wait for the result,
  /// record sim-time duration and tracking error.
  ExecResult execute(const trajectory_msgs::msg::JointTrajectory & trajectory, double timeout_s);

  double simNow() const { return node_->get_clock()->now().seconds(); }

private:
  using FJT = control_msgs::action::FollowJointTrajectory;
  struct Sample
  {
    double t;
    std::vector<double> q;
  };
  std::vector<double> reorder(
    const std::vector<std::string> & names, const std::vector<double> & values) const;

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  rclcpp_action::Client<FJT>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr cs_sub_;
  mutable std::mutex mutex_;
  std::vector<double> latest_q_;
  std::vector<double> latest_v_;
  bool monitoring_ = false;
  std::vector<Sample> js_samples_;
  std::vector<Sample> ref_samples_;
};

}  // namespace pcp_direct

// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// shelf_boxes_loader: publishes the shelf boxes of pcp_scenes/config/shelf.yaml as
// moveit_msgs/CollisionObject boxes on the relative topic `collision_object`, which MoveItCpp's
// planning-scene monitor (world geometry monitor) listens to in the same namespace. The direct
// checker reads the same YAML, so both pipelines see the same boxes.
//
// Parameters: shelf_yaml (string, required), period_s (double, default 1.0).
// Boxes are re-published every period: the monitor's subscription is volatile, so a subscriber
// that appears later still receives them; an ADD of an existing id replaces it with the same box.
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "pcp_direct/collision.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("shelf_boxes_loader");
  const auto yaml = node->declare_parameter<std::string>("shelf_yaml", "");
  const auto period = node->declare_parameter<double>("period_s", 1.0);
  if (yaml.empty()) {
    RCLCPP_ERROR(node->get_logger(), "parameter shelf_yaml is required");
    rclcpp::shutdown();
    return 1;
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto & box : pcp_direct::loadBoxes(yaml)) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = box.name;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {box.size.x(), box.size.y(), box.size.z()};
    geometry_msgs::msg::Pose pose;
    pose.position.x = box.position.x();
    pose.position.y = box.position.y();
    pose.position.z = box.position.z();
    pose.orientation.w = 1.0;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.pose.orientation.w = 1.0;
    objects.push_back(obj);
  }
  auto pub = node->create_publisher<moveit_msgs::msg::CollisionObject>(
    "collision_object", rclcpp::QoS(100).reliable());
  RCLCPP_INFO(
    node->get_logger(), "publishing %zu shelf boxes from %s on %s", objects.size(), yaml.c_str(),
    pub->get_topic_name());
  auto timer = node->create_wall_timer(std::chrono::duration<double>(period), [&]() {
    if (pub->get_subscription_count() == 0) {
      return;
    }
    for (const auto & obj : objects) {
      pub->publish(obj);
    }
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

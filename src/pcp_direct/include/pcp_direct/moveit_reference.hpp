// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// MoveIt-side helpers for the reference checks (agreement test, unit tests, cross-validation).
// Header-only so that the direct pipeline libraries never link MoveIt.
#pragma once

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "pcp_direct/collision.hpp"

namespace pcp_direct::moveit_reference
{

inline NamedPair orderedPair(std::string a, std::string b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return {std::move(a), std::move(b)};
}

/// The pairs MoveIt's OMPL validity checker tests for `group`: every pair of collision objects
/// (links with collision geometry, world objects) in which at least one link moves with the group
/// (FCL's `active_components_only` = the group's updated links), minus the pairs the allowed
/// collision matrix always allows.
inline std::vector<NamedPair> checkedPairs(
  const planning_scene::PlanningScene & scene, const std::string & group)
{
  const auto & model = *scene.getRobotModel();
  const auto * jmg = model.getJointModelGroup(group);
  const auto & moving = jmg->getUpdatedLinkModelsSet();
  std::vector<std::pair<std::string, bool>> objects;  // name, moves with the group
  for (const auto * link : model.getLinkModelsWithCollisionGeometry()) {
    objects.emplace_back(link->getName(), moving.count(link) > 0);
  }
  for (const auto & id : scene.getWorld()->getObjectIds()) {
    objects.emplace_back(id, false);
  }
  const auto & acm = scene.getAllowedCollisionMatrix();
  std::vector<NamedPair> pairs;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    for (std::size_t j = i + 1; j < objects.size(); ++j) {
      if (!objects[i].second && !objects[j].second) {
        continue;
      }
      collision_detection::AllowedCollision::Type type;
      if (
        acm.getEntry(objects[i].first, objects[j].first, type) &&
        type == collision_detection::AllowedCollision::ALWAYS) {
        continue;
      }
      pairs.push_back(orderedPair(objects[i].first, objects[j].first));
    }
  }
  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

/// MoveIt's verdict with the request its OMPL validity checker uses (group set, no contacts).
inline bool isColliding(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const std::string & group)
{
  collision_detection::CollisionRequest req;
  req.group_name = group;
  collision_detection::CollisionResult res;
  scene.checkCollision(req, res, state);
  return res.collision;
}

/// Every pair MoveIt reports in contact (contacts enabled, one contact per pair).
inline std::vector<NamedPair> collidingPairs(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const std::string & group)
{
  collision_detection::CollisionRequest req;
  req.group_name = group;
  req.contacts = true;
  req.max_contacts = 1000;
  req.max_contacts_per_pair = 1;
  collision_detection::CollisionResult res;
  scene.checkCollision(req, res, state);
  std::vector<NamedPair> pairs;
  for (const auto & [names, contacts] : res.contacts) {
    (void)contacts;
    pairs.push_back(orderedPair(names.first, names.second));
  }
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs;
}

/// MoveIt's global minimum distance (self and world, same group/ACM rules) and its pair.
inline PairDistance minimumDistance(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const std::string & group)
{
  collision_detection::DistanceRequest req;
  req.group_name = group;
  req.enableGroup(scene.getRobotModel());
  req.acm = &scene.getAllowedCollisionMatrix();
  req.enable_signed_distance = true;
  req.type = collision_detection::DistanceRequestType::GLOBAL;
  collision_detection::DistanceResult self;
  collision_detection::DistanceResult world;
  scene.getCollisionEnvUnpadded()->distanceSelf(req, self, state);
  scene.getCollisionEnv()->distanceRobot(req, world, state);
  const auto & best = self.minimum_distance.distance <= world.minimum_distance.distance
                        ? self.minimum_distance
                        : world.minimum_distance;
  return {orderedPair(best.link_names[0], best.link_names[1]), best.distance};
}

}  // namespace pcp_direct::moveit_reference

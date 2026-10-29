// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_direct/collision.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <coal/shape/geometric_shapes.h>
#include <yaml-cpp/yaml.h>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace pcp_direct
{
namespace
{
Eigen::Vector3d vec3(const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("shelf yaml: expected a 3-element list");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

NamedPair ordered(std::string a, std::string b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return {std::move(a), std::move(b)};
}
}  // namespace

std::string readTextFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<BoxSpec> loadBoxes(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  if (root["frame"] && root["frame"].as<std::string>() != "world") {
    throw std::runtime_error("shelf yaml: boxes must be given in the world frame");
  }
  std::vector<BoxSpec> boxes;
  for (const auto & b : root["boxes"]) {
    boxes.push_back({b["name"].as<std::string>(), vec3(b["size"]), vec3(b["position"])});
  }
  if (boxes.empty()) {
    throw std::runtime_error("shelf yaml: no boxes in " + yaml_path);
  }
  return boxes;
}

struct CollisionChecker::Impl
{
  pinocchio::Model model;
  pinocchio::GeometryModel geom;
  mutable pinocchio::Data data;
  mutable pinocchio::GeometryData gdata;
  std::vector<Eigen::Index> q_index;  ///< position in model q of each joint in joint_names order
  mutable Eigen::VectorXd q;

  explicit Impl(pinocchio::Model m, pinocchio::GeometryModel g)
  : model(std::move(m)), geom(std::move(g)), data(model), gdata(geom), q(model.nq)
  {
    q.setZero();
  }

  void setQ(const double * values) const
  {
    for (std::size_t j = 0; j < q_index.size(); ++j) {
      q[q_index[j]] = values[j];
    }
  }

  /// MoveIt's name for a geometry: its link (BODY frame) for robot meshes, its own name for boxes.
  std::string objectName(pinocchio::GeomIndex g) const
  {
    const auto & obj = geom.geometryObjects[g];
    if (obj.parentJoint == 0 && obj.parentFrame == 0) {
      return obj.name;  // a world box (attached directly to the universe frame)
    }
    return model.frames[obj.parentFrame].name;
  }

  NamedPair pairName(std::size_t k) const
  {
    const auto & cp = geom.collisionPairs[k];
    return ordered(objectName(cp.first), objectName(cp.second));
  }
};

CollisionChecker::CollisionChecker(
  const std::string & urdf_xml, const std::string & srdf_xml, const std::vector<BoxSpec> & boxes,
  const std::vector<std::string> & joint_names, const std::vector<std::string> & package_dirs)
: joint_names_(joint_names)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  pinocchio::GeometryModel geom;
  std::istringstream urdf_stream(urdf_xml);
  pinocchio::urdf::buildGeom(model, urdf_stream, pinocchio::COLLISION, geom, package_dirs);

  for (const auto & box : boxes) {
    auto shape = std::make_shared<coal::Box>(box.size.x(), box.size.y(), box.size.z());
    const pinocchio::SE3 placement(Eigen::Matrix3d::Identity(), box.position);
    geom.addGeometryObject(pinocchio::GeometryObject(box.name, 0, 0, placement, shape));
  }

  geom.addAllCollisionPairs();
  const std::size_t all_pairs = geom.collisionPairs.size();
  pinocchio::srdf::removeCollisionPairsFromXML(model, geom, srdf_xml);
  srdf_removed_ = all_pairs - geom.collisionPairs.size();

  impl_ = std::make_unique<Impl>(std::move(model), std::move(geom));
  for (const auto & name : joint_names_) {
    if (!impl_->model.existJointName(name)) {
      throw std::invalid_argument("joint '" + name + "' is not in the URDF");
    }
    const auto & joint = impl_->model.joints[impl_->model.getJointId(name)];
    if (joint.nq() != 1) {
      throw std::invalid_argument("joint '" + name + "' is not a 1-DoF joint");
    }
    impl_->q_index.push_back(joint.idx_q());
    lower_.push_back(impl_->model.lowerPositionLimit[joint.idx_q()]);
    upper_.push_back(impl_->model.upperPositionLimit[joint.idx_q()]);
  }
  if (static_cast<int>(joint_names_.size()) != impl_->model.nq) {
    throw std::invalid_argument("joint_names must list every movable joint of the URDF");
  }
}

CollisionChecker::~CollisionChecker() = default;

bool CollisionChecker::isValid(const double * q)
{
  const auto t0 = std::chrono::steady_clock::now();
  impl_->setQ(q);
  const bool colliding = pinocchio::computeCollisions(
    impl_->model, impl_->data, impl_->geom, impl_->gdata, impl_->q, true);
  const auto t1 = std::chrono::steady_clock::now();
  call_times_us_.push_back(std::chrono::duration<float, std::micro>(t1 - t0).count());
  return !colliding;
}

bool CollisionChecker::isColliding(const std::vector<double> & q) const
{
  impl_->setQ(q.data());
  return pinocchio::computeCollisions(
    impl_->model, impl_->data, impl_->geom, impl_->gdata, impl_->q, true);
}

std::vector<NamedPair> CollisionChecker::collidingPairs(const std::vector<double> & q) const
{
  impl_->setQ(q.data());
  pinocchio::computeCollisions(
    impl_->model, impl_->data, impl_->geom, impl_->gdata, impl_->q, false);
  std::vector<NamedPair> pairs;
  for (std::size_t k = 0; k < impl_->geom.collisionPairs.size(); ++k) {
    if (impl_->gdata.activeCollisionPairs[k] && impl_->gdata.collisionResults[k].isCollision()) {
      pairs.push_back(impl_->pairName(k));
    }
  }
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs;
}

PairDistance CollisionChecker::minimumDistance(const std::vector<double> & q) const
{
  impl_->setQ(q.data());
  const std::size_t k =
    pinocchio::computeDistances(impl_->model, impl_->data, impl_->geom, impl_->gdata, impl_->q);
  PairDistance out;
  if (k >= impl_->geom.collisionPairs.size()) {
    out.distance = std::numeric_limits<double>::infinity();
    return out;
  }
  out.pair = impl_->pairName(k);
  out.distance = impl_->gdata.distanceResults[k].min_distance;
  return out;
}

double CollisionChecker::pairDistance(const std::vector<double> & q, const NamedPair & pair) const
{
  impl_->setQ(q.data());
  pinocchio::updateGeometryPlacements(
    impl_->model, impl_->data, impl_->geom, impl_->gdata, impl_->q);
  const NamedPair want = ordered(pair.first, pair.second);
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t k = 0; k < impl_->geom.collisionPairs.size(); ++k) {
    if (impl_->pairName(k) != want) {
      continue;
    }
    found = true;
    pinocchio::computeDistance(impl_->geom, impl_->gdata, k);
    best = std::min(best, impl_->gdata.distanceResults[k].min_distance);
  }
  if (!found) {
    throw std::invalid_argument("pair " + want.first + " / " + want.second + " is not active");
  }
  return best;
}

std::size_t CollisionChecker::activePairCount() const { return impl_->geom.collisionPairs.size(); }

std::vector<NamedPair> CollisionChecker::activePairs() const
{
  std::vector<NamedPair> pairs;
  for (std::size_t k = 0; k < impl_->geom.collisionPairs.size(); ++k) {
    pairs.push_back(impl_->pairName(k));
  }
  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

std::size_t CollisionChecker::geometryCount() const { return impl_->geom.ngeoms; }

double CollisionChecker::medianCallTimeUs() const
{
  if (call_times_us_.empty()) {
    return 0.0;
  }
  std::vector<float> t = call_times_us_;
  const auto mid = t.begin() + static_cast<std::ptrdiff_t>(t.size() / 2);
  std::nth_element(t.begin(), mid, t.end());
  if (t.size() % 2 == 1) {
    return *mid;
  }
  const float upper = *mid;
  const float lower = *std::max_element(t.begin(), mid);
  return 0.5 * (static_cast<double>(lower) + static_cast<double>(upper));
}

double CollisionChecker::totalCallTimeS() const
{
  double sum = 0.0;
  for (float t : call_times_us_) {
    sum += t;
  }
  return sum * 1e-6;
}

}  // namespace pcp_direct

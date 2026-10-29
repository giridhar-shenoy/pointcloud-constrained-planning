// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_direct/constraint.hpp"

#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace pcp_direct
{

struct FrameKinematics::Impl
{
  pinocchio::Model model;
  mutable pinocchio::Data data;
  pinocchio::FrameIndex frame = 0;
  std::vector<Eigen::Index> q_index;
  mutable Eigen::VectorXd q;

  explicit Impl(pinocchio::Model m) : model(std::move(m)), data(model), q(model.nq) { q.setZero(); }
};

FrameKinematics::FrameKinematics(
  const std::string & urdf_xml, const std::vector<std::string> & joint_names,
  const std::string & frame)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  impl_ = std::make_unique<Impl>(std::move(model));
  if (!impl_->model.existFrame(frame)) {
    throw std::invalid_argument("frame '" + frame + "' is not in the URDF");
  }
  impl_->frame = impl_->model.getFrameId(frame);
  for (const auto & name : joint_names) {
    impl_->q_index.push_back(impl_->model.joints[impl_->model.getJointId(name)].idx_q());
  }
}

FrameKinematics::~FrameKinematics() = default;

Eigen::Isometry3d FrameKinematics::pose(const Eigen::Ref<const Eigen::VectorXd> & q) const
{
  for (std::size_t j = 0; j < impl_->q_index.size(); ++j) {
    impl_->q[impl_->q_index[j]] = q[static_cast<Eigen::Index>(j)];
  }
  pinocchio::framesForwardKinematics(impl_->model, impl_->data, impl_->q);
  const auto & M = impl_->data.oMf[impl_->frame];
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = M.rotation();
  T.translation() = M.translation();
  return T;
}

KeepUprightConstraint::KeepUprightConstraint(
  std::shared_ptr<const FrameKinematics> fk, const Eigen::Matrix3d & target,
  const Eigen::Vector3d & tolerance)
: ompl::base::Constraint(6, 3), fk_(std::move(fk)), target_(target), tolerance_(tolerance)
{
}

Eigen::Vector3d KeepUprightConstraint::error(const Eigen::Ref<const Eigen::VectorXd> & q) const
{
  const Eigen::AngleAxisd aa(fk_->pose(q).linear().transpose() * target_);
  return aa.angle() * aa.axis();
}

void KeepUprightConstraint::function(
  const Eigen::Ref<const Eigen::VectorXd> & q, Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::Vector3d e = error(q);
  for (int i = 0; i < 3; ++i) {
    out[i] = e[i] > tolerance_[i]    ? e[i] - tolerance_[i]
             : e[i] < -tolerance_[i] ? -tolerance_[i] - e[i]
                                     : 0.0;
  }
}

void KeepUprightConstraint::jacobian(
  const Eigen::Ref<const Eigen::VectorXd> & q, Eigen::Ref<Eigen::MatrixXd> out) const
{
  const Eigen::Vector3d e = error(q);
  Eigen::Vector3d sign;
  for (int i = 0; i < 3; ++i) {
    sign[i] = e[i] > tolerance_[i] ? 1.0 : e[i] < -tolerance_[i] ? -1.0 : 0.0;
  }
  constexpr double h = 1e-6;
  Eigen::VectorXd qp = q;
  Eigen::VectorXd qm = q;
  for (Eigen::Index j = 0; j < q.size(); ++j) {
    qp[j] = q[j] + h;
    qm[j] = q[j] - h;
    const Eigen::Vector3d d = (error(qp) - error(qm)) / (2.0 * h);
    for (int i = 0; i < 3; ++i) {
      out(i, j) = sign[i] * d[i];
    }
    qp[j] = q[j];
    qm[j] = q[j];
  }
}

}  // namespace pcp_direct

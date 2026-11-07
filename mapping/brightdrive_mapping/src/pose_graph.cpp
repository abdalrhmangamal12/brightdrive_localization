// PoseGraph implementation: the GTSAM side of the backend.
#include "brightdrive_mapping/pose_graph.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

#include <boost/make_shared.hpp>
#include <stdexcept>

namespace brightdrive_mapping {

namespace {
// X(k) = keyframe k, G(0) = the map origin T_mo. Symbols, never raw ints:
// a raw-int loop over the estimate would hit G(0) once it exists.
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::X;

gtsam::Pose3 toGtsam(const Eigen::Isometry3d& T) {
  return gtsam::Pose3(gtsam::Rot3(Eigen::Matrix3d(T.linear())),
                      gtsam::Point3(T.translation()));
}

Eigen::Isometry3d fromGtsam(const gtsam::Pose3& P) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = P.rotation().matrix();
  T.translation() = P.translation();
  return T;
}

// Diagonal::Sigmas, not Variances: every number in the config is a sigma.
gtsam::noiseModel::Diagonal::shared_ptr diagonalSigmas(const Vector6d& sigmas) {
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}
}  // namespace

struct PoseGraph::Impl {
  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph new_factors;  // since the last update()
  gtsam::Values new_values;                 // since the last update()
  gtsam::Values estimate;  // after the last update() or batchOptimize()
  gtsam::NonlinearFactorGraph all_factors;  // everything, for batchOptimize()
  int n_keyframes = 0;
  bool has_origin = false;

  explicit Impl(const gtsam::ISAM2Params& params) : isam(params) {}

  // Every factor goes to iSAM2 (through new_factors) and to the batch graph.
  void addFactor(const gtsam::NonlinearFactor::shared_ptr& factor) {
    new_factors.add(factor);
    all_factors.add(factor);
  }

  // The optimized pose if the key has one, else its initial guess.
  gtsam::Pose3 poseOf(gtsam::Key key) const {
    if (estimate.exists(key)) return estimate.at<gtsam::Pose3>(key);
    if (new_values.exists(key)) return new_values.at<gtsam::Pose3>(key);
    throw std::out_of_range("PoseGraph: no value for key " +
                            gtsam::Symbol(key).string());
  }
};

PoseGraph::PoseGraph(double relin_threshold, int relin_skip) {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = relin_threshold;
  params.relinearizeSkip = relin_skip;
  impl_ = std::make_unique<Impl>(params);
}

PoseGraph::~PoseGraph() = default;

void PoseGraph::addKeyframeNode(int k, const Eigen::Isometry3d& guess) {
  impl_->new_values.insert(X(k), toGtsam(guess));
  ++impl_->n_keyframes;
}

void PoseGraph::addPriorFactor(int k, const Eigen::Isometry3d& T,
                               const Vector6d& sigmas) {
  impl_->addFactor(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(k), toGtsam(T), diagonalSigmas(sigmas)));
}

void PoseGraph::addOdomFactor(int from, int to, const Eigen::Isometry3d& T_rel,
                              const Vector6d& sigmas) {
  impl_->addFactor(boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(from), X(to), toGtsam(T_rel), diagonalSigmas(sigmas)));
}

void PoseGraph::addLoopFactor(int prev, int cur, const Eigen::Isometry3d& T_rel,
                              const Vector6d& sigmas) {
  // Cauchy(1.0): the residual's weight falls as 1 / (1 + r^2), so a loop that
  // disagrees with the rest of the graph by many sigmas is nearly ignored.
  const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
      diagonalSigmas(sigmas));
  impl_->addFactor(boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(prev), X(cur), toGtsam(T_rel), robust));
}

void PoseGraph::addMapOriginNode(const Eigen::Isometry3d& T_mo0,
                                 const Vector6d& sigmas) {
  if (impl_->has_origin) {
    throw std::logic_error("PoseGraph::addMapOriginNode called twice");
  }
  impl_->new_values.insert(G(0), toGtsam(T_mo0));
  impl_->addFactor(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      G(0), toGtsam(T_mo0), diagonalSigmas(sigmas)));
  impl_->has_origin = true;
}

void PoseGraph::addGnssFactor(int k, const Eigen::Vector3d& p_map,
                              const Eigen::Matrix3d& cov,
                              const Eigen::Vector3d& lever_arm,
                              double huber_k) {
  if (!impl_->has_origin) {
    throw std::logic_error("PoseGraph::addGnssFactor before addMapOriginNode");
  }
  // Predicted antenna position T_mo * X_k * l as an expression: GTSAM derives
  // the Jacobians for both poses, so no custom factor class. The factor's
  // error is the prediction minus p_map.
  const gtsam::Pose3_ T_mo(G(0));
  const gtsam::Pose3_ X_k(X(k));
  const gtsam::Point3_ antenna = gtsam::transformFrom(
      T_mo * X_k, gtsam::Point3_(gtsam::Point3(lever_arm)));
  // Huber: quadratic inside huber_k sigmas, linear outside, so one bad fix
  // pulls with bounded force.
  const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
      gtsam::noiseModel::Gaussian::Covariance(cov));
  impl_->addFactor(boost::make_shared<gtsam::ExpressionFactor<gtsam::Point3>>(
      robust, gtsam::Point3(p_map), antenna));
}

void PoseGraph::update(int extra_iterations) {
  impl_->isam.update(impl_->new_factors, impl_->new_values);
  for (int i = 0; i < extra_iterations; ++i) impl_->isam.update();
  impl_->new_factors.resize(0);
  impl_->new_values.clear();
  impl_->estimate = impl_->isam.calculateEstimate();
}

void PoseGraph::batchOptimize() {
  // Anything added since the last update() has no estimate yet; push it in
  // first so every key in all_factors has a value.
  if (!impl_->new_factors.empty() || !impl_->new_values.empty()) update();
  if (impl_->estimate.empty()) return;
  gtsam::LevenbergMarquardtOptimizer optimizer(impl_->all_factors,
                                               impl_->estimate);
  impl_->estimate = optimizer.optimize();
}

Eigen::Isometry3d PoseGraph::keyframePose(int k) const {
  return fromGtsam(impl_->poseOf(X(k)));
}

std::vector<Eigen::Isometry3d> PoseGraph::keyframePoses() const {
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(impl_->n_keyframes);
  for (int k = 0; k < impl_->n_keyframes; ++k) poses.push_back(keyframePose(k));
  return poses;
}

std::optional<Eigen::Isometry3d> PoseGraph::mapOrigin() const {
  if (!impl_->has_origin) return std::nullopt;
  return fromGtsam(impl_->poseOf(G(0)));
}

int PoseGraph::numKeyframes() const { return impl_->n_keyframes; }

}  // namespace brightdrive_mapping

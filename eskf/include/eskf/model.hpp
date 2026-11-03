// Process and measurement model matrices as free functions: F, Q and H depend
// only on their arguments, so tests build them without a filter instance.
// Math: docs/math.md sections 7 and 8.
#pragma once

#include <Eigen/Core>
#include <stdexcept>

#include "eskf/params.hpp"
#include "eskf/so3.hpp"
#include "eskf/types.hpp"

namespace eskf {

// Error-state transition over dt (math.md 7.1). R, a_c, w_c are the values at
// the start of the step: build F before integrating the nominal.
inline Matrix15d buildF(const Eigen::Matrix3d& R, const Eigen::Vector3d& a_c,
                        const Eigen::Vector3d& w_c, double dt) {
  Matrix15d F = Matrix15d::Identity();
  F.block<3, 3>(kP, kV) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(kV, kTh) = -R * So3::skew(a_c) * dt;
  F.block<3, 3>(kV, kBa) = -R * dt;
  F.block<3, 3>(kTh, kTh) = So3::ExpSO3(-w_c * dt);
  F.block<3, 3>(kTh, kBg) = -Eigen::Matrix3d::Identity() * dt;
  return F;
}

// Process noise Q = Fi Qi Fi^T over dt, diagonal (math.md 7.3): zero for dp,
// one variance per block. White-noise sigmas depend on noise_model, bias
// sigmas do not.
inline Matrix15d buildQ(const Params& params, double dt) {
  if (!params.noise_model) {
    throw std::invalid_argument("buildQ: Params::noise_model not set");
  }
  double qa = 0.0, qg = 0.0;  // white noise -> v, theta
  switch (*params.noise_model) {
    case NoiseModel::Density:  // sigma = density, variance grows with dt
      qa = params.sigma_a * params.sigma_a * dt;
      qg = params.sigma_g * params.sigma_g * dt;
      break;
    case NoiseModel::PerSample:  // sigma = std of one reading
      qa = (params.sigma_a * dt) * (params.sigma_a * dt);
      qg = (params.sigma_g * dt) * (params.sigma_g * dt);
      break;
  }
  const double qba = params.sigma_ba * params.sigma_ba * dt;  // always density
  const double qbg = params.sigma_bg * params.sigma_bg * dt;

  Matrix15d Q = Matrix15d::Zero();
  Q.block<3, 3>(kV, kV).diagonal().setConstant(qa);
  Q.block<3, 3>(kTh, kTh).diagonal().setConstant(qg);
  Q.block<3, 3>(kBa, kBa).diagonal().setConstant(qba);
  Q.block<3, 3>(kBg, kBg).diagonal().setConstant(qbg);
  return Q;
}

// GNSS position fix with the antenna at lever_arm (IMU axes), math.md 8.4:
// H = [ I  0  -R skew(lever_arm)  0  0 ]  (3x15).
inline Eigen::Matrix<double, 3, 15> buildHGnss(
    const Eigen::Matrix3d& R, const Eigen::Vector3d& lever_arm) {
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, kP) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, kTh) = -R * So3::skew(lever_arm);
  return H;
}

}  // namespace eskf

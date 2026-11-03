// The filter: IMU predict, GNSS correct. Math: docs/math.md sections 5-9.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eskf/params.hpp"  // Params (sigmas, g, gate, noise_model)
#include "eskf/types.hpp"   // NominalState, ImuSample, GnssMeas, UpdateResult

namespace eskf {

class Eskf {
 public:
  // Throws std::invalid_argument on bad params:
  // missing noise model, sigma <= 0, gate <= 0.
  explicit Eskf(const Params& params);

  void initialize(const NominalState& x0, const Matrix15d& P0, double t0);

  void predict(const ImuSample& imu);

  UpdateResult correctGnss(const GnssMeas& z);

  // Accessors
  const NominalState& state() const { return x_; }
  const Matrix15d& covariance() const { return P_; }
  double time() const { return t_last_; }
  bool initialized() const { return initialized_; }

 private:
  friend struct EskfTestAccess;  // white-box access for test/test_jacobian.cpp

  Matrix15d buildF(const Eigen::Matrix3d& R, const Eigen::Vector3d& a_c,
                   const Eigen::Vector3d& w_c, double dt) const;

  // Process noise Fi Qi Fi^T for this dt.
  // sigma_a, sigma_g scale with dt^2; sigma_ba, sigma_bg with dt.
  Matrix15d buildQ(double dt) const;
  void integrateNominal(const Eigen::Vector3d& a_c, const Eigen::Vector3d& w_c,
                        double dt);
  // x <- x (+) dx: p += dp, v += dv, q = q * Exp(dtheta) then normalize,
  // b_a += dba, b_g += dbg.
  void inject(const Vector15d& dx);

  // dx = 0 (implicit); P <- G P G^T, G = I except the dtheta block
  // I - skew(dtheta/2). v1 may pass dtheta = 0, which gives G = I.
  void reset(const Eigen::Vector3d& dtheta);

  Params params_;
  NominalState x_;
  Matrix15d P_ = Matrix15d::Identity();
  double t_last_ = 0.0;
  bool initialized_ = false;
};

}  // namespace eskf
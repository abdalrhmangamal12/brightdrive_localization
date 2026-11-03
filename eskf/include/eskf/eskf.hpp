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
  // F, Q and H come from eskf/model.hpp; the filter only owns the state.

  // Sensor-independent half of every update (math.md 8.6, 9): S = H P H^T + V
  // by Cholesky, chi-square gate, K, Joseph form, inject, reset. M is the
  // measurement size; each correctXxx() only builds r, H, V and picks the gate.
  template <int M>
  UpdateResult applyCorrection(const Eigen::Matrix<double, M, 1>& r,
                               const Eigen::Matrix<double, M, 15>& H,
                               const Eigen::Matrix<double, M, M>& V,
                               double gate);
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
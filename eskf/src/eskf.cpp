// Eskf implementation. Math: docs/math.md sections 5-9, summary in 10.
#include "eskf/eskf.hpp"

#include <Eigen/Cholesky>  // LLT
#include <stdexcept>
#include <string>

#include "eskf/so3.hpp"

namespace eskf {

Eskf::Eskf(const Params& params) : params_(params) {
  if (!params_.noise_model) {
    throw std::invalid_argument("Params::noise_model not set");
  }
  // !(x > 0) also rejects NaN.
  const auto require_positive = [](double value, const char* name) {
    if (!(value > 0.0)) {
      throw std::invalid_argument(std::string("Params::") + name +
                                  " must be > 0");
    }
  };
  require_positive(params_.sigma_a, "sigma_a");
  require_positive(params_.sigma_g, "sigma_g");
  require_positive(params_.sigma_ba, "sigma_ba");
  require_positive(params_.sigma_bg, "sigma_bg");
  require_positive(params_.gate, "gate");
}

void Eskf::initialize(const NominalState& x0, const Matrix15d& P0, double t0) {
  // Validate first, so a bad call leaves the filter untouched.
  if (!(x0.q.norm() > 0.0)) {
    throw std::invalid_argument("initialize: x0.q is zero or NaN");
  }
  if (!P0.allFinite()) {
    throw std::invalid_argument("initialize: P0 has NaN or Inf");
  }
  if (!P0.isApprox(P0.transpose())) {  // a typo in one entry is an error
    throw std::invalid_argument("initialize: P0 must be symmetric");
  }
  // Cholesky succeeds only for a positive definite matrix: every direction
  // of the state must have a non-zero variance.
  if (Eigen::LLT<Matrix15d>(P0).info() != Eigen::Success) {
    throw std::invalid_argument("initialize: P0 must be positive definite");
  }
  x_ = x0;
  x_.q.normalize();  // never trust an outside quaternion to be unit
  P_ = P0;
  t_last_ = t0;
  initialized_ = true;
}
void Eskf::integrateNominal(const Eigen::Vector3d& a_c,
                            const Eigen::Vector3d& w_c, double dt) {
  // Order matters (math.md section 5): p uses the old v, q is updated last.
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d a = R * a_c + params_.g;
  x_.p += x_.v * dt + 0.5 * a * (dt * dt);
  x_.v += a * dt;
  x_.q = (x_.q * So3::ExpQ(w_c * dt)).normalized();
  // b_a, b_g unchanged
}
Matrix15d Eskf::buildF(const Eigen::Matrix3d& R, const Eigen::Vector3d& a_c,
                       const Eigen::Vector3d& w_c, double dt) const {
  // F = I plus five blocks (math.md 7.1). R, a_c, w_c are from the start of
  // the step: the caller builds F before integrating the nominal.
  Matrix15d F = Matrix15d::Identity();
  F.block<3, 3>(kP, kV) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(kV, kTh) = -R * So3::skew(a_c) * dt;
  F.block<3, 3>(kV, kBa) = -R * dt;
  F.block<3, 3>(kTh, kTh) = So3::ExpSO3(-w_c * dt);
  F.block<3, 3>(kTh, kBg) = -Eigen::Matrix3d::Identity() * dt;
  return F;
}

Matrix15d Eskf::buildQ(double dt) const {
  // Q = Fi Qi Fi^T, diagonal (math.md 7.3): zero for dp, then one variance
  // per block. White-noise sigmas depend on noise_model, bias sigmas do not.
  double qa = 0.0, qg = 0.0;  // white noise -> v, theta
  switch (*params_.noise_model) {
    case NoiseModel::Density:  // sigma = density, variance grows with dt
      qa = params_.sigma_a * params_.sigma_a * dt;
      qg = params_.sigma_g * params_.sigma_g * dt;
      break;
    case NoiseModel::PerSample:  // sigma = std of one reading
      qa = (params_.sigma_a * dt) * (params_.sigma_a * dt);
      qg = (params_.sigma_g * dt) * (params_.sigma_g * dt);
      break;
  }
  const double qba =
      params_.sigma_ba * params_.sigma_ba * dt;  // always density
  const double qbg = params_.sigma_bg * params_.sigma_bg * dt;

  Matrix15d Q = Matrix15d::Zero();
  Q.block<3, 3>(kV, kV).diagonal().setConstant(qa);
  Q.block<3, 3>(kTh, kTh).diagonal().setConstant(qg);
  Q.block<3, 3>(kBa, kBa).diagonal().setConstant(qba);
  Q.block<3, 3>(kBg, kBg).diagonal().setConstant(qbg);
  return Q;
}

void Eskf::predict(const ImuSample& imu) {
  if (!initialized_) {
    throw std::logic_error("Eskf::predict called before initialize()");
  }
  const double dt = imu.t - t_last_;
  if (!(dt > 0.0)) return;  // duplicate, out-of-order or NaN timestamp: skip
  // TODO(v1): dt far above the nominal rate (sensor dropout) -> warn and clamp

  // F and Q use the state at the start of the step, so build them first.
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d a_c = imu.acc - x_.b_a;
  const Eigen::Vector3d w_c = imu.gyro - x_.b_g;
  const Matrix15d F = buildF(R, a_c, w_c, dt);
  const Matrix15d Q = buildQ(dt);
  P_ = F * P_ * F.transpose() + Q;  // products: Eigen uses its own temporary
  // A sum with its own transpose aliases: .eval() makes a temporary first.
  P_ = (0.5 * (P_ + P_.transpose())).eval();
  integrateNominal(a_c, w_c, dt);
  t_last_ = imu.t;
}

UpdateResult Eskf::correctGnss(const GnssMeas& z) {
  if (!initialized_) {
    throw std::logic_error("Eskf::correctGnss called before initialize()");
  }
  // V = 0 means "not set" (types.hpp). A fix with no usable covariance is a
  // caller bug, so fail loudly instead of trusting it.
  if (!z.V.allFinite() || !z.V.isApprox(z.V.transpose()) ||
      Eigen::LLT<Eigen::Matrix3d>(z.V).info() != Eigen::Success) {
    throw std::invalid_argument(
        "correctGnss: V must be finite, symmetric, positive definite");
  }

  // Predicted antenna position and innovation (math.md 8.4, 8.6).
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d z_hat = x_.p + R * params_.lever_arm;
  UpdateResult out;
  out.innovation = z.z - z_hat;

  // H = [ I  0  -R skew(l)  0  0 ]  (3x15): position plus the lever-arm swing.
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, kP) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, kTh) = -R * So3::skew(params_.lever_arm);

  // Innovation covariance and chi-square gate. One LDLT serves both the gate
  // and the gain: no explicit inverse.
  const Eigen::Matrix3d S = H * P_ * H.transpose() + z.V;
  const Eigen::LDLT<Eigen::Matrix3d> S_ldlt(S);
  out.d2 = out.innovation.dot(S_ldlt.solve(out.innovation));
  if (!(out.d2 <= params_.gate))
    return out;  // gated (or NaN): accepted stays false

  // K = P H^T S^-1. S is symmetric, so P H^T S^-1 = (S^-1 (P H^T)^T)^T.
  const Eigen::Matrix<double, 15, 3> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, 15, 3> K =
      S_ldlt.solve(PHt.transpose()).transpose();
  const Vector15d dx = K * out.innovation;

  // Joseph form keeps P symmetric and positive for any K; then symmetrize the
  // round-off away. Named temporary: P_ on both sides would alias.
  const Matrix15d A = Matrix15d::Identity() - K * H;
  const Matrix15d P_new = A * P_ * A.transpose() + K * z.V * K.transpose();
  P_ = 0.5 * (P_new + P_new.transpose());

  inject(dx);
  reset(dx.segment<3>(kTh));
  out.accepted = true;
  return out;
}

void Eskf::inject(const Vector15d& dx) {
  // x <- x (+) dx (math.md section 9): plain sums, rotation on the right.
  x_.p += dx.segment<3>(kP);
  x_.v += dx.segment<3>(kV);
  x_.q = (x_.q * So3::ExpQ(dx.segment<3>(kTh))).normalized();
  x_.b_a += dx.segment<3>(kBa);
  x_.b_g += dx.segment<3>(kBg);
}

void Eskf::reset(const Eigen::Vector3d& dtheta) {
  // The error mean is never stored, so dx = 0 needs no code. After injection
  // the error is measured from the new nominal, whose axes turned by dtheta,
  // so the dtheta part of P rotates by half of it (math.md section 9,
  // Sola eq. 284-288). G = I in every other block.
  Matrix15d G = Matrix15d::Identity();
  G.block<3, 3>(kTh, kTh) =
      Eigen::Matrix3d::Identity() - So3::skew(0.5 * dtheta);
  const Matrix15d P_new = G * P_ * G.transpose();
  P_ = 0.5 * (P_new + P_new.transpose());
}

}  // namespace eskf

// README test 6: simulated truth, noisy IMU and GNSS, consistency of the
// estimate with its own covariance (NEES, 2-sigma count, innovation mean).
#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <random>

#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Vector3d;
using eskf::Eskf;
using eskf::kBa;
using eskf::kBg;
using eskf::kP;
using eskf::kTh;
using eskf::kV;
using eskf::Matrix15d;
using eskf::NominalState;
using eskf::Vector15d;

struct Metrics {
  double mean_nees = 0.0;
  double frac_in_2sigma = 0.0;  // over all 15 components and all fixes
  Vector3d mean_innovation = Vector3d::Zero();
  double pos_rmse = 0.0;
  int fixes = 0, accepted = 0;
};

// Circle of radius 20 m at 5 m/s, level, body x along the velocity.
// IMU at 200 Hz, GNSS at 5 Hz with a lever arm, biases random-walking.
Metrics runSim(unsigned seed, double T = 60.0) {
  const double radius = 20.0, speed = 5.0, wz = speed / radius;
  const double gnss_sigma = 0.3, gnss_dt = 0.2;
  eskf::Params prm = goodParams();
  prm.lever_arm = Vector3d(0.3, 0.0, 1.8);
  const Vector3d g(0, 0, -kG);

  std::mt19937 rng(seed);
  std::normal_distribution<double> N01(0.0, 1.0);
  auto noise3 = [&](double s) {
    return Vector3d(s * N01(rng), s * N01(rng), s * N01(rng));
  };

  auto truthAt = [&](double t) {
    NominalState x;
    const double th = wz * t;
    x.p = Vector3d(radius * std::sin(th), radius * (1 - std::cos(th)), 0);
    x.v = Vector3d(speed * std::cos(th), speed * std::sin(th), 0);
    x.q = eskf::So3::ExpQ(Vector3d(0, 0, th));
    return x;
  };
  auto accelAt = [&](double t) {
    const double th = wz * t;
    return Vector3d(-radius * wz * wz * std::sin(th),
                    radius * wz * wz * std::cos(th), 0);
  };

  // Initial estimate: truth plus a draw from P0, so NEES starts consistent.
  Vector15d sig0;
  sig0 << 1, 1, 1, 0.3, 0.3, 0.3, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.005,
      0.005, 0.005;
  Matrix15d P0 = sig0.cwiseAbs2().asDiagonal();
  NominalState truth = truthAt(0.0);
  truth.b_a = Vector3d(0.05, -0.03, 0.02);
  truth.b_g = Vector3d(0.002, -0.001, 0.003);
  Vector15d dx0;
  for (int i = 0; i < 15; ++i) dx0(i) = sig0(i) * N01(rng);
  Eskf f(prm);
  f.initialize(boxplus(truth, dx0), P0, 0.0);

  Metrics m;
  double sum_nees = 0.0, sum_pos2 = 0.0;
  int in2 = 0, total = 0;
  Vector3d sum_r = Vector3d::Zero();
  double next_gnss = gnss_dt;
  const int steps = static_cast<int>(T / kDt);
  for (int k = 1; k <= steps; ++k) {
    const double t = k * kDt;
    // Biases random-walk with the filter's own sigma.
    truth.b_a += noise3(prm.sigma_ba * std::sqrt(kDt));
    truth.b_g += noise3(prm.sigma_bg * std::sqrt(kDt));
    const NominalState tr = truthAt(t);
    const Eigen::Matrix3d R = tr.q.toRotationMatrix();
    const Vector3d acc = R.transpose() * (accelAt(t) - g) + truth.b_a +
                         noise3(prm.sigma_a / std::sqrt(kDt));
    const Vector3d gyro =
        Vector3d(0, 0, wz) + truth.b_g + noise3(prm.sigma_g / std::sqrt(kDt));
    f.predict(imuAt(t, acc, gyro));

    if (t + 1e-9 >= next_gnss) {
      next_gnss += gnss_dt;
      const Vector3d z = tr.p + R * prm.lever_arm + noise3(gnss_sigma);
      const eskf::UpdateResult u =
          f.correctGnss(gnssAt(t, z, gnss_sigma * gnss_sigma));
      ++m.fixes;
      if (u.accepted) ++m.accepted;
      if (t < 5.0) continue;  // let the initial transient settle
      NominalState full = tr;
      full.b_a = truth.b_a;
      full.b_g = truth.b_g;
      const Vector15d e = boxminus(full, f.state());
      const Matrix15d& P = f.covariance();
      sum_nees += e.dot(P.ldlt().solve(e));
      for (int i = 0; i < 15; ++i) {
        if (std::abs(e(i)) <= 2.0 * std::sqrt(P(i, i))) ++in2;
        ++total;
      }
      sum_r += u.innovation;
      sum_pos2 += e.segment<3>(kP).squaredNorm();
    }
  }
  const int n = total / 15;
  m.mean_nees = sum_nees / n;
  m.frac_in_2sigma = static_cast<double>(in2) / total;
  m.mean_innovation = sum_r / n;
  m.pos_rmse = std::sqrt(sum_pos2 / n);
  return m;
}

TEST(Synthetic, EstimateIsConsistentWithItsCovariance) {
  const Metrics m = runSim(12345);
  RecordProperty("mean_nees", m.mean_nees);
  std::printf(
      "mean NEES %.2f (15 dof)  in-2-sigma %.3f  innovation mean (%.3f, %.3f, "
      "%.3f) m  pos RMSE %.3f m  accepted %d/%d\n",
      m.mean_nees, m.frac_in_2sigma, m.mean_innovation.x(),
      m.mean_innovation.y(), m.mean_innovation.z(), m.pos_rmse, m.accepted,
      m.fixes);
  // Chi-square with 15 dof has mean 15; the time-averaged NEES must sit near
  // it.
  EXPECT_GT(m.mean_nees, 9.0);
  EXPECT_LT(m.mean_nees, 21.0);
  EXPECT_GT(m.frac_in_2sigma, 0.90);  // nominal 0.954
  EXPECT_LT(m.mean_innovation.norm(),
            0.1);              // unbiased: 0.3 m / sqrt(275) = 0.018
  EXPECT_LT(m.pos_rmse, 0.5);  // better than the 0.3 m GNSS alone in 3 axes
  EXPECT_GT(m.accepted, m.fixes * 0.9);  // 99 % gate
}

TEST(Synthetic, ConsistentAcrossSeeds) {
  for (unsigned seed : {1u, 2u, 3u}) {
    const Metrics m = runSim(seed, 30.0);
    EXPECT_GT(m.mean_nees, 8.0) << "seed " << seed;
    EXPECT_LT(m.mean_nees, 22.0) << "seed " << seed;
  }
}

}  // namespace

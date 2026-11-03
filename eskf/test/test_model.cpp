// Model matrices from eskf/model.hpp: Q for both noise models, H blocks,
// and the noise-model guard. F is covered by test_jacobian.cpp.
#include <gtest/gtest.h>

#include <stdexcept>

#include "eskf/model.hpp"
#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using eskf::kBa;
using eskf::kBg;
using eskf::kP;
using eskf::kTh;
using eskf::kV;
using eskf::Matrix15d;

TEST(Model, QDensityIsSigmaSquaredDt) {
  const eskf::Params p = goodParams();  // Density
  const Matrix15d Q = eskf::buildQ(p, kDt);
  EXPECT_DOUBLE_EQ(Q(kV, kV), p.sigma_a * p.sigma_a * kDt);
  EXPECT_DOUBLE_EQ(Q(kTh, kTh), p.sigma_g * p.sigma_g * kDt);
  EXPECT_DOUBLE_EQ(Q(kBa, kBa), p.sigma_ba * p.sigma_ba * kDt);
  EXPECT_DOUBLE_EQ(Q(kBg, kBg), p.sigma_bg * p.sigma_bg * kDt);
  EXPECT_TRUE((Q.block<3, 3>(kP, kP).isZero(0.0)));
  EXPECT_TRUE((Q - Matrix15d(Q.diagonal().asDiagonal())).isZero(0.0));
}

TEST(Model, QPerSampleIsSigmaDtSquaredForWhiteNoiseOnly) {
  eskf::Params p = goodParams();
  p.noise_model = eskf::NoiseModel::PerSample;
  const Matrix15d Q = eskf::buildQ(p, kDt);
  EXPECT_DOUBLE_EQ(Q(kV, kV), (p.sigma_a * kDt) * (p.sigma_a * kDt));
  EXPECT_DOUBLE_EQ(Q(kTh, kTh), (p.sigma_g * kDt) * (p.sigma_g * kDt));
  EXPECT_DOUBLE_EQ(Q(kBa, kBa), p.sigma_ba * p.sigma_ba * kDt);  // unchanged
  EXPECT_DOUBLE_EQ(Q(kBg, kBg), p.sigma_bg * p.sigma_bg * kDt);
}

TEST(Model, QRejectsUnsetNoiseModel) {
  eskf::Params p = goodParams();
  p.noise_model.reset();
  EXPECT_THROW(eskf::buildQ(p, kDt), std::invalid_argument);
}

TEST(Model, HGnssBlocks) {
  const Matrix3d R = eskf::So3::ExpSO3(Vector3d(0.2, -0.1, 0.7));
  const Vector3d l(0.3, -0.2, 1.8);
  const Eigen::Matrix<double, 3, 15> H = eskf::buildHGnss(R, l);
  EXPECT_TRUE((H.block<3, 3>(0, kP) - Matrix3d::Identity()).isZero(0.0));
  EXPECT_TRUE((H.block<3, 3>(0, kTh) + R * eskf::So3::skew(l)).isZero(1e-15));
  EXPECT_TRUE((H.block<3, 3>(0, kV).isZero(0.0)));
  EXPECT_TRUE((H.block<3, 3>(0, kBa).isZero(0.0)));
  EXPECT_TRUE((H.block<3, 3>(0, kBg).isZero(0.0)));
  // No lever arm: pure position measurement.
  EXPECT_TRUE(
      (eskf::buildHGnss(R, Vector3d::Zero()).block<3, 3>(0, kTh).isZero(0.0)));
}

}  // namespace

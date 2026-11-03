// README test 4: predict() guards and covariance growth.
#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <stdexcept>

#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Vector3d;
using eskf::Eskf;
using eskf::Matrix15d;
using eskf::NominalState;

TEST(Predict, ThrowsBeforeInitialize) {
  Eskf f(goodParams());
  EXPECT_THROW(f.predict(imuAt(0.0, {0, 0, kG}, {0, 0, 0})), std::logic_error);
}

TEST(Predict, SkipsNonPositiveDt) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, Matrix15d::Identity() * 1e-4, 1.0);
  const Matrix15d P0 = f.covariance();
  f.predict(imuAt(1.0, {1, 0, kG}, {0, 0, 1}));  // dt = 0
  f.predict(imuAt(0.5, {1, 0, kG}, {0, 0, 1}));  // dt < 0
  EXPECT_EQ(f.time(), 1.0);
  EXPECT_TRUE(f.state().p.isZero(0.0));
  EXPECT_TRUE(f.covariance() == P0);
}

TEST(Predict, CovarianceSymmetricGrowingPositiveDefinite) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, Matrix15d::Identity() * 1e-4, 0.0);
  const Matrix15d P0 = f.covariance();
  for (int i = 1; i <= 200; ++i)
    f.predict(imuAt(i * kDt, {0, 0, kG}, {0, 0, 0.1}));
  const Matrix15d& P = f.covariance();
  EXPECT_TRUE((P - P.transpose()).isZero(0.0));  // exactly symmetric
  EXPECT_GT(P.trace(), P0.trace());
  Eigen::SelfAdjointEigenSolver<Matrix15d> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

TEST(Predict, PositionUncertaintyGrowsFasterThanBias) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, Matrix15d::Identity() * 1e-6, 0.0);
  for (int i = 1; i <= 200; ++i)
    f.predict(imuAt(i * kDt, {0, 0, kG}, {0, 0, 0}));
  const Matrix15d& P = f.covariance();
  // Noise enters v and theta directly and reaches p by integration; the bias
  // blocks only see their small random walk.
  EXPECT_GT(P(eskf::kV, eskf::kV), P(eskf::kBa, eskf::kBa));
  EXPECT_GT(P(eskf::kTh, eskf::kTh), P(eskf::kBg, eskf::kBg));
  EXPECT_GT(P(eskf::kP, eskf::kP), 1e-6);
}

}  // namespace

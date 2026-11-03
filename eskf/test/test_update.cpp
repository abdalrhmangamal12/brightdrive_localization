// README test 5: correctGnss() numbers, gate, guards and lever arm.
#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <stdexcept>

#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using eskf::Eskf;
using eskf::GnssMeas;
using eskf::kP;
using eskf::kV;
using eskf::Matrix15d;
using eskf::NominalState;
using eskf::UpdateResult;

// The 1-axis sheet case: P_pp 0.25, P_vp 0.05, V 0.09.
Matrix15d sheetP() {
  Matrix15d P = Matrix15d::Identity() * 1e-2;
  P(kP, kP) = 0.25;
  P(kV, kV) = 0.25;
  P(kP, kV) = P(kV, kP) = 0.05;
  return P;
}

TEST(Update, ThrowsBeforeInitialize) {
  Eskf f(goodParams());
  EXPECT_THROW(f.correctGnss(gnssAt(0.0, {0, 0, 0}, 0.09)), std::logic_error);
}

TEST(Update, RejectsUnsetCovariance) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, sheetP(), 0.0);
  GnssMeas g;  // V left at Zero(): "not set"
  g.z = Vector3d(0.4, 0, 0);
  EXPECT_THROW(f.correctGnss(g), std::invalid_argument);
}

TEST(Update, OneAxisSheetNumbers) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, sheetP(), 0.0);
  const UpdateResult u = f.correctGnss(gnssAt(0.0, {0.4, 0, 0}, 0.09));
  EXPECT_TRUE(u.accepted);
  EXPECT_NEAR(u.d2, 0.4 * 0.4 / 0.34, 1e-12);  // 0.4706
  EXPECT_NEAR(f.state().p.x(), 0.25 / 0.34 * 0.4,
              1e-12);  // K_p 0.735 -> dp 0.294
  EXPECT_NEAR(f.state().v.x(), 0.05 / 0.34 * 0.4, 1e-12);  // dv 0.0588
  EXPECT_NEAR(f.covariance()(kP, kP), 0.25 - 0.25 * 0.25 / 0.34,
              1e-12);  // 0.0662
}

TEST(Update, GatesLargeInnovation) {
  Eskf f(goodParams());
  f.initialize(NominalState{}, sheetP(), 0.0);
  const UpdateResult u = f.correctGnss(gnssAt(0.0, {2.0, 0, 0}, 0.09));
  EXPECT_FALSE(u.accepted);
  EXPECT_NEAR(u.d2, 4.0 / 0.34, 1e-12);  // 11.76 > 11.34
  EXPECT_TRUE(u.innovation.isApprox(Vector3d(2.0, 0, 0)));
  EXPECT_TRUE(f.covariance() == sheetP());
  EXPECT_TRUE(f.state().p.isZero(0.0));
}

TEST(Update, LeverArmShiftsPredictedAntenna) {
  eskf::Params p = goodParams();
  p.lever_arm = Vector3d(0.3, -0.2, 1.8);
  NominalState x0;
  x0.p = Vector3d(1, 2, 3);
  x0.q = eskf::So3::ExpQ(Vector3d(0.2, -0.1, 0.7));
  Eskf f(p);
  f.initialize(x0, Matrix15d::Identity(), 0.0);
  const Vector3d z(5, 6, 7);
  const UpdateResult u =
      f.correctGnss(gnssAt(0.0, z, 1e6));  // huge V: no state change wanted
  const Vector3d want = z - (x0.p + x0.q.toRotationMatrix() * p.lever_arm);
  EXPECT_TRUE(u.innovation.isApprox(want, 1e-12));
}

TEST(Update, CovarianceStaysSymmetricPositiveAfterPredictAndUpdate) {
  eskf::Params p = goodParams();
  p.lever_arm = Vector3d(0, 0, 1.8);
  Eskf f(p);
  f.initialize(NominalState{}, Matrix15d::Identity() * 0.1, 0.0);
  for (int i = 1; i <= 200; ++i)
    f.predict(imuAt(i * kDt, {0, 0, kG}, {0, 0, 0.1}));
  const UpdateResult u = f.correctGnss(gnssAt(1.0, {0.2, -0.1, 1.9}, 0.09));
  ASSERT_TRUE(u.accepted);
  const Matrix15d& P = f.covariance();
  EXPECT_TRUE((P - P.transpose()).isZero(0.0));
  Eigen::SelfAdjointEigenSolver<Matrix15d> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

}  // namespace

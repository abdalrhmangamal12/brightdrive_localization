// README test 2: nominal integration, driven through predict().
#include <gtest/gtest.h>

#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Vector3d;
using eskf::Eskf;
using eskf::Matrix15d;
using eskf::NominalState;

Eskf makeFilter(const NominalState& x0) {
  Eskf f(goodParams());
  f.initialize(x0, Matrix15d::Identity() * 1e-4, 0.0);
  return f;
}

TEST(Nominal, LevelStillKeepsPositionAndVelocity) {
  Eskf f = makeFilter(NominalState{});
  for (int i = 1; i <= 200; ++i)
    f.predict(imuAt(i * kDt, {0, 0, kG}, {0, 0, 0}));
  EXPECT_TRUE(f.state().p.isZero(1e-12));
  EXPECT_TRUE(f.state().v.isZero(1e-12));
}

TEST(Nominal, TiltedStillKeepsPositionAndVelocity) {
  NominalState x0;
  x0.q = eskf::So3::ExpQ(Vector3d(0, 10 * kPi / 180, 0));  // 10 deg pitch
  const Vector3d a_c = x0.q.toRotationMatrix().transpose() * Vector3d(0, 0, kG);
  Eskf f = makeFilter(x0);
  for (int i = 1; i <= 200; ++i) f.predict(imuAt(i * kDt, a_c, {0, 0, 0}));
  EXPECT_TRUE(f.state().p.isZero(1e-9));
  EXPECT_TRUE(f.state().v.isZero(1e-9));
}

TEST(Nominal, YawRateIntegratesToAngle) {
  Eskf f = makeFilter(NominalState{});
  for (int i = 1; i <= 200; ++i)
    f.predict(imuAt(i * kDt, {0, 0, kG}, {0, 0, 0.1}));
  EXPECT_NEAR(yawDeg(f.state().q), 0.1 * 180.0 / kPi, 1e-9);  // 5.7296 deg
  EXPECT_TRUE(f.state().p.isZero(1e-12));
  EXPECT_TRUE(f.state().v.isZero(1e-12));
}

TEST(Nominal, SheetExampleOneStep) {
  // Yaw 90 deg, moving along +y at 5 m/s, 1 m/s^2 forward, 0.2 rad/s yaw rate.
  NominalState x0;
  x0.q = eskf::So3::ExpQ(Vector3d(0, 0, kPi / 2));
  x0.p = Vector3d(0, 10, 0);
  x0.v = Vector3d(0, 5, 0);
  Eskf f = makeFilter(x0);
  f.predict(imuAt(kDt, {1, 0, kG}, {0, 0, 0.2}));
  EXPECT_NEAR(f.state().p.y(), 10.0250125, 1e-9);
  EXPECT_NEAR(f.state().v.y(), 5.005, 1e-9);
  EXPECT_NEAR(yawDeg(f.state().q), 90.0 + 0.001 * 180.0 / kPi, 1e-9);  // 90.057
}

}  // namespace

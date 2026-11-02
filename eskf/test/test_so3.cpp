// Unit tests for eskf::So3. One property per test, tolerances explicit.
#include <gtest/gtest.h>

#include <algorithm>

#include "eskf/so3.hpp"

namespace {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using eskf::So3;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-12;

TEST(So3, SkewMatchesCross) {
  const Vector3d a(0.3, -1.2, 2.0);
  const Vector3d b(1.0, 0.5, -0.7);
  // [a]x b is the cross product a x b.
  EXPECT_TRUE((So3::skew(a) * b - a.cross(b)).isZero(kTol));
}

TEST(So3, SkewIsAntisymmetric) {
  const Matrix3d S = So3::skew(Vector3d(0.3, -1.2, 2.0));
  EXPECT_TRUE((S + S.transpose()).isZero(kTol));
}

TEST(So3, ExpOfZeroIsIdentity) {
  const Quaterniond q = So3::ExpQ(Vector3d::Zero());
  EXPECT_NEAR(q.w(), 1.0, kTol);
  EXPECT_TRUE(q.vec().isZero(kTol));
  EXPECT_TRUE(
      (So3::ExpSO3(Vector3d::Zero()) - Matrix3d::Identity()).isZero(kTol));
}

TEST(So3, NinetyDegYawMapsXtoY) {
  const Matrix3d R = So3::ExpSO3(Vector3d(0.0, 0.0, kPi / 2));
  EXPECT_TRUE((R * Vector3d::UnitX() - Vector3d::UnitY()).isZero(kTol));
}

TEST(So3, HalfTurnsCompose) {
  // Two 45 deg yaws equal one 90 deg yaw. q and -q are the same rotation.
  const Quaterniond half = So3::ExpQ(Vector3d(0.0, 0.0, kPi / 4));
  const Quaterniond q = half * half;
  const Quaterniond ref = So3::ExpQ(Vector3d(0.0, 0.0, kPi / 2));
  const double d_plus = (q.coeffs() - ref.coeffs()).norm();
  const double d_minus = (q.coeffs() + ref.coeffs()).norm();
  EXPECT_LT(std::min(d_plus, d_minus), kTol);
}

TEST(So3, TinyAngleStaysUnit) {
  // Below the small-angle threshold: first-order branch, still unit length.
  const Quaterniond q = So3::ExpQ(Vector3d(1e-10, 0.0, 0.0));
  EXPECT_NEAR(q.norm(), 1.0, 1e-15);
}

TEST(So3, ExpIsInSO3) {
  const Matrix3d R = So3::ExpSO3(Vector3d(0.3, -0.2, 0.5));
  EXPECT_TRUE((R.transpose() * R - Matrix3d::Identity()).isZero(kTol));
  EXPECT_NEAR(R.determinant(), 1.0, kTol);
}

}  // namespace

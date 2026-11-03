// README test 3: buildF (model.hpp) against a numerical Jacobian of predict().
#include <gtest/gtest.h>

#include "eskf/model.hpp"
#include "test_helpers.hpp"

namespace {

using namespace eskf_test;
using Eigen::Vector3d;
using eskf::Eskf;
using eskf::Matrix15d;
using eskf::NominalState;
using eskf::Vector15d;

// Largest entry of |F - F_numeric| for one step of size dt.
double maxJacobianError(double dt) {
  NominalState x;
  x.p = Vector3d(1, 2, 3);
  x.v = Vector3d(0.5, -0.2, 0.1);
  x.q = eskf::So3::ExpQ(Vector3d(0.05, -0.1, 0.5));
  x.b_a = Vector3d(0.1, -0.05, 0.02);
  x.b_g = Vector3d(0.01, -0.02, 0.005);
  const Vector3d acc(0.5, -0.3, 9.9), gyro(0.1, -0.2, 0.3);
  const eskf::ImuSample imu = imuAt(dt, acc, gyro);
  const Matrix15d P0 = Matrix15d::Identity();

  Eskf f(goodParams());
  f.initialize(x, P0, 0.0);
  const Matrix15d F =
      eskf::buildF(x.q.toRotationMatrix(), acc - x.b_a, gyro - x.b_g, dt);
  f.predict(imu);

  const double eps = 1e-6;
  Matrix15d Fnum;
  for (int j = 0; j < 15; ++j) {
    Vector15d dx = Vector15d::Zero();
    dx(j) = eps;
    Eskf fj(goodParams());
    fj.initialize(boxplus(x, dx), P0, 0.0);
    fj.predict(imu);  // the perturbed filter subtracts its own biases
    Fnum.col(j) = boxminus(fj.state(), f.state()) / eps;
  }
  return (F - Fnum).cwiseAbs().maxCoeff();
}

TEST(Jacobian, FMatchesNumericalJacobianSmallDt) {
  EXPECT_LT(maxJacobianError(1e-4), 1e-6);
}

TEST(Jacobian, FMatchesNumericalJacobianAt200Hz) {
  // F is first order in dt. The nominal integration carries 0.5*a*dt^2, so
  // the position rows differ by about 0.5*|a_c|*dt^2 = 1.2e-4 at 200 Hz.
  EXPECT_LT(maxJacobianError(kDt), 2e-4);
}

}  // namespace

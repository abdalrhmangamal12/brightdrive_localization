// Small helpers shared by the ESKF tests. Not a test target itself.
#pragma once

#include <cmath>

#include "eskf/eskf.hpp"
#include "eskf/so3.hpp"

namespace eskf_test {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.005;  // 200 Hz
constexpr double kG = 9.81;

// Valid params: density model, plausible MEMS numbers, default gate 11.34.
inline eskf::Params goodParams() {
  eskf::Params p;
  p.noise_model = eskf::NoiseModel::Density;
  p.sigma_a = 0.05;   // m/s^2/sqrt(Hz)
  p.sigma_g = 0.002;  // rad/s/sqrt(Hz)
  p.sigma_ba = 1e-4;  // m/s^3/sqrt(Hz)
  p.sigma_bg = 1e-5;  // rad/s^2/sqrt(Hz)
  return p;
}

inline eskf::ImuSample imuAt(double t, const Eigen::Vector3d& acc,
                             const Eigen::Vector3d& gyro) {
  eskf::ImuSample s;
  s.t = t;
  s.acc = acc;
  s.gyro = gyro;
  return s;
}

inline eskf::GnssMeas gnssAt(double t, const Eigen::Vector3d& z, double var) {
  eskf::GnssMeas g;
  g.t = t;
  g.z = z;
  g.V = Eigen::Matrix3d::Identity() * var;
  return g;
}

inline double yawDeg(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0)) * 180.0 / kPi;
}

// x (+) dx, same rule as Eskf::inject.
inline eskf::NominalState boxplus(const eskf::NominalState& x,
                                  const eskf::Vector15d& dx) {
  eskf::NominalState t = x;
  t.p += dx.segment<3>(eskf::kP);
  t.v += dx.segment<3>(eskf::kV);
  t.q = (x.q * eskf::So3::ExpQ(dx.segment<3>(eskf::kTh))).normalized();
  t.b_a += dx.segment<3>(eskf::kBa);
  t.b_g += dx.segment<3>(eskf::kBg);
  return t;
}

// a (-) b: the error state that takes b to a (local rotation error).
inline eskf::Vector15d boxminus(const eskf::NominalState& a,
                                const eskf::NominalState& b) {
  eskf::Vector15d d;
  d.segment<3>(eskf::kP) = a.p - b.p;
  d.segment<3>(eskf::kV) = a.v - b.v;
  const Eigen::AngleAxisd aa(b.q.conjugate() * a.q);
  d.segment<3>(eskf::kTh) = aa.angle() * aa.axis();
  d.segment<3>(eskf::kBa) = a.b_a - b.b_a;
  d.segment<3>(eskf::kBg) = a.b_g - b.b_g;
  return d;
}

}  // namespace eskf_test

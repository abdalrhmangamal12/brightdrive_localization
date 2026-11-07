// State and measurement types for the ESKF.
// Math and conventions are in docs/math.md (sections 1 and 2).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf {

// Error-state size: dp, dv, dtheta, dba, dbg (3 each).
inline constexpr int kErrorDim = 15;

// Eigen only has short names up to size 4.
using Matrix15d = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using Vector15d = Eigen::Matrix<double, kErrorDim, 1>;

// Start index of each 3-element block in the error state.
// Order: dp, dv, dtheta, dba, dbg
inline constexpr int kP = 0;
inline constexpr int kV = 3;
inline constexpr int kTh = 6;
inline constexpr int kBa = 9;
inline constexpr int kBg = 12;

// Nominal state. p and v are in the world frame, the biases in IMU axes.
// q rotates IMU vectors into the world frame (Hamilton).
struct NominalState {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
};

// Raw IMU reading. acc includes gravity (specific force).
struct ImuSample {
  double t = 0.0;                                  // sensor time [s]
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();   // [m/s^2]
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  // [rad/s]
};

// GNSS position already converted to the world frame.
struct GnssMeas {
  double t = 0.0;
  Eigen::Vector3d z = Eigen::Vector3d::Zero();  // [m]
  // covariance [m^2], not std. Zero = "not set": correctGnss() rejects it.
  Eigen::Matrix3d V = Eigen::Matrix3d::Zero();
};

// Pose of the IMU in the world frame, e.g. a LiDAR scan match moved to the
// IMU with the extrinsics. V is 6x6 in the order [position, rotation]: m^2
// for p, rad^2 for the rotation error on the right (q_meas = q * Exp(e), the
// same local error as the state). Zero = "not set": correctPose() rejects it.
struct PoseMeas {
  double t = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();            // [m]
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // IMU -> world
  Eigen::Matrix<double, 6, 6> V = Eigen::Matrix<double, 6, 6>::Zero();
};

// What correctGnss() and correctPose() report back. d2 is used by the health
// monitor.
struct UpdateResult {
  bool accepted = false;
  double d2 = 0.0;
  Eigen::VectorXd innovation;  // size of the measurement: 3 GNSS, 6 pose
};

}  // namespace eskf

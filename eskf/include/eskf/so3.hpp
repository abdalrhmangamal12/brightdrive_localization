// Rotation helpers on SO(3). See docs/math.md section 3.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf {

struct So3 {
  // [v]x : skew(v) * w == v.cross(w)
  static Eigen::Matrix3d skew(const Eigen::Vector3d& v);

  // Rotation vector (axis * angle, rad) to a unit quaternion, Hamilton.
  // Meant to be multiplied on the right: q_new = q * ExpQ(theta).
  static Eigen::Quaterniond ExpQ(const Eigen::Vector3d& theta);

  // Same rotation as a 3x3 matrix.
  static Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& theta);

  // Inverse of ExpQ: unit quaternion to rotation vector (axis * angle, rad),
  // angle in [0, pi]. q and -q give the same vector.
  static Eigen::Vector3d Log(const Eigen::Quaterniond& q);
};

}  // namespace eskf
// Rotation helpers on SO(3). Math: docs/math.md section 3.
#include "eskf/so3.hpp"

namespace eskf {

namespace {
// Below this angle [rad] ExpQ uses the first-order form, so there is no
// division by a vanishing norm when the IMU is still.
constexpr double kSmallAngle = 1e-8;
}  // namespace

Eigen::Matrix3d So3::skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<    0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return m;
}

Eigen::Quaterniond So3::ExpQ(const Eigen::Vector3d& theta) {
  const double angle = theta.norm();
  if (angle < kSmallAngle) {
    // [1, theta/2] then normalize. Quaterniond takes (w, x, y, z).
    return Eigen::Quaterniond(1.0, 0.5 * theta.x(), 0.5 * theta.y(),
                              0.5 * theta.z()).normalized();
  }
  const Eigen::Vector3d axis = theta / angle;  // angle > 0 here: unit axis
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

Eigen::Matrix3d So3::ExpSO3(const Eigen::Vector3d& theta) {
  return ExpQ(theta).toRotationMatrix();
}

}  // namespace eskf

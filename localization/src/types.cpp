#include "brightdrive_localization/types.hpp"

#include <algorithm>
#include <limits>

namespace brightdrive_localization {

double Scan::tMin() const {
  return time.empty() ? t_stamp : *std::min_element(time.begin(), time.end());
}

double Scan::tMax() const {
  return time.empty() ? t_stamp : *std::max_element(time.begin(), time.end());
}

const char* toString(Lifecycle s) {
  switch (s) {
    case Lifecycle::kWaitData:
      return "WAIT_DATA";
    case Lifecycle::kInitializing:
      return "INITIALIZING";
    case Lifecycle::kTracking:
      return "TRACKING";
  }
  return "?";
}

const char* toString(Health h) {
  switch (h) {
    case Health::kNominal:
      return "NOMINAL";
    case Health::kDegraded:
      return "DEGRADED";
    case Health::kDeadReckoning:
      return "DEAD_RECKONING";
    case Health::kFailed:
      return "FAILED";
  }
  return "?";
}

Eigen::Isometry3d isometryFromXyzRpyDeg(const Eigen::Vector3d& xyz,
                                        const Eigen::Vector3d& rpy_deg) {
  const Eigen::Vector3d rpy = rpy_deg * M_PI / 180.0;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  T.translation() = xyz;
  return T;
}

double rotationAngle(const Eigen::Matrix3d& R) {
  // trace = 1 + 2 cos(angle); clamp against round-off.
  const double c = std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c);
}

double yawOf(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

}  // namespace brightdrive_localization

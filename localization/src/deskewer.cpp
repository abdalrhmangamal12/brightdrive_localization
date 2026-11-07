#include "brightdrive_localization/deskewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace brightdrive_localization {

namespace {

// Point times closer than 0.1 us share one transform.
constexpr double kTimeBucketsPerSecond = 1e7;
// Coverage tolerance at both ends of the sweep [s].
constexpr double kSameTime = 1e-9;

Eigen::Isometry3d poseIn(const std::vector<PoseSample>& poses, double t) {
  auto hi =
      std::lower_bound(poses.begin(), poses.end(), t,
                       [](const PoseSample& s, double v) { return s.t < v; });
  if (hi == poses.end()) return poses.back().T_map_imu;
  if (hi == poses.begin()) return hi->T_map_imu;
  return interpolatePose(*std::prev(hi), *hi, t);
}

}  // namespace

Deskewer::Deskewer(const Eigen::Isometry3d& T_imu_lidar)
    : T_imu_lidar_(T_imu_lidar) {}

std::optional<double> Deskewer::apply(
    Scan& scan, const std::vector<PoseSample>& poses) const {
  if (scan.size() == 0 || poses.empty()) return std::nullopt;
  const double t_min = scan.tMin();
  const double t_ref = scan.tMax();
  if (poses.front().t > t_min + kSameTime ||
      poses.back().t < t_ref - kSameTime) {
    return std::nullopt;
  }
  const Eigen::Isometry3d T_ref_inv =
      (poseIn(poses, t_ref) * T_imu_lidar_).inverse();
  std::unordered_map<int64_t, Eigen::Isometry3d> rel;
  for (size_t i = 0; i < scan.size(); ++i) {
    const double t = scan.time[i];
    const int64_t key = std::llround(t * kTimeBucketsPerSecond);
    auto it = rel.find(key);
    if (it == rel.end()) {
      it = rel.emplace(key, T_ref_inv * poseIn(poses, t) * T_imu_lidar_).first;
    }
    Eigen::Vector4d& p = scan.points[i];
    p.head<3>() = it->second * Eigen::Vector3d(p.head<3>());
    p.w() = 1.0;
  }
  return t_ref;
}

}  // namespace brightdrive_localization

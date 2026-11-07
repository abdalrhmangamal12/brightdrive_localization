// Removes the motion of the car from a LiDAR sweep: every point is moved into
// the lidar frame at the sweep's reference time t_ref = max(point time), with
// the filter's own IMU poses (rotation and translation), so a 40 ms sweep at
// 15 m/s is not smeared by 0.6 m along the road.
#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

class Deskewer {
 public:
  // T_imu_lidar: the lidar frame in the IMU frame (T_base_imu^-1 T_base_lidar).
  explicit Deskewer(const Eigen::Isometry3d& T_imu_lidar);

  // poses: T_map_imu samples in time order covering [scan.tMin(),
  // scan.tMax()] (StateHistory::posesCovering). Each point p measured at t
  // becomes T_lidar(t_ref)^-1 T_lidar(t) p with T_lidar = T_map_imu
  // T_imu_lidar; one transform per distinct point time (CARLA: 5 slices per
  // sweep). Returns t_ref, or nullopt (scan untouched) when the poses do not
  // cover the sweep or the scan is empty.
  std::optional<double> apply(Scan& scan,
                              const std::vector<PoseSample>& poses) const;

 private:
  Eigen::Isometry3d T_imu_lidar_;
};

}  // namespace brightdrive_localization

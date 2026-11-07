// Plain data shared by the estimation plumbing and the ROS side.
// Frames: map (georeferenced, name from map.yaml), odom, base_link = IMU,
// lidar. Times are header stamps in seconds.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

namespace brightdrive_localization {

// One LiDAR sweep in the lidar frame. time[i] is the absolute measurement
// time of point i, already built from the per-point field with the points.*
// rules (params.hpp), so t_ref = max(time) and the span is max - min.
struct Scan {
  double t_stamp = 0.0;                 // header stamp [s]
  std::vector<Eigen::Vector4d> points;  // x y z 1
  std::vector<double> time;             // [s], same size as points

  size_t size() const { return points.size(); }
  double tMin() const;
  double tMax() const;  // the reference time of a deskewed sweep
};

// What ScanMatcher::align reports. T_map_lidar is small_gicp's
// T_target_source with the map as target. H is the Gauss-Newton information
// of the right perturbation [rx ry rz tx ty tz] of T_map_lidar at the last
// linearization; error is the sum of 0.5 r^T M r over the inliers,
// dimensionless (small_gicp regularizes every covariance to eigenvalues
// 1e-3, 1, 1).
struct MatchResult {
  bool converged = false;
  int iterations = 0;
  int num_inliers = 0;
  int num_source = 0;
  double error = 0.0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  double time_ms = 0.0;

  double inlierRatio() const {
    return num_source > 0 ? static_cast<double>(num_inliers) / num_source : 0.0;
  }
  double errorPerInlier() const {
    return num_inliers > 0 ? error / num_inliers : 0.0;
  }
};

// The `map:` block of map.yaml, the contract between mapping and
// localization. pcd_path is resolved against the yaml's folder.
struct MapInfo {
  std::string pcd_path;
  std::string frame;
  bool georeferenced = false;
  std::string projection;  // geo::parseProjection name
  double origin_lat_deg = 0.0;
  double origin_lon_deg = 0.0;
  double origin_alt_m = 0.0;
  double resolution_m = 0.0;
};

enum class Lifecycle { kWaitData, kInitializing, kTracking };
enum class Health { kNominal, kDegraded, kDeadReckoning, kFailed };

const char* toString(Lifecycle s);
const char* toString(Health h);

// xyz [m] and roll, pitch, yaw [deg] -> T (rotation = Rz(yaw) Ry(pitch)
// Rx(roll)).
Eigen::Isometry3d isometryFromXyzRpyDeg(const Eigen::Vector3d& xyz,
                                        const Eigen::Vector3d& rpy_deg);

// Angle of a rotation [rad] and yaw [rad] of a rotation, for error reports.
double rotationAngle(const Eigen::Matrix3d& R);
double yawOf(const Eigen::Matrix3d& R);

}  // namespace brightdrive_localization

// Data records shared by the backend: keyframes, GNSS fixes, loop edges.
// Design: mapping/brightdrive_mapping/README.md (class diagram).
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace brightdrive_mapping {

using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using CloudConstPtr = pcl::PointCloud<pcl::PointXYZI>::ConstPtr;

// Eigen only has short names up to size 4. GTSAM Pose3 order: rot x, y, z
// then trans x, y, z.
using Vector6d = Eigen::Matrix<double, 6, 1>;

// One keyframe. The optimized pose is not stored here: it lives in the pose
// graph and is read back when needed, so nothing goes stale after a loop
// closure.
struct Keyframe {
  int id = 0;
  double t = 0.0;  // scan end time [s], the stamp both inputs carry
  // Body (IMU) pose in the front-end's odometry frame. Never changes.
  Eigen::Isometry3d T_odom = Eigen::Isometry3d::Identity();
  CloudConstPtr cloud_body;  // range-cropped, voxelled scan in the body frame
};

// A GNSS fix already converted to map metres (geo::llaToMap).
struct GnssFix {
  double t = 0.0;
  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();  // antenna in map [m]
  // Covariance [m^2], not std. Zero = the message had none;
  // Backend::addGnssFix replaces it with diag(gnss_default_sigma^2).
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

// An accepted loop closure: cur seen from prev, T_rel = X_prev^-1 * X_cur.
struct LoopEdge {
  int cur = 0;
  int prev = 0;
  Eigen::Isometry3d T_rel = Eigen::Isometry3d::Identity();
  double fitness = 0.0;  // ICP mean squared point distance [m^2]
};

// What the core cannot know about a run and map.yaml should record anyway:
// where the inputs came from and which code made the map. The node fills it
// from parameters; an unknown entry stays empty.
struct MapSource {
  std::string bag;              // rosbag2 directory replayed, empty when live
  std::string frontend_commit;  // LIO front-end revision
  std::string backend_commit;   // this package's revision
};

}  // namespace brightdrive_mapping

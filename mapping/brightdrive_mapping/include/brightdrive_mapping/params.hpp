// Backend settings, loaded once at startup from config/backend.yaml.
// Every noise value is a sigma (not a variance) in GTSAM Pose3 order:
// rot x, y, z [rad] then trans x, y, z [m].
#pragma once

#include <Eigen/Core>
#include <string>

#include "brightdrive_mapping/types.hpp"  // Vector6d

namespace brightdrive_mapping {

struct BackendConfig {
  // Left at 0 on purpose: the Backend constructor rejects values <= 0, so a
  // key missing from the YAML fails at startup instead of silently.

  // GNSS antenna position in the IMU (body) frame [m].
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();

  // Keyframe selection: a scan becomes a keyframe after this much motion
  // since the last one.
  double kf_dist = 0.0;   // [m]
  double kf_angle = 0.0;  // [rad]
  // The cloud kept per keyframe: range crop, then voxel.
  double kf_leaf = 0.0;    // [m]
  double min_range = 0.0;  // [m]
  double max_range = 0.0;  // [m]

  // Graph noise.
  Vector6d prior_sigma = Vector6d::Zero();   // on X(0)
  Vector6d odom_sigma = Vector6d::Zero();    // between consecutive keyframes
  Vector6d origin_sigma = Vector6d::Zero();  // on G(0) = T_mo

  // GNSS.
  bool gnss_enable = true;
  double align_distance = 0.0;  // keyframe path before the first T_mo fit [m]
  double max_time_shift = 0.0;  // fix-to-keyframe time gap, else dropped [s]
  double gnss_huber_k = 0.0;    // Huber threshold [sigmas]
  double gnss_default_sigma = 0.0;  // when the fix has no covariance [m]

  // Loop closure.
  bool loop_enable = true;
  double loop_radius = 0.0;     // candidate must be this close [m]
  double loop_time_diff = 0.0;  // and at least this much older [s]
  int loop_search_num = 0;      // ICP target = candidate +- this many keyframes
  double loop_icp_leaf = 0.0;   // [m]
  double loop_fitness = 0.0;    // accept below this ICP fitness [m^2]
  // Loop factor sigma = sqrt(fitness * scale): fitness is a mean squared
  // distance [m^2], so the scales turn it into rad^2 and m^2.
  double loop_rot_scale = 0.0;
  double loop_trans_scale = 0.0;

  // Frames and output. Both frame names are recorded in map.yaml.
  std::string map_frame;         // georeferenced frame of the saved map
  std::string origin_frame;      // the front-end's fixed frame (TF child)
  std::string map_name;          // <dir>/<map_name>.pcd
  double save_resolution = 0.0;  // voxel size of the saved map [m]
  // Also write every keyframe cloud as <dir>/keyframes/NNNNNN.pcd in the map
  // frame with its pose in the PCD header: the input of per-frame
  // dynamic-object filters (DUFOMap, DynamicMap_Benchmark unified format).
  bool save_keyframes = false;
};

}  // namespace brightdrive_mapping

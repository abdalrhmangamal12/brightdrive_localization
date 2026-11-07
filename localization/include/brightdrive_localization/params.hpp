// Localizer settings, loaded once at startup from config/localization.yaml.
// Values left at 0 on purpose: validate() rejects values <= 0, so a key
// missing from the YAML fails at startup instead of silently (the rule of
// mapping's BackendConfig). Keys that may legitimately be 0 or negative say so.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "eskf/params.hpp"

namespace brightdrive_localization {

// Sensor mounts, from sim/config/sensors.yaml by hand (not /tf_static).
struct ExtrinsicsConfig {
  Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
  double imu_time_offset_s = 0.0;  // added to IMU stamps; may be 0 or negative
};

// How a per-point time is read from the cloud. t_i = stamp + time_scale *
// field_i when stamp_is == "start"; t_i = stamp - time_scale * (max(field) -
// field_i) when stamp_is == "end". Field types float32, float64, uint32.
struct PointsConfig {
  std::string time_field;  // "time" (CARLA bridge, Velodyne), "t" (Ouster), ...
  double time_scale = 0.0;  // field unit -> seconds: 1.0 for s, 1e-9 for ns
  std::string stamp_is;     // "start" | "end"
};

struct MatcherConfig {
  double map_voxel_m = 0.0;             // fine GaussianVoxelMap
  double coarse_voxel_m = 0.0;          // coarse GaussianVoxelMap (initializer)
  double source_leaf_m = 0.0;           // voxel downsampling of a sweep
  double max_corr_dist_m = 0.0;         // correspondence rejector, fine map
  double coarse_max_corr_dist_m = 0.0;  // same, coarse map
  int max_iterations = 0;
  int num_threads = 0;
  int cov_neighbors = 0;     // neighbours per point covariance (map and source)
  double min_range_m = 0.0;  // the car's own returns
  double max_range_m = 0.0;
};

// The fixed measurement noise of a LiDAR pose (R floors), §6 of the plan.
struct MeasConfig {
  double sigma_pos_m = 0.0;
  double sigma_rot_rad = 0.0;
};

// Quality checks before a match reaches the filter; the first failure is the
// reject code.
struct AcceptConfig {
  double min_inlier_ratio = 0.0;
  double max_error_per_inlier = 0.0;
  double max_jump_m = 0.0;
  double max_jump_rad = 0.0;
  double max_age_s = 0.0;
};

struct HistoryConfig {
  double keep_s =
      0.0;  // poses and events kept for deskew, forwarding, init catch-up
};

struct InitConfig {
  int yaw_steps = 0;    // headings tried at the seed on the coarse map
  int refine_top = 0;   // best candidates refined on the fine map
  double margin = 0.0;  // inlier-ratio lead of the best over the second
  int confirm_n = 0;    // accepted matches in a row before TRACKING
  double timeout_s = 0.0;
  double seed_gnss_max_age_s = 0.0;  // a younger fix is the position seed
  double still_window_s = 0.0;       // gyro mean -> b_g on a cold start
  double p0_sigma_pos_m = 0.0;
  double p0_sigma_yaw_deg = 0.0;
  double p0_sigma_rollpitch_deg = 0.0;
  double p0_sigma_vel_mps = 0.0;  // car still at init
  double p0_sigma_vel_moving_mps =
      0.0;  // no still window seen, or re-init after FAILED
  double p0_sigma_ba = 0.0;
  double p0_sigma_bg = 0.0;
};

struct HealthConfig {
  double scan_timeout_s = 0.0;
  double gnss_timeout_s = 0.0;
  double dr_timeout_s = 0.0;
  double degraded_sigma_m = 0.0;
  int fail_consecutive_gated = 0;
  double fail_timeout_s = 0.0;
  double fail_sigma_m = 0.0;
};

struct LocalizerConfig {
  eskf::Params eskf;  // the Eskf constructor validates these
  // Chi-square gate of the 6-DoF pose update. Read from eskf.gate_pose; it
  // moves into eskf::Params when the filter gets correctPose().
  double gate_pose = 0.0;
  double gnss_default_sigma_m = 0.0;  // when a fix carries no covariance
  ExtrinsicsConfig extrinsics;
  PointsConfig points;
  MatcherConfig matcher;
  MeasConfig meas;
  AcceptConfig accept;
  HistoryConfig history;
  InitConfig init;
  HealthConfig health;
};

// Throws std::invalid_argument naming the first bad key. Constructs an
// eskf::Eskf once so the filter's own checks run here too.
void validate(const LocalizerConfig& c);

}  // namespace brightdrive_localization

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
  int min_source_points = 0;  // after preprocess; below 5 no covariances exist
  double min_inlier_ratio = 0.0;
  double max_error_per_inlier = 0.0;
  double max_jump_m = 0.0;
  double max_jump_rad = 0.0;
  double max_age_s = 0.0;
};

struct ScanConfig {
  // How long a sweep waits for the IMU sample at its end before it is
  // skipped: deskew and the prediction need the filter's pose at t_ref.
  double imu_wait_s = 0.0;  // wall time
};

struct HistoryConfig {
  double keep_s =
      0.0;  // poses and events kept for deskew, forwarding, init catch-up
};

struct InitConfig {
  int yaw_steps = 0;       // headings tried at the seed on the coarse map
  int refine_top = 0;      // best candidates refined on the fine map
  double margin = 0.0;     // error-per-inlier lead of the best over the second
                           // distinct pose (lower error is better)
  int confirm_n = 0;       // accepted matches in a row before TRACKING
  double timeout_s = 0.0;  // INITIALIZING longer than this: status warning
  // Position seed: a fix at most this old with at most this horizontal sigma;
  // else /initialpose when the car has been still since its stamp.
  double seed_gnss_max_age_s = 0.0;
  double max_seed_gnss_sigma_m = 0.0;
  // GNSS velocity at init: least squares over the fixes of the window.
  double vel_window_s = 0.0;
  int vel_min_fixes = 0;
  double vel_min_span_s = 0.0;       // first to last fix
  double vel_sigma_floor_mps = 0.0;  // lower bound of its sigma
  // Still test over the window before the sweep; b_g = gyro mean when still.
  double still_window_s = 0.0;
  double still_max_gyro_radps = 0.0;         // every sample
  double still_max_acc_std_mps2 = 0.0;       // spread of |acc|
  double still_max_acc_mean_dev_mps2 = 0.0;  // |mean |acc| - |g||
  // GNSS speed bound = max(still_max_gnss_speed_mps, still_gnss_speed_sigmas
  // * sigma of the GNSS velocity): the 1 s fit alone is ~0.33 m/s noisy.
  double still_max_gnss_speed_mps = 0.0;
  double still_gnss_speed_sigmas = 0.0;
  // Heading: above cog_min_speed_mps the yaw hint is the GNSS course over
  // ground; above cog_check_min_speed_mps a winner further than
  // cog_max_diff_deg from it is rejected.
  double cog_min_speed_mps = 0.0;
  double cog_check_min_speed_mps = 0.0;
  double cog_max_diff_deg = 0.0;
  // A discarded filter's b_g seeds the next start only below this, per axis.
  double carry_bg_max_radps = 0.0;
  double p0_sigma_pos_m = 0.0;
  double p0_sigma_yaw_deg = 0.0;
  double p0_sigma_rollpitch_deg = 0.0;
  double p0_sigma_vel_mps = 0.0;         // the still test says standing
  double p0_sigma_vel_moving_mps = 0.0;  // moving, no GNSS velocity
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
  eskf::Params eskf;  // the Eskf constructor validates these; gate_pose here
  double gnss_default_sigma_m = 0.0;  // when a fix carries no covariance
  ExtrinsicsConfig extrinsics;
  PointsConfig points;
  MatcherConfig matcher;
  MeasConfig meas;
  AcceptConfig accept;
  ScanConfig scan;
  HistoryConfig history;
  InitConfig init;
  HealthConfig health;
};

// Throws std::invalid_argument naming the first bad key. Constructs an
// eskf::Eskf once so the filter's own checks run here too.
void validate(const LocalizerConfig& c);

}  // namespace brightdrive_localization

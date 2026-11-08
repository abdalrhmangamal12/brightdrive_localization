#include "brightdrive_localization/config_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

namespace {

class Reader {
 public:
  explicit Reader(const YAML::Node& params) : root_(params) {}

  template <typename T>
  T get(const std::string& dotted) const {
    YAML::Node n = YAML::Clone(root_);
    size_t start = 0;
    while (true) {
      const size_t dot = dotted.find('.', start);
      const std::string key = dotted.substr(start, dot - start);
      if (!n[key]) {
        throw std::runtime_error("localization.yaml: missing key " + dotted);
      }
      n = n[key];
      if (dot == std::string::npos) break;
      start = dot + 1;
    }
    return n.as<T>();
  }

  Eigen::Vector3d vec3(const std::string& dotted) const {
    const auto v = get<std::vector<double>>(dotted);
    if (v.size() != 3) {
      throw std::runtime_error("localization.yaml: " + dotted +
                               " must have 3 values");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

 private:
  YAML::Node root_;
};

}  // namespace

LocalizerConfig loadLocalizerConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("localization.yaml: cannot read " + yaml_path +
                             ": " + e.what());
  }
  if (!root["/**"] || !root["/**"]["ros__parameters"]) {
    throw std::runtime_error(
        "localization.yaml: no `/**: ros__parameters:` block in " + yaml_path);
  }
  const Reader r(root["/**"]["ros__parameters"]);
  LocalizerConfig c;

  c.extrinsics.T_base_imu = isometryFromXyzRpyDeg(r.vec3("T_base_imu_xyz"),
                                                  r.vec3("T_base_imu_rpy_deg"));
  c.extrinsics.T_base_lidar = isometryFromXyzRpyDeg(
      r.vec3("T_base_lidar_xyz"), r.vec3("T_base_lidar_rpy_deg"));
  c.extrinsics.imu_time_offset_s = r.get<double>("imu_time_offset_s");

  c.points.time_field = r.get<std::string>("points.time_field");
  c.points.time_scale = r.get<double>("points.time_scale");
  c.points.stamp_is = r.get<std::string>("points.stamp_is");

  const std::string noise_model = r.get<std::string>("eskf.noise_model");
  if (noise_model == "per_sample") {
    c.eskf.noise_model = eskf::NoiseModel::PerSample;
  } else if (noise_model == "density") {
    c.eskf.noise_model = eskf::NoiseModel::Density;
  } else {
    throw std::runtime_error(
        "localization.yaml: eskf.noise_model must be 'per_sample' or "
        "'density', got '" +
        noise_model + "'");
  }
  c.eskf.sigma_a = r.get<double>("eskf.sigma_a");
  c.eskf.sigma_g = r.get<double>("eskf.sigma_g");
  c.eskf.sigma_ba = r.get<double>("eskf.sigma_ba");
  c.eskf.sigma_bg = r.get<double>("eskf.sigma_bg");
  c.eskf.lever_arm = r.vec3("eskf.lever_arm");
  c.eskf.gate = r.get<double>("eskf.gate");
  c.eskf.gate_pose = r.get<double>("eskf.gate_pose");
  c.gnss_default_sigma_m = r.get<double>("gnss_default_sigma_m");

  c.matcher.map_voxel_m = r.get<double>("matcher.map_voxel_m");
  c.matcher.coarse_voxel_m = r.get<double>("matcher.coarse_voxel_m");
  c.matcher.source_leaf_m = r.get<double>("matcher.source_leaf_m");
  c.matcher.max_corr_dist_m = r.get<double>("matcher.max_corr_dist_m");
  c.matcher.coarse_max_corr_dist_m =
      r.get<double>("matcher.coarse_max_corr_dist_m");
  c.matcher.max_iterations = r.get<int>("matcher.max_iterations");
  c.matcher.num_threads = r.get<int>("matcher.num_threads");
  c.matcher.cov_neighbors = r.get<int>("matcher.cov_neighbors");
  c.matcher.min_range_m = r.get<double>("matcher.min_range_m");
  c.matcher.max_range_m = r.get<double>("matcher.max_range_m");

  c.meas.sigma_pos_m = r.get<double>("meas.sigma_pos_m");
  c.meas.sigma_rot_rad = r.get<double>("meas.sigma_rot_rad");

  c.accept.min_source_points = r.get<int>("accept.min_source_points");
  c.accept.min_inlier_ratio = r.get<double>("accept.min_inlier_ratio");
  c.accept.max_error_per_inlier = r.get<double>("accept.max_error_per_inlier");
  c.accept.max_jump_m = r.get<double>("accept.max_jump_m");
  c.accept.max_jump_rad = r.get<double>("accept.max_jump_rad");
  c.accept.max_age_s = r.get<double>("accept.max_age_s");

  c.scan.imu_wait_s = r.get<double>("scan.imu_wait_s");
  c.history.keep_s = r.get<double>("history.keep_s");

  c.init.yaw_steps = r.get<int>("init.yaw_steps");
  c.init.refine_top = r.get<int>("init.refine_top");
  c.init.margin = r.get<double>("init.margin");
  c.init.confirm_n = r.get<int>("init.confirm_n");
  c.init.timeout_s = r.get<double>("init.timeout_s");
  c.init.seed_gnss_max_age_s = r.get<double>("init.seed_gnss_max_age_s");
  c.init.still_window_s = r.get<double>("init.still_window_s");
  c.init.max_seed_gnss_sigma_m = r.get<double>("init.max_seed_gnss_sigma_m");
  c.init.vel_window_s = r.get<double>("init.vel_window_s");
  c.init.vel_min_fixes = r.get<int>("init.vel_min_fixes");
  c.init.vel_min_span_s = r.get<double>("init.vel_min_span_s");
  c.init.vel_sigma_floor_mps = r.get<double>("init.vel_sigma_floor_mps");
  c.init.still_max_gyro_radps = r.get<double>("init.still_max_gyro_radps");
  c.init.still_max_acc_std_mps2 = r.get<double>("init.still_max_acc_std_mps2");
  c.init.still_max_acc_mean_dev_mps2 =
      r.get<double>("init.still_max_acc_mean_dev_mps2");
  c.init.still_max_gnss_speed_mps =
      r.get<double>("init.still_max_gnss_speed_mps");
  c.init.still_gnss_speed_sigmas =
      r.get<double>("init.still_gnss_speed_sigmas");
  c.init.cog_min_speed_mps = r.get<double>("init.cog_min_speed_mps");
  c.init.cog_check_min_speed_mps =
      r.get<double>("init.cog_check_min_speed_mps");
  c.init.cog_max_diff_deg = r.get<double>("init.cog_max_diff_deg");
  c.init.carry_bg_max_radps = r.get<double>("init.carry_bg_max_radps");
  c.init.p0_sigma_pos_m = r.get<double>("init.p0_sigma_pos_m");
  c.init.p0_sigma_yaw_deg = r.get<double>("init.p0_sigma_yaw_deg");
  c.init.p0_sigma_rollpitch_deg = r.get<double>("init.p0_sigma_rollpitch_deg");
  c.init.p0_sigma_vel_mps = r.get<double>("init.p0_sigma_vel_mps");
  c.init.p0_sigma_vel_moving_mps =
      r.get<double>("init.p0_sigma_vel_moving_mps");
  c.init.p0_sigma_ba = r.get<double>("init.p0_sigma_ba");
  c.init.p0_sigma_bg = r.get<double>("init.p0_sigma_bg");

  c.health.scan_timeout_s = r.get<double>("health.scan_timeout_s");
  c.health.gnss_timeout_s = r.get<double>("health.gnss_timeout_s");
  c.health.dr_timeout_s = r.get<double>("health.dr_timeout_s");
  c.health.degraded_sigma_m = r.get<double>("health.degraded_sigma_m");
  c.health.fail_consecutive_gated = r.get<int>("health.fail_consecutive_gated");
  c.health.fail_timeout_s = r.get<double>("health.fail_timeout_s");
  c.health.fail_sigma_m = r.get<double>("health.fail_sigma_m");

  validate(c);
  return c;
}

}  // namespace brightdrive_localization

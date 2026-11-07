// localization_node: the ROS 2 wiring around Localizer. Parameters and their
// startup log, map.yaml, message conversion, publishers and TF. No estimation
// here. Today it loads and logs the configuration and the map description;
// the subscriptions, the matcher thread and the outputs follow.
#include <Eigen/Geometry>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "brightdrive_localization/geo.hpp"
#include "brightdrive_localization/map_io.hpp"
#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

namespace {

// Settings only the node needs; everything else is in LocalizerConfig.
struct NodeParams {
  std::string imu_topic;
  std::string gnss_topic;
  std::string points_topic;
  std::string initialpose_topic;
  std::string map_yaml;
  std::string odom_frame;
  std::string base_frame;
  bool publish_tf = true;
  int output_every_n_imu = 2;
  double status_rate_hz = 10.0;
  bool debug = false;
};

std::string toString(const Eigen::Vector3d& v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "[%.4f, %.4f, %.4f]", v.x(), v.y(), v.z());
  return buf;
}

std::string toString(const Eigen::Isometry3d& T) {
  const Eigen::Vector3d rpy =
      T.linear().eulerAngles(2, 1, 0).reverse() * 180.0 / M_PI;
  char buf[160];
  std::snprintf(buf, sizeof(buf), "xyz %s m, rpy [%.2f, %.2f, %.2f] deg",
                toString(T.translation()).c_str(), rpy.x(), rpy.y(), rpy.z());
  return buf;
}

}  // namespace

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode() : Node("localization_node") {
    loadParams();
    validate(cfg_);
    map_ = loadMapInfo(np_.map_yaml);
    projector_ = std::make_unique<geo::Projector>(
        geo::parseProjection(map_.projection), map_.origin_lat_deg,
        map_.origin_lon_deg, map_.origin_alt_m);
    logParams();
  }

 private:
  // ---- parameters ---------------------------------------------------------
  // Core keys have no defaults: a missing one stops the node naming the key.
  double requiredDouble(const std::string& name) {
    return declare_parameter(name, rclcpp::PARAMETER_DOUBLE).get<double>();
  }
  int requiredInt(const std::string& name) {
    return static_cast<int>(
        declare_parameter(name, rclcpp::PARAMETER_INTEGER).get<int64_t>());
  }
  std::string requiredString(const std::string& name) {
    return declare_parameter(name, rclcpp::PARAMETER_STRING).get<std::string>();
  }
  Eigen::Vector3d requiredVector3(const std::string& name) {
    const auto v = declare_parameter(name, rclcpp::PARAMETER_DOUBLE_ARRAY)
                       .get<std::vector<double>>();
    if (v.size() != 3) {
      throw std::invalid_argument("localization.yaml: " + name +
                                  " must have 3 values");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  void loadParams() {
    np_.imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    np_.gnss_topic =
        declare_parameter<std::string>("gnss_topic", "/gnss/fix_gated");
    np_.points_topic =
        declare_parameter<std::string>("points_topic", "/points_raw");
    np_.initialpose_topic =
        declare_parameter<std::string>("initialpose_topic", "/initialpose");
    np_.map_yaml =
        declare_parameter<std::string>("map_yaml", "/maps/town03_static.yaml");
    np_.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    np_.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    np_.publish_tf = declare_parameter<bool>("publish_tf", true);
    np_.output_every_n_imu = declare_parameter<int>("output_every_n_imu", 2);
    np_.status_rate_hz = declare_parameter<double>("status_rate_hz", 10.0);
    np_.debug = declare_parameter<bool>("debug", false);

    auto& c = cfg_;
    c.extrinsics.T_base_imu =
        isometryFromXyzRpyDeg(requiredVector3("T_base_imu_xyz"),
                              requiredVector3("T_base_imu_rpy_deg"));
    c.extrinsics.T_base_lidar =
        isometryFromXyzRpyDeg(requiredVector3("T_base_lidar_xyz"),
                              requiredVector3("T_base_lidar_rpy_deg"));
    c.extrinsics.imu_time_offset_s = requiredDouble("imu_time_offset_s");

    c.points.time_field = requiredString("points.time_field");
    c.points.time_scale = requiredDouble("points.time_scale");
    c.points.stamp_is = requiredString("points.stamp_is");

    const std::string noise_model = requiredString("eskf.noise_model");
    if (noise_model == "per_sample") {
      c.eskf.noise_model = eskf::NoiseModel::PerSample;
    } else if (noise_model == "density") {
      c.eskf.noise_model = eskf::NoiseModel::Density;
    } else {
      throw std::invalid_argument(
          "localization.yaml: eskf.noise_model must be 'per_sample' or "
          "'density', got '" +
          noise_model + "'");
    }
    c.eskf.sigma_a = requiredDouble("eskf.sigma_a");
    c.eskf.sigma_g = requiredDouble("eskf.sigma_g");
    c.eskf.sigma_ba = requiredDouble("eskf.sigma_ba");
    c.eskf.sigma_bg = requiredDouble("eskf.sigma_bg");
    c.eskf.lever_arm = requiredVector3("eskf.lever_arm");
    c.eskf.gate = requiredDouble("eskf.gate");
    c.gate_pose = requiredDouble("eskf.gate_pose");
    c.gnss_default_sigma_m = requiredDouble("gnss_default_sigma_m");

    c.matcher.map_voxel_m = requiredDouble("matcher.map_voxel_m");
    c.matcher.coarse_voxel_m = requiredDouble("matcher.coarse_voxel_m");
    c.matcher.source_leaf_m = requiredDouble("matcher.source_leaf_m");
    c.matcher.max_corr_dist_m = requiredDouble("matcher.max_corr_dist_m");
    c.matcher.coarse_max_corr_dist_m =
        requiredDouble("matcher.coarse_max_corr_dist_m");
    c.matcher.max_iterations = requiredInt("matcher.max_iterations");
    c.matcher.num_threads = requiredInt("matcher.num_threads");
    c.matcher.cov_neighbors = requiredInt("matcher.cov_neighbors");
    c.matcher.min_range_m = requiredDouble("matcher.min_range_m");
    c.matcher.max_range_m = requiredDouble("matcher.max_range_m");

    c.meas.sigma_pos_m = requiredDouble("meas.sigma_pos_m");
    c.meas.sigma_rot_rad = requiredDouble("meas.sigma_rot_rad");

    c.accept.min_inlier_ratio = requiredDouble("accept.min_inlier_ratio");
    c.accept.max_error_per_inlier =
        requiredDouble("accept.max_error_per_inlier");
    c.accept.max_jump_m = requiredDouble("accept.max_jump_m");
    c.accept.max_jump_rad = requiredDouble("accept.max_jump_rad");
    c.accept.max_age_s = requiredDouble("accept.max_age_s");

    c.history.keep_s = requiredDouble("history.keep_s");

    c.init.yaw_steps = requiredInt("init.yaw_steps");
    c.init.refine_top = requiredInt("init.refine_top");
    c.init.margin = requiredDouble("init.margin");
    c.init.confirm_n = requiredInt("init.confirm_n");
    c.init.timeout_s = requiredDouble("init.timeout_s");
    c.init.seed_gnss_max_age_s = requiredDouble("init.seed_gnss_max_age_s");
    c.init.still_window_s = requiredDouble("init.still_window_s");
    c.init.p0_sigma_pos_m = requiredDouble("init.p0_sigma_pos_m");
    c.init.p0_sigma_yaw_deg = requiredDouble("init.p0_sigma_yaw_deg");
    c.init.p0_sigma_rollpitch_deg =
        requiredDouble("init.p0_sigma_rollpitch_deg");
    c.init.p0_sigma_vel_mps = requiredDouble("init.p0_sigma_vel_mps");
    c.init.p0_sigma_vel_moving_mps =
        requiredDouble("init.p0_sigma_vel_moving_mps");
    c.init.p0_sigma_ba = requiredDouble("init.p0_sigma_ba");
    c.init.p0_sigma_bg = requiredDouble("init.p0_sigma_bg");

    c.health.scan_timeout_s = requiredDouble("health.scan_timeout_s");
    c.health.gnss_timeout_s = requiredDouble("health.gnss_timeout_s");
    c.health.dr_timeout_s = requiredDouble("health.dr_timeout_s");
    c.health.degraded_sigma_m = requiredDouble("health.degraded_sigma_m");
    c.health.fail_consecutive_gated =
        requiredInt("health.fail_consecutive_gated");
    c.health.fail_timeout_s = requiredDouble("health.fail_timeout_s");
    c.health.fail_sigma_m = requiredDouble("health.fail_sigma_m");
  }

  void logParams() {
    const auto& c = cfg_;
    RCLCPP_INFO(get_logger(),
                "topics: imu %s, gnss %s, points %s, initialpose %s",
                np_.imu_topic.c_str(), np_.gnss_topic.c_str(),
                np_.points_topic.c_str(), np_.initialpose_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "node: frames %s -> %s -> %s, publish_tf %d, output every %d "
                "IMU samples, status %.1f Hz, debug %d",
                map_.frame.c_str(), np_.odom_frame.c_str(),
                np_.base_frame.c_str(), np_.publish_tf, np_.output_every_n_imu,
                np_.status_rate_hz, np_.debug);
    RCLCPP_INFO(get_logger(),
                "map: %s -> %s, frame %s, resolution %.2f m, projection %s "
                "origin lat %.8f lon %.8f alt %.2f",
                np_.map_yaml.c_str(), map_.pcd_path.c_str(), map_.frame.c_str(),
                map_.resolution_m, map_.projection.c_str(), map_.origin_lat_deg,
                map_.origin_lon_deg, map_.origin_alt_m);
    RCLCPP_INFO(get_logger(), "T_base_imu: %s",
                toString(c.extrinsics.T_base_imu).c_str());
    RCLCPP_INFO(get_logger(), "T_base_lidar: %s",
                toString(c.extrinsics.T_base_lidar).c_str());
    RCLCPP_INFO(get_logger(),
                "imu_time_offset %.4f s; points: field '%s' x %.1e s, stamp "
                "is the sweep %s",
                c.extrinsics.imu_time_offset_s, c.points.time_field.c_str(),
                c.points.time_scale, c.points.stamp_is.c_str());
    RCLCPP_INFO(get_logger(),
                "eskf: %s sigma_a %.4f sigma_g %.5f sigma_ba %.1e sigma_bg "
                "%.1e, lever_arm %s m, gate %.2f, gate_pose %.2f, "
                "gnss_default_sigma %.2f m",
                c.eskf.noise_model == eskf::NoiseModel::PerSample ? "per_sample"
                                                                  : "density",
                c.eskf.sigma_a, c.eskf.sigma_g, c.eskf.sigma_ba,
                c.eskf.sigma_bg, toString(c.eskf.lever_arm).c_str(),
                c.eskf.gate, c.gate_pose, c.gnss_default_sigma_m);
    const auto& m = c.matcher;
    RCLCPP_INFO(get_logger(),
                "matcher: voxels %.2f / %.2f m, source leaf %.2f m, max corr "
                "%.2f / %.2f m, %d iterations, %d threads, %d cov neighbours, "
                "range %.1f..%.1f m",
                m.map_voxel_m, m.coarse_voxel_m, m.source_leaf_m,
                m.max_corr_dist_m, m.coarse_max_corr_dist_m, m.max_iterations,
                m.num_threads, m.cov_neighbors, m.min_range_m, m.max_range_m);
    RCLCPP_INFO(get_logger(),
                "meas: sigma_pos %.3f m, sigma_rot %.4f rad; accept: inlier "
                "ratio >= %.2f, error/inlier <= %.3g, jump <= %.2f m / %.3f "
                "rad, age <= %.2f s",
                c.meas.sigma_pos_m, c.meas.sigma_rot_rad,
                c.accept.min_inlier_ratio, c.accept.max_error_per_inlier,
                c.accept.max_jump_m, c.accept.max_jump_rad, c.accept.max_age_s);
    const auto& i = c.init;
    RCLCPP_INFO(get_logger(),
                "history keep %.1f s; init: %d yaws, refine %d, margin %.2f, "
                "confirm %d, timeout %.1f s, gnss seed age <= %.1f s, still "
                "window %.1f s",
                c.history.keep_s, i.yaw_steps, i.refine_top, i.margin,
                i.confirm_n, i.timeout_s, i.seed_gnss_max_age_s,
                i.still_window_s);
    RCLCPP_INFO(get_logger(),
                "P0: pos %.2f m, yaw %.1f deg, roll/pitch %.1f deg, vel %.2f "
                "(moving %.1f) m/s, b_a %.3f, b_g %.4f",
                i.p0_sigma_pos_m, i.p0_sigma_yaw_deg, i.p0_sigma_rollpitch_deg,
                i.p0_sigma_vel_mps, i.p0_sigma_vel_moving_mps, i.p0_sigma_ba,
                i.p0_sigma_bg);
    const auto& h = c.health;
    RCLCPP_INFO(get_logger(),
                "health: scan timeout %.1f s, gnss timeout %.1f s, dead "
                "reckoning after %.1f s, degraded sigma %.2f m, fail after %d "
                "gated / %.1f s / sigma %.1f m",
                h.scan_timeout_s, h.gnss_timeout_s, h.dr_timeout_s,
                h.degraded_sigma_m, h.fail_consecutive_gated, h.fail_timeout_s,
                h.fail_sigma_m);
  }

  NodeParams np_;
  LocalizerConfig cfg_;
  MapInfo map_;
  std::unique_ptr<geo::Projector> projector_;
};

}  // namespace brightdrive_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = EXIT_SUCCESS;
  try {
    auto node = std::make_shared<brightdrive_localization::LocalizationNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // A missing key, a bad value or an unreadable map.yaml ends here.
    RCLCPP_FATAL(rclcpp::get_logger("localization_node"), "%s", e.what());
    code = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return code;
}

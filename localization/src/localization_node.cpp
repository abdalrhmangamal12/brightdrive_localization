// localization_node: the ROS 2 wiring around Localizer. Parameters and their
// startup log, the map, message conversion, the one-deep sweep slot and the
// matcher thread, publishers and TF. No estimation here.
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "brightdrive_localization/geo.hpp"
#include "brightdrive_localization/localizer.hpp"
#include "brightdrive_localization/map_io.hpp"
#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/ros_conversions.hpp"
#include "brightdrive_localization/scan_matcher.hpp"
#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

namespace {

using Clock = std::chrono::steady_clock;

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

// A path pose every this many seconds of data time.
constexpr double kPathPeriodS = 0.5;

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

builtin_interfaces::msg::Time toStamp(double t) {
  builtin_interfaces::msg::Time s;
  const double sec = std::floor(t);
  s.sec = static_cast<int32_t>(sec);
  s.nanosec =
      static_cast<uint32_t>(std::llround((t - sec) * 1e9) % 1000000000LL);
  return s;
}

geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond q(T.linear());
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  return p;
}

geometry_msgs::msg::Transform toTransform(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Transform tf;
  tf.translation.x = T.translation().x();
  tf.translation.y = T.translation().y();
  tf.translation.z = T.translation().z();
  const Eigen::Quaterniond q(T.linear());
  tf.rotation.w = q.w();
  tf.rotation.x = q.x();
  tf.rotation.y = q.y();
  tf.rotation.z = q.z();
  return tf;
}

void copyCov(const Matrix6d& c, std::array<double, 36>& out) {
  for (int r = 0; r < 6; ++r) {
    for (int k = 0; k < 6; ++k) out[r * 6 + k] = c(r, k);
  }
}

sensor_msgs::msg::PointCloud2 toCloud(const small_gicp::PointCloud& pc,
                                      const Eigen::Isometry3d& T,
                                      const std::string& frame, double t) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.header.stamp = toStamp(t);
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pc.size());
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (size_t i = 0; i < pc.size(); ++i, ++x, ++y, ++z) {
    const Eigen::Vector3d p = T * Eigen::Vector3d(pc.point(i).head<3>());
    *x = static_cast<float>(p.x());
    *y = static_cast<float>(p.y());
    *z = static_cast<float>(p.z());
  }
  return msg;
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

    // The map once, before any subscription: the first sweep needs it.
    const auto t0 = Clock::now();
    const auto points = loadMapPoints(map_.pcd_path);
    matcher_ = std::make_unique<ScanMatcher>(cfg_.matcher);
    matcher_->setMap(points);
    const auto& ms = matcher_->mapStats();
    RCLCPP_INFO(get_logger(),
                "map ready in %.2f s: %zu points, covariances %.2f s, voxel "
                "maps %.2f s (%zu at %.1f m, %zu at %.1f m)",
                std::chrono::duration<double>(Clock::now() - t0).count(),
                ms.points, ms.covariance_s, ms.voxelmap_s, ms.voxels_fine,
                cfg_.matcher.map_voxel_m, ms.voxels_coarse,
                cfg_.matcher.coarse_voxel_m);
    loc_ = std::make_unique<Localizer>(cfg_);

    const rclcpp::QoS out_qos(10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose", out_qos);
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/localization/velocity", out_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/localization/odometry", out_qos);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/localization/status", out_qos);
    path_pub_ =
        create_publisher<nav_msgs::msg::Path>("/localization/path", out_qos);
    if (np_.debug) {
      match_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/localization/debug/match", rclcpp::QoS(50));
      aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/localization/debug/aligned_scan", rclcpp::QoS(2));
      deskewed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/localization/debug/deskewed_cloud", rclcpp::QoS(2));
    }
    if (np_.publish_tf) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        np_.imu_topic, rclcpp::SensorDataQoS().keep_last(200),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imuCallback(*m); });
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        np_.gnss_topic, rclcpp::SensorDataQoS().keep_last(10),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
          gnssCallback(*m);
        });
    // Newest sweep wins: best effort, depth 1, and a one-deep slot.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        np_.points_topic, rclcpp::QoS(1).best_effort(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          cloudCallback(m);
        });
    initialpose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            np_.initialpose_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
                       m) { initialPoseCallback(*m); });

    matcher_thread_ = std::thread([this]() { matcherLoop(); });
  }

  ~LocalizationNode() override {
    {
      std::lock_guard<std::mutex> lk(slot_mtx_);
      stop_ = true;
    }
    slot_cv_.notify_all();
    if (matcher_thread_.joinable()) matcher_thread_.join();
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
    c.eskf.gate_pose = requiredDouble("eskf.gate_pose");
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

    c.accept.min_source_points = requiredInt("accept.min_source_points");
    c.accept.min_inlier_ratio = requiredDouble("accept.min_inlier_ratio");
    c.accept.max_error_per_inlier =
        requiredDouble("accept.max_error_per_inlier");
    c.accept.max_jump_m = requiredDouble("accept.max_jump_m");
    c.accept.max_jump_rad = requiredDouble("accept.max_jump_rad");
    c.accept.max_age_s = requiredDouble("accept.max_age_s");

    c.scan.imu_wait_s = requiredDouble("scan.imu_wait_s");
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
                c.eskf.gate, c.eskf.gate_pose, c.gnss_default_sigma_m);
    const auto& m = c.matcher;
    RCLCPP_INFO(get_logger(),
                "matcher: voxels %.2f / %.2f m, source leaf %.2f m, max corr "
                "%.2f / %.2f m, %d iterations, %d threads, %d cov neighbours, "
                "range %.1f..%.1f m",
                m.map_voxel_m, m.coarse_voxel_m, m.source_leaf_m,
                m.max_corr_dist_m, m.coarse_max_corr_dist_m, m.max_iterations,
                m.num_threads, m.cov_neighbors, m.min_range_m, m.max_range_m);
    RCLCPP_INFO(get_logger(),
                "meas: sigma_pos %.3f m, sigma_rot %.4f rad; accept: source "
                "points >= %d, inlier ratio >= %.2f, error/inlier <= %.3g, "
                "jump <= %.2f m / %.3f rad, age <= %.2f s; imu wait %.0f ms",
                c.meas.sigma_pos_m, c.meas.sigma_rot_rad,
                c.accept.min_source_points, c.accept.min_inlier_ratio,
                c.accept.max_error_per_inlier, c.accept.max_jump_m,
                c.accept.max_jump_rad, c.accept.max_age_s,
                c.scan.imu_wait_s * 1e3);
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

  // ---- inputs -------------------------------------------------------------
  void imuCallback(const sensor_msgs::msg::Imu& m) {
    const auto entry = Clock::now();
    eskf::ImuSample s;
    s.t = toSeconds(m.header.stamp);
    s.acc = Eigen::Vector3d(m.linear_acceleration.x, m.linear_acceleration.y,
                            m.linear_acceleration.z);
    s.gyro = Eigen::Vector3d(m.angular_velocity.x, m.angular_velocity.y,
                             m.angular_velocity.z);
    if (!s.acc.allFinite() || !s.gyro.allFinite()) return;
    loc_->onImu(s);
    ++imu_count_;
    if (imu_count_ % static_cast<uint64_t>(np_.output_every_n_imu) == 0) {
      publishOutputs(entry);
    }
    maybePublishStatus(s.t + cfg_.extrinsics.imu_time_offset_s);
  }

  void gnssCallback(const sensor_msgs::msg::NavSatFix& m) {
    if (m.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !std::isfinite(m.latitude) || !std::isfinite(m.longitude) ||
        !std::isfinite(m.altitude)) {
      ++gnss_no_fix_;
      return;
    }
    eskf::GnssMeas z;
    z.t = toSeconds(m.header.stamp);
    // The antenna position in map metres, raw: correctGnss applies the lever
    // arm itself.
    z.z = projector_->llaToMap(m.latitude, m.longitude, m.altitude);
    const double d = cfg_.gnss_default_sigma_m * cfg_.gnss_default_sigma_m;
    z.V = Eigen::Matrix3d::Identity() * d;
    if (m.position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const Eigen::Vector3d diag(m.position_covariance[0],
                                 m.position_covariance[4],
                                 m.position_covariance[8]);
      if ((diag.array() > 0.0).all() && diag.allFinite()) {
        z.V = diag.asDiagonal();
      }
    }
    loc_->onGnss(z);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m) {
    {
      std::lock_guard<std::mutex> lk(slot_mtx_);
      if (slot_) ++sweeps_dropped_busy_;
      slot_ = m;
      ++sweeps_received_;
    }
    slot_cv_.notify_one();
  }

  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped& m) {
    Matrix6d cov;
    for (int r = 0; r < 6; ++r) {
      for (int k = 0; k < 6; ++k) cov(r, k) = m.pose.covariance[r * 6 + k];
    }
    loc_->onInitialPose(toEigen(m.pose.pose), cov, toSeconds(m.header.stamp));
    RCLCPP_INFO(get_logger(), "/initialpose received at t %.2f s",
                toSeconds(m.header.stamp));
  }

  // ---- matcher thread -----------------------------------------------------
  void matcherLoop() {
    while (true) {
      sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
      {
        std::unique_lock<std::mutex> lk(slot_mtx_);
        slot_cv_.wait(lk, [this] { return stop_ || slot_ != nullptr; });
        if (stop_) return;
        msg = slot_;
        slot_.reset();
      }
      Scan scan;
      try {
        scan = scanFromMsg(*msg, cfg_.points);
      } catch (const std::exception& e) {
        ++scan_errors_;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        continue;
      }
      const SweepReport rep = loc_->processScan(std::move(scan), *matcher_);
      if (rep.kind == SweepReport::Kind::kStarted) {
        RCLCPP_INFO(get_logger(), "started on the sweep at t %.2f s",
                    rep.t_ref);
      }
      publishDebug(rep, msg->header.frame_id);
    }
  }

  // ---- outputs ------------------------------------------------------------
  void publishOutputs(Clock::time_point entry) {
    const Output o = loc_->output();
    if (!o.valid) return;
    const auto stamp = toStamp(o.t);
    const std::string& map_frame = map_.frame;

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame;
    pose.pose.pose = toPose(o.T_map_base);
    copyCov(o.pose_cov, pose.pose.covariance);
    pose_pub_->publish(pose);

    geometry_msgs::msg::TwistWithCovarianceStamped vel;
    vel.header.stamp = stamp;
    vel.header.frame_id = np_.base_frame;
    vel.twist.twist.linear.x = o.v_base.x();
    vel.twist.twist.linear.y = o.v_base.y();
    vel.twist.twist.linear.z = o.v_base.z();
    vel.twist.twist.angular.x = o.w_base.x();
    vel.twist.twist.angular.y = o.w_base.y();
    vel.twist.twist.angular.z = o.w_base.z();
    copyCov(o.twist_cov, vel.twist.covariance);
    vel_pub_->publish(vel);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = np_.base_frame;
    odom.pose = pose.pose;
    odom.twist = vel.twist;
    odom_pub_->publish(odom);

    if (tf_) {
      // One message, one stamp: map -> odom and odom -> base_link.
      std::vector<geometry_msgs::msg::TransformStamped> tfs(2);
      tfs[0].header.stamp = stamp;
      tfs[0].header.frame_id = map_frame;
      tfs[0].child_frame_id = np_.odom_frame;
      tfs[0].transform = toTransform(o.T_map_base * o.T_odom_base.inverse());
      tfs[1].header.stamp = stamp;
      tfs[1].header.frame_id = np_.odom_frame;
      tfs[1].child_frame_id = np_.base_frame;
      tfs[1].transform = toTransform(o.T_odom_base);
      tf_->sendTransform(tfs);
    }

    if (o.t - last_path_t_ >= kPathPeriodS) {
      last_path_t_ = o.t;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = pose.header;
      ps.pose = pose.pose.pose;
      path_.header = pose.header;
      path_.poses.push_back(ps);
      path_pub_->publish(path_);
    }
    // IMU callback entry to the last output published, wall time.
    last_output_latency_ms_ =
        std::chrono::duration<double>(Clock::now() - entry).count() * 1e3;
    max_output_latency_ms_ =
        std::max(max_output_latency_ms_, last_output_latency_ms_);
  }

  void maybePublishStatus(double t) {
    const Lifecycle lc = loc_->lifecycle();
    const bool changed = lc != last_status_lifecycle_;
    if (!changed && t - last_status_t_ < 1.0 / np_.status_rate_hz) return;
    last_status_t_ = t;
    last_status_lifecycle_ = lc;

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "localization";
    st.hardware_id = "brightdrive_localization";
    std::string health = "-";
    for (const auto& [k, v] : loc_->statusValues()) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
      if (k == "health") health = v;
    }
    {
      std::lock_guard<std::mutex> lk(slot_mtx_);
      addValue(st, "sweeps_received", std::to_string(sweeps_received_));
      addValue(st, "sweeps_dropped_busy", std::to_string(sweeps_dropped_busy_));
    }
    addValue(st, "scan_convert_errors", std::to_string(scan_errors_.load()));
    addValue(st, "gnss_no_fix", std::to_string(gnss_no_fix_));
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", last_output_latency_ms_);
    addValue(st, "output_latency_ms", buf);
    std::snprintf(buf, sizeof(buf), "%.3f", max_output_latency_ms_);
    addValue(st, "output_latency_max_ms", buf);

    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (lc != Lifecycle::kTracking) {
      st.level = DS::STALE;
    } else if (health == "NOMINAL") {
      st.level = DS::OK;
    } else if (health == "FAILED") {
      st.level = DS::ERROR;
    } else {
      st.level = DS::WARN;
    }
    st.message = std::string(toString(lc)) +
                 (lc == Lifecycle::kTracking ? " " + health : "");

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = toStamp(t);
    arr.status.push_back(st);
    status_pub_->publish(arr);
  }

  static void addValue(diagnostic_msgs::msg::DiagnosticStatus& st,
                       const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    st.values.push_back(kv);
  }

  void publishDebug(const SweepReport& rep, const std::string& lidar_frame) {
    if (!np_.debug) return;
    if (rep.kind == SweepReport::Kind::kMatched ||
        rep.kind == SweepReport::Kind::kStarted) {
      const MatchResult& m = rep.match;
      std_msgs::msg::Float64MultiArray a;
      a.data = {rep.t_ref,
                m.time_ms,
                static_cast<double>(m.num_inliers),
                static_cast<double>(m.num_source),
                m.inlierRatio(),
                m.errorPerInlier(),
                rep.nis,
                rep.kind == SweepReport::Kind::kStarted
                    ? 2.0
                    : (rep.reject == Reject::kNone ? 1.0 : 0.0),
                static_cast<double>(rep.reject),
                rep.age_s * 1e3};
      match_pub_->publish(a);
    }
    if (!rep.source) return;
    if (aligned_pub_->get_subscription_count() > 0 &&
        rep.match.num_inliers > 0) {
      aligned_pub_->publish(
          toCloud(*rep.source, rep.match.T_map_lidar, map_.frame, rep.t_ref));
    }
    if (deskewed_pub_->get_subscription_count() > 0) {
      deskewed_pub_->publish(toCloud(*rep.source, Eigen::Isometry3d::Identity(),
                                     lidar_frame, rep.t_ref));
    }
  }

  NodeParams np_;
  LocalizerConfig cfg_;
  MapInfo map_;
  std::unique_ptr<geo::Projector> projector_;
  std::unique_ptr<ScanMatcher> matcher_;
  std::unique_ptr<Localizer> loc_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr match_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_sub_;

  // The one-deep sweep slot between the executor and the matcher thread.
  std::mutex slot_mtx_;
  std::condition_variable slot_cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr slot_;
  bool stop_ = false;
  uint64_t sweeps_received_ = 0;
  uint64_t sweeps_dropped_busy_ = 0;
  std::thread matcher_thread_;
  std::atomic<uint64_t> scan_errors_{0};

  // Executor-only state.
  uint64_t imu_count_ = 0;
  uint64_t gnss_no_fix_ = 0;
  double last_status_t_ = -1e9;
  Lifecycle last_status_lifecycle_ = Lifecycle::kWaitData;
  double last_path_t_ = -1e9;
  nav_msgs::msg::Path path_;
  double last_output_latency_ms_ = 0.0;
  double max_output_latency_ms_ = 0.0;
};

}  // namespace brightdrive_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = EXIT_SUCCESS;
  try {
    auto node = std::make_shared<brightdrive_localization::LocalizationNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // A missing key, a bad value or an unreadable map ends here.
    RCLCPP_FATAL(rclcpp::get_logger("localization_node"), "%s", e.what());
    code = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return code;
}

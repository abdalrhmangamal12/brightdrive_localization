// backend_node: the ROS 2 wiring around Backend. Parameters and their startup
// log, stamp pairing of odometry and body clouds, message conversion, TF
// map_frame -> origin_frame, path and map publishers, the loop thread, the map
// timer, the save service and stop(). No estimation here.
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "brightdrive_mapping/backend.hpp"
#include "brightdrive_mapping/geo.hpp"
#include "brightdrive_mapping/params.hpp"
#include "brightdrive_mapping/types.hpp"

namespace brightdrive_mapping {

namespace {

// The front-end stamps the odometry and the body cloud of one scan
// identically; 1 ms absorbs the float conversion.
constexpr int64_t kPairToleranceNs = 1000000;
// A cloud whose odometry has not arrived 1 s of odometry later never gets it.
constexpr int64_t kPairTimeoutNs = 1000000000;
// Odometry samples kept for pairing (Backend keeps its own full history).
constexpr size_t kOdomBufferSize = 400;

// Settings only the node needs; everything else is in BackendConfig.
struct NodeParams {
  std::string odom_topic;
  std::string cloud_topic;
  std::string gnss_topic;
  std::string path_topic;
  std::string map_topic;
  std::string save_service;
  bool publish_tf = true;
  double loop_frequency = 1.0;  // [Hz]
  double map_period = 5.0;      // [s]
  std::string save_dir;
  MapSource source;  // bag and commits for map.yaml, set by the launch file
};

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose& m) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(m.orientation.w, m.orientation.x,
                                  m.orientation.y, m.orientation.z)
                   .normalized()
                   .toRotationMatrix();
  T.translation() = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
  return T;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Pose m;
  const Eigen::Quaterniond q(T.linear());
  m.position.x = T.translation().x();
  m.position.y = T.translation().y();
  m.position.z = T.translation().z();
  m.orientation.w = q.w();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  return m;
}

geometry_msgs::msg::Transform toTransformMsg(const Eigen::Isometry3d& T) {
  const geometry_msgs::msg::Pose p = toPoseMsg(T);
  geometry_msgs::msg::Transform m;
  m.translation.x = p.position.x;
  m.translation.y = p.position.y;
  m.translation.z = p.position.z;
  m.rotation = p.orientation;
  return m;
}

std::string toString(const Eigen::VectorXd& v) {
  std::string s = "[";
  for (int i = 0; i < v.size(); ++i) {
    s += (i ? ", " : "") + std::to_string(v[i]);
  }
  return s + "]";
}

std::string describe(const Eigen::Isometry3d& T) {
  const Eigen::Vector3d t = T.translation();
  const double yaw_deg = std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI;
  char buf[128];
  std::snprintf(buf, sizeof(buf), "xyz (%.3f, %.3f, %.3f) m, yaw %.3f deg",
                t.x(), t.y(), t.z(), yaw_deg);
  return buf;
}

}  // namespace

class BackendNode : public rclcpp::Node {
 public:
  BackendNode() : Node("mapping_backend") {
    loadParams();
    backend_ = std::make_unique<Backend>(cfg_);  // throws on a bad config
    // Built once; every fix goes through it (gnssCallback).
    projector_ = std::make_unique<geo::Projector>(
        geo::parseProjection(cfg_.geo_projection), cfg_.geo_origin_lat_deg,
        cfg_.geo_origin_lon_deg, cfg_.geo_origin_alt_m);

    // The three inputs share one mutually exclusive group: they are handled
    // in arrival order and never concurrently. The save service and the map
    // timer use the node's default group.
    ingest_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ingest;
    ingest.callback_group = ingest_group_;
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        np_.odom_topic, rclcpp::QoS(200),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odomCallback(msg);
        },
        ingest);
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        np_.cloud_topic, rclcpp::QoS(50),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(msg);
        },
        ingest);
    // The bridge publishes GNSS best-effort; a reliable subscriber would not
    // match it.
    sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        np_.gnss_topic, rclcpp::SensorDataQoS().keep_last(50),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
          gnssCallback(msg);
        },
        ingest);

    pub_path_ = create_publisher<nav_msgs::msg::Path>(np_.path_topic, 1);
    pub_map_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(np_.map_topic, 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    srv_save_ = create_service<std_srvs::srv::Trigger>(
        np_.save_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          saveMapCallback(req, res);
        });
    map_timer_ =
        create_wall_timer(std::chrono::duration<double>(np_.map_period),
                          [this]() { mapTimer(); });
    status_timer_ =
        create_wall_timer(std::chrono::seconds(5), [this]() { printStatus(); });

    running_ = true;
    loop_thread_ = std::thread([this]() { loopThread(); });
    RCLCPP_INFO(get_logger(), "mapping backend ready");
  }

  // Joins the loop thread and saves the map, unless nothing arrived since the
  // last save (the replay launch saves through the service first). Called
  // from main() after spin() returns, never from a destructor.
  void stop() {
    if (!running_.exchange(false)) return;
    stop_cv_.notify_all();
    if (loop_thread_.joinable()) loop_thread_.join();
    if (backend_->numKeyframes() == saved_keyframes_ &&
        backend_->numLoops() == saved_loops_) {
      RCLCPP_INFO(
          get_logger(),
          "save_map (shutdown): nothing new since the last save, skipped");
      return;
    }
    saveMap("shutdown");
  }

 private:
  // ---- parameters ---------------------------------------------------------
  // Backend values have no defaults: a key missing from backend.yaml fails
  // here with its name. Node-only values default to the design names.
  double requiredDouble(const std::string& name) {
    return declare_parameter(name, rclcpp::PARAMETER_DOUBLE).get<double>();
  }
  int requiredInt(const std::string& name) {
    return static_cast<int>(
        declare_parameter(name, rclcpp::PARAMETER_INTEGER).get<int64_t>());
  }
  bool requiredBool(const std::string& name) {
    return declare_parameter(name, rclcpp::PARAMETER_BOOL).get<bool>();
  }
  std::string requiredString(const std::string& name) {
    return declare_parameter(name, rclcpp::PARAMETER_STRING).get<std::string>();
  }
  Eigen::VectorXd requiredVector(const std::string& name, int size) {
    const std::vector<double> v =
        declare_parameter(name, rclcpp::PARAMETER_DOUBLE_ARRAY)
            .get<std::vector<double>>();
    if (static_cast<int>(v.size()) != size) {
      throw std::invalid_argument(name + " needs " + std::to_string(size) +
                                  " values, got " + std::to_string(v.size()));
    }
    return Eigen::Map<const Eigen::VectorXd>(v.data(), size);
  }

  void loadParams() {
    np_.odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    np_.cloud_topic =
        declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    np_.gnss_topic = declare_parameter<std::string>("gnss_topic", "/gnss/fix");
    np_.path_topic =
        declare_parameter<std::string>("path_topic", "/mapping/path");
    np_.map_topic =
        declare_parameter<std::string>("map_topic", "/mapping/global_map");
    np_.save_service =
        declare_parameter<std::string>("save_service", "/mapping/save_map");
    np_.publish_tf = declare_parameter<bool>("publish_tf", true);
    np_.loop_frequency = declare_parameter<double>("loop_frequency", 1.0);
    np_.map_period = declare_parameter<double>("map_period", 5.0);
    np_.save_dir = declare_parameter<std::string>("save_dir", "/maps");
    np_.source.bag = declare_parameter<std::string>("source_bag", "");
    np_.source.frontend_commit =
        declare_parameter<std::string>("frontend_commit", "");
    np_.source.backend_commit =
        declare_parameter<std::string>("backend_commit", "");

    cfg_.lever_arm = requiredVector("gnss_lever_arm", 3);
    cfg_.kf_dist = requiredDouble("kf_dist");
    cfg_.kf_angle = requiredDouble("kf_angle");
    cfg_.kf_leaf = requiredDouble("kf_leaf");
    cfg_.min_range = requiredDouble("min_range");
    cfg_.max_range = requiredDouble("max_range");
    cfg_.prior_sigma = requiredVector("prior_sigma", 6);
    cfg_.odom_sigma = requiredVector("odom_sigma", 6);
    cfg_.origin_sigma = requiredVector("origin_sigma", 6);
    cfg_.gnss_enable = requiredBool("gnss_enable");
    cfg_.align_distance = requiredDouble("align_distance");
    cfg_.max_time_shift = requiredDouble("max_time_shift");
    cfg_.gnss_huber_k = requiredDouble("gnss_huber_k");
    cfg_.gnss_default_sigma = requiredDouble("gnss_default_sigma");
    cfg_.geo_projection = requiredString("geo_projection");
    cfg_.geo_origin_lat_deg = requiredDouble("geo_origin_lat_deg");
    cfg_.geo_origin_lon_deg = requiredDouble("geo_origin_lon_deg");
    cfg_.geo_origin_alt_m = requiredDouble("geo_origin_alt_m");
    cfg_.loop_enable = requiredBool("loop_enable");
    cfg_.loop_radius = requiredDouble("loop_radius");
    cfg_.loop_time_diff = requiredDouble("loop_time_diff");
    cfg_.loop_search_num = requiredInt("loop_search_num");
    cfg_.loop_icp_leaf = requiredDouble("loop_icp_leaf");
    cfg_.loop_fitness = requiredDouble("loop_fitness");
    cfg_.loop_rot_scale = requiredDouble("loop_rot_scale");
    cfg_.loop_trans_scale = requiredDouble("loop_trans_scale");
    cfg_.map_frame = requiredString("map_frame");
    cfg_.origin_frame = requiredString("origin_frame");
    cfg_.map_name = requiredString("map_name");
    cfg_.save_resolution = requiredDouble("save_resolution");
    // The one backend key with a default: a per-run switch the launch sets.
    cfg_.save_keyframes = declare_parameter<bool>("save_keyframes", false);
    logParams();
  }

  // Every value the run uses, so a log alone explains a map.
  void logParams() {
    const auto& c = cfg_;
    RCLCPP_INFO(
        get_logger(),
        "topics: odom %s, cloud %s, gnss %s -> path %s, map %s, service %s",
        np_.odom_topic.c_str(), np_.cloud_topic.c_str(), np_.gnss_topic.c_str(),
        np_.path_topic.c_str(), np_.map_topic.c_str(),
        np_.save_service.c_str());
    RCLCPP_INFO(get_logger(),
                "node: publish_tf %d, loop_frequency %.2f Hz, map_period %.1f "
                "s, save_dir %s",
                np_.publish_tf, np_.loop_frequency, np_.map_period,
                np_.save_dir.c_str());
    RCLCPP_INFO(get_logger(),
                "source: bag '%s', frontend commit '%s', backend commit '%s'",
                np_.source.bag.c_str(), np_.source.frontend_commit.c_str(),
                np_.source.backend_commit.c_str());
    RCLCPP_INFO(get_logger(),
                "frames: map_frame %s, origin_frame %s; map_name %s, "
                "save_resolution %.2f m, save_keyframes %d",
                c.map_frame.c_str(), c.origin_frame.c_str(), c.map_name.c_str(),
                c.save_resolution, c.save_keyframes);
    RCLCPP_INFO(get_logger(), "gnss_lever_arm %s m (antenna in the IMU frame)",
                toString(c.lever_arm).c_str());
    RCLCPP_INFO(get_logger(),
                "keyframes: kf_dist %.2f m, kf_angle %.3f rad, kf_leaf %.2f m, "
                "range %.1f..%.1f m",
                c.kf_dist, c.kf_angle, c.kf_leaf, c.min_range, c.max_range);
    RCLCPP_INFO(get_logger(), "prior_sigma %s",
                toString(c.prior_sigma).c_str());
    RCLCPP_INFO(get_logger(), "odom_sigma %s", toString(c.odom_sigma).c_str());
    RCLCPP_INFO(get_logger(), "origin_sigma %s",
                toString(c.origin_sigma).c_str());
    RCLCPP_INFO(get_logger(),
                "gnss: enable %d, align_distance %.1f m, max_time_shift %.2f "
                "s, huber_k %.3f, default_sigma %.2f m",
                c.gnss_enable, c.align_distance, c.max_time_shift,
                c.gnss_huber_k, c.gnss_default_sigma);
    RCLCPP_INFO(get_logger(),
                "geo: %s, origin lat %.8f deg, lon %.8f deg, alt %.2f m "
                "(map.yaml map.projection)",
                c.geo_projection.c_str(), c.geo_origin_lat_deg,
                c.geo_origin_lon_deg, c.geo_origin_alt_m);
    RCLCPP_INFO(
        get_logger(),
        "loop: enable %d, radius %.1f m, time_diff %.0f s, search_num %d, "
        "icp_leaf %.2f m, fitness < %.3f, rot_scale %.4f, trans_scale %.3f",
        c.loop_enable, c.loop_radius, c.loop_time_diff, c.loop_search_num,
        c.loop_icp_leaf, c.loop_fitness, c.loop_rot_scale, c.loop_trans_scale);
  }

  // ---- inputs -------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    const Eigen::Isometry3d T_odom = toEigen(msg->pose.pose);
    odom_buf_[stamp] = T_odom;
    while (odom_buf_.size() > kOdomBufferSize)
      odom_buf_.erase(odom_buf_.begin());
    backend_->addOdometry(stamp * 1e-9, T_odom);

    // map_frame -> origin_frame with this message's stamp; empty (nothing
    // published) until GNSS alignment.
    if (np_.publish_tf) {
      const std::optional<Eigen::Isometry3d> T = backend_->mapToOdomTransform();
      if (T) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = cfg_.map_frame;
        tf.child_frame_id = cfg_.origin_frame;
        tf.transform = toTransformMsg(*T);
        tf_broadcaster_->sendTransform(tf);
      }
    }
    processPending();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    ++n_scans_;
    pending_.push_back(msg);
    processPending();
  }

  // Pairs every waiting cloud with the odometry sample of the same stamp.
  // A cloud is dropped once odometry 1 s newer exists without a match.
  void processPending() {
    while (!pending_.empty()) {
      const auto& cloud = pending_.front();
      const int64_t stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
      const auto it = odom_buf_.lower_bound(stamp - kPairToleranceNs);
      if (it != odom_buf_.end() &&
          std::llabs(it->first - stamp) <= kPairToleranceNs) {
        handleScan(cloud, it->second, stamp * 1e-9);
        pending_.pop_front();
        continue;
      }
      if (!odom_buf_.empty() &&
          odom_buf_.rbegin()->first > stamp + kPairTimeoutNs) {
        pending_.pop_front();
        ++n_unsynced_;
        continue;
      }
      break;  // its odometry may still come
    }
  }

  void handleScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                  const Eigen::Isometry3d& T_odom, double t) {
    ++n_paired_;
    CloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    try {
      if (!backend_->processScan(t, T_odom, cloud)) return;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "processScan failed at t=%.3f: %s", t,
                   e.what());
      return;
    }
    if (!align_logged_ && backend_->aligned()) {
      align_logged_ = true;
      const std::optional<Eigen::Isometry3d> T = backend_->mapToOdomTransform();
      RCLCPP_INFO(get_logger(),
                  "GNSS alignment at t=%.3f after %d keyframes: %s -> %s = %s",
                  t, backend_->numKeyframes(), cfg_.map_frame.c_str(),
                  cfg_.origin_frame.c_str(), T ? describe(*T).c_str() : "none");
    }
    publishPath(msg->header.stamp);
  }

  void gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      ++n_gnss_nofix_;
      return;
    }
    GnssFix fix;
    fix.t = rclcpp::Time(msg->header.stamp).seconds();
    fix.p_map =
        projector_->llaToMap(msg->latitude, msg->longitude, msg->altitude);
    // Zero cov = unknown: Backend substitutes gnss_default_sigma.
    if (msg->position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      fix.cov = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          msg->position_covariance.data());
    }
    backend_->addGnssFix(fix);
    ++n_gnss_;
  }

  // ---- loop thread, map timer, service
  // ----------------------------------------
  void loopThread() {
    const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.01, np_.loop_frequency));
    while (waitFor(period)) {
      try {
        if (backend_->tryLoopClosure()) {
          RCLCPP_INFO(get_logger(), "loop closure accepted (%d so far)",
                      backend_->numLoops());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "loop closure failed: %s", e.what());
      }
    }
  }

  // Sleeps for period; false when the node is stopping.
  bool waitFor(const std::chrono::duration<double>& period) {
    std::unique_lock<std::mutex> lock(stop_mtx_);
    stop_cv_.wait_for(lock, period, [this]() { return !running_.load(); });
    return running_.load();
  }

  void mapTimer() {
    if (pub_map_->get_subscription_count() == 0) return;
    const CloudPtr map = backend_->buildGlobalMap(cfg_.save_resolution);
    if (map->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.stamp = now();
    msg.header.frame_id = cfg_.map_frame;
    pub_map_->publish(msg);
  }

  void saveMapCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
    res->message = saveMap("service");
    res->success = res->message.rfind("saved", 0) == 0;
  }

  // Runs the writer and returns the log line.
  std::string saveMap(const char* who) {
    std::string message;
    try {
      message = backend_->saveMap(np_.save_dir, np_.source)
                    ? "saved " + std::to_string(backend_->numKeyframes()) +
                          " keyframes, " +
                          std::to_string(backend_->numLoops()) + " loops (" +
                          std::to_string(backend_->numLoopsRejected()) +
                          " rejected) to " + np_.save_dir
                    : "no keyframes yet, nothing saved";
    } catch (const std::exception& e) {
      message = std::string("save failed: ") + e.what();
    }
    RCLCPP_INFO(get_logger(), "save_map (%s): %s", who, message.c_str());
    if (message.rfind("saved", 0) == 0) {
      saved_keyframes_ = backend_->numKeyframes();
      saved_loops_ = backend_->numLoops();
    }
    return message;
  }

  // ---- outputs
  // ----------------------------------------------------------------
  void publishPath(const builtin_interfaces::msg::Time& stamp) {
    if (pub_path_->get_subscription_count() == 0) return;
    const std::vector<Eigen::Isometry3d> poses = backend_->keyframePathInMap();
    if (poses.empty()) return;  // not aligned yet
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = cfg_.map_frame;
    path.poses.reserve(poses.size());
    for (const auto& T : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = toPoseMsg(T);
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  void printStatus() {
    const uint64_t scans = n_scans_.load();
    if (scans == last_status_scans_) return;  // nothing new, stay quiet
    last_status_scans_ = scans;
    RCLCPP_INFO(get_logger(),
                "scans %lu paired %lu unsynced %lu | gnss %lu (no fix %lu) | "
                "keyframes %d | loops %d (%d rejected) | aligned %d",
                static_cast<unsigned long>(scans),
                static_cast<unsigned long>(n_paired_.load()),
                static_cast<unsigned long>(n_unsynced_.load()),
                static_cast<unsigned long>(n_gnss_.load()),
                static_cast<unsigned long>(n_gnss_nofix_.load()),
                backend_->numKeyframes(), backend_->numLoops(),
                backend_->numLoopsRejected(), backend_->aligned());
  }

  // ---- state
  // ------------------------------------------------------------------
  NodeParams np_;
  BackendConfig cfg_;
  std::unique_ptr<Backend> backend_;
  std::unique_ptr<geo::Projector> projector_;  // lat/lon/alt -> map metres

  // Ingest group only.
  std::map<int64_t, Eigen::Isometry3d> odom_buf_;  // stamp [ns] -> pose
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_;
  bool align_logged_ = false;

  std::atomic<uint64_t> n_scans_{0}, n_paired_{0}, n_unsynced_{0}, n_gnss_{0},
      n_gnss_nofix_{0};
  uint64_t last_status_scans_ = 0;

  std::atomic<bool> running_{false};
  int saved_keyframes_ = -1;  // counts at the last successful save
  int saved_loops_ = -1;
  std::mutex stop_mtx_;
  std::condition_variable stop_cv_;
  std::thread loop_thread_;

  rclcpp::CallbackGroup::SharedPtr ingest_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::TimerBase::SharedPtr map_timer_, status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace brightdrive_mapping

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<brightdrive_mapping::BackendNode>();
  // Two threads: one for the ingest group, one for the service and timers.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  executor.spin();
  node->stop();  // joins the loop thread, then saves the map
  rclcpp::shutdown();
  return 0;
}

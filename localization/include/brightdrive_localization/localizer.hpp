// The localization core around eskf::Eskf: IMU prediction, GNSS and LiDAR
// pose corrections with late-measurement forwarding, the per-sweep pipeline
// (wait for the IMU, deskew, match, accept), a simple start, a provisional
// health level, the status values and the outputs. No ROS.
//
// Threads: onImu / onGnss / onInitialPose / output / statusValues from the
// executor, processScan from the matcher thread. One mutex guards all state;
// deskew, preprocess and align run without it.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "brightdrive_localization/deskewer.hpp"
#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/scan_matcher.hpp"
#include "brightdrive_localization/state_history.hpp"
#include "brightdrive_localization/types.hpp"
#include "eskf/eskf.hpp"

namespace brightdrive_localization {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// What one sweep produced, for the debug topics.
struct SweepReport {
  enum class Kind {
    kNotReady,       // no GNSS fix fresh enough to start from, or no IMU yet
    kSkippedNoImu,   // the IMU sample at the sweep end did not arrive in time
    kSkippedNoPose,  // the sweep starts before the filter's history
    kStartFailed,    // the start search found no acceptable heading
    kStarted,        // the filter was initialized on this sweep
    kMatched,        // tracking: matched, then accepted or rejected
  };
  Kind kind = Kind::kNotReady;
  double t_ref = 0.0;
  MatchResult match;
  Reject reject = Reject::kNone;
  double nis = 0.0;    // d2 of correctPose, 0 when it was not reached
  double age_s = 0.0;  // filter time - t_ref when the match was applied
  // The preprocessed sweep in the lidar frame at t_ref (deskewed when
  // tracking); with match.T_map_lidar it gives the aligned scan.
  small_gicp::PointCloud::ConstPtr source;
};

// A copy of everything the outputs need, taken under the lock.
struct Output {
  bool valid = false;  // false until the filter is initialized
  double t = 0.0;      // filter time = stamp of every output [s]
  Lifecycle lifecycle = Lifecycle::kWaitData;
  Health health = Health::kDeadReckoning;
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  // Pose covariance in the ROS order [x y z, rot x y z], rotation about the
  // map axes. m^2, rad^2.
  Matrix6d pose_cov = Matrix6d::Zero();
  Eigen::Vector3d v_base = Eigen::Vector3d::Zero();  // base_link axes [m/s]
  Eigen::Vector3d w_base = Eigen::Vector3d::Zero();  // base_link axes [rad/s]
  Matrix6d twist_cov = Matrix6d::Zero();             // [v, w], base_link axes
};

class Localizer {
 public:
  // Throws std::invalid_argument on a bad config (validate()).
  explicit Localizer(const LocalizerConfig& cfg);

  // Executor thread.
  void onImu(const eskf::ImuSample& s);
  // z.z = antenna position in map metres (llaToMap), z.V its covariance.
  void onGnss(const eskf::GnssMeas& z);
  void onInitialPose(const Eigen::Isometry3d& T_map_base, const Matrix6d& cov,
                     double t);

  // Matcher thread: one sweep through the whole pipeline.
  SweepReport processScan(Scan scan, const ScanMatcher& matcher);

  Output output() const;
  Lifecycle lifecycle() const;
  // Status key-values (diagnostic_msgs/KeyValue), formatted.
  std::vector<std::pair<std::string, std::string>> statusValues() const;

 private:
  SweepReport trackSweep(Scan scan, const ScanMatcher& matcher);
  SweepReport startSweep(Scan scan, const ScanMatcher& matcher);

  // Everything below needs mtx_ held.
  Eigen::Isometry3d mapImuLocked() const;
  Eigen::Isometry3d mapBaseLocked() const;
  void predictLocked(const eskf::ImuSample& s);
  void handleGnssLocked(const eskf::GnssMeas& z);
  void applyGnssLocked(const eskf::GnssMeas& z);
  void releaseHeldGnssLocked();
  void onMatchLocked(const MatchResult& m, double t_ref, SweepReport& rep);
  Health healthLocked() const;
  double sigmaPosLocked() const;

  LocalizerConfig cfg_;
  Eigen::Isometry3d T_imu_base_;
  Eigen::Isometry3d T_imu_lidar_;
  Eigen::Isometry3d T_lidar_imu_;
  Matrix6d V_pose_;
  Deskewer deskewer_;

  mutable std::mutex mtx_;
  std::condition_variable imu_cv_;  // notified after every IMU sample

  eskf::Eskf eskf_;
  StateHistory history_;
  Lifecycle lifecycle_ = Lifecycle::kWaitData;
  Eigen::Isometry3d T_odom_base_ = Eigen::Isometry3d::Identity();
  std::deque<eskf::GnssMeas> held_gnss_;  // fixes newer than the filter

  bool have_imu_ = false;
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  bool have_fix_ = false;
  eskf::GnssMeas latest_fix_;
  bool have_initialpose_ = false;
  double initialpose_t_ = 0.0;
  double initialpose_yaw_ = 0.0;

  // Status.
  bool have_lidar_accept_ = false;
  double last_lidar_accept_t_ = 0.0;  // t_ref of the last accepted match
  bool have_gnss_accept_ = false;
  double last_gnss_accept_t_ = 0.0;
  double last_nis_pose_ = 0.0;
  double last_nis_gnss_ = 0.0;
  double nis_pose_sum_ = 0.0;
  long nis_pose_n_ = 0;
  double last_error_per_inlier_ = 0.0;
  double last_inlier_ratio_ = 0.0;
  double last_match_ms_ = 0.0;
  double last_age_ms_ = 0.0;
  int consecutive_gated_ = 0;
  long sweeps_skipped_no_imu_ = 0;
  long sweeps_skipped_no_pose_ = 0;
  long start_attempts_ = 0;
  std::array<long, kNumRejectCodes> reject_counts_{};
  long gnss_exact_ = 0, gnss_late_ = 0, gnss_held_ = 0, gnss_dropped_ = 0;
  long gnss_accepted_ = 0, gnss_gated_ = 0;
  std::string start_info_ = "not started";
};

}  // namespace brightdrive_localization

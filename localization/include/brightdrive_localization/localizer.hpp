// The localization core around eskf::Eskf: IMU prediction, GNSS and LiDAR
// pose corrections with late-measurement forwarding, the per-sweep pipeline
// (wait for the IMU, deskew, match, accept), initialization (search, catch-up,
// confirmation), a provisional health level, the status values and the
// outputs. No ROS.
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
#include "brightdrive_localization/health_monitor.hpp"
#include "brightdrive_localization/initializer.hpp"
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
    kNotReady,       // waiting for GNSS, for the IMU window, or not tracking
    kSkippedNoImu,   // the IMU sample at the sweep end did not arrive in time
    kSkippedNoPose,  // the sweep starts before the filter's history
    kStartFailed,    // the initializer found no acceptable pose on this sweep
    kStarted,        // the filter was initialized on this sweep (confirming)
    kMatched,  // matched, then accepted or rejected (confirming or tracking)
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
  // [lifecycle 0 wait / 1 init / 2 tracking, health 0 nominal / 1 degraded /
  //  2 dead reckoning / 3 failed (-1 not tracking), sigma_pos_m, gnss_age_s,
  //  nis_pose, error_per_inlier, match_age_ms, consecutive_gated, scan_age_s]
  std::vector<double> healthNumbers() const;
  // Status key-values (diagnostic_msgs/KeyValue), formatted.
  std::vector<std::pair<std::string, std::string>> statusValues() const;

 private:
  SweepReport trackSweep(Scan scan, const ScanMatcher& matcher);
  SweepReport initSweep(Scan scan, const ScanMatcher& matcher);

  // Everything below needs mtx_ held.
  Eigen::Isometry3d mapImuLocked() const;
  Eigen::Isometry3d mapBaseLocked() const;
  void predictLocked(const eskf::ImuSample& s);
  void handleGnssLocked(const eskf::GnssMeas& z);
  void applyGnssLocked(const eskf::GnssMeas& z);
  void releaseHeldGnssLocked();
  void onMatchLocked(const MatchResult& m, double t_ref, SweepReport& rep);
  // The filter goes back to "not initialized": search again.
  void discardFilterLocked(const std::string& why);
  void evaluateHealthLocked();
  bool stateFiniteLocked() const;
  double sigmaPosLocked() const;

  LocalizerConfig cfg_;
  Eigen::Isometry3d T_imu_base_;
  Eigen::Isometry3d T_imu_lidar_;
  Eigen::Isometry3d T_lidar_imu_;
  Matrix6d V_pose_;
  Deskewer deskewer_;
  Initializer initializer_;
  HealthMonitor health_;

  mutable std::mutex mtx_;
  std::condition_variable imu_cv_;  // notified after every IMU sample

  eskf::Eskf eskf_;
  StateHistory history_;
  Lifecycle lifecycle_ = Lifecycle::kWaitData;
  // The filter runs (predicts, takes fixes and matches) while confirming and
  // while tracking.
  bool filter_active_ = false;
  bool confirming_ = false;
  int confirm_count_ = 0;
  Eigen::Vector3d b_g_prev_ =
      Eigen::Vector3d::Zero();  // from a discarded filter
  Eigen::Isometry3d T_odom_base_ = Eigen::Isometry3d::Identity();
  std::deque<eskf::GnssMeas> held_gnss_;  // fixes newer than the filter

  bool have_imu_ = false;
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  bool have_fix_ = false;
  eskf::GnssMeas latest_fix_;
  bool have_initialpose_ = false;
  double initialpose_t_ = 0.0;
  double initialpose_yaw_ = 0.0;

  // Health, evaluated after every IMU step while tracking.
  Health health_now_ = Health::kNominal;
  long failures_ = 0;
  bool recovering_ = false;  // initializing again after a FAILED
  std::string last_fail_reason_ = "-";

  // Status.
  double last_nis_pose_ = 0.0;
  double last_nis_gnss_ = 0.0;
  double nis_pose_sum_ = 0.0;
  long nis_pose_n_ = 0;
  double last_error_per_inlier_ = 0.0;
  double last_inlier_ratio_ = 0.0;
  double last_match_ms_ = 0.0;
  double last_age_ms_ = 0.0;
  long sweeps_skipped_no_imu_ = 0;
  long sweeps_skipped_no_pose_ = 0;
  long init_searches_ = 0;
  long init_confirm_failures_ = 0;
  bool have_init_start_t_ = false;
  double init_start_t_ =
      0.0;  // first IMU sample: the clock of time_to_tracking
  double time_to_tracking_s_ = -1.0;
  std::string init_last_result_ = "-";
  std::array<long, kNumRejectCodes> reject_counts_{};
  long gnss_exact_ = 0, gnss_late_ = 0, gnss_held_ = 0, gnss_dropped_ = 0;
  long gnss_accepted_ = 0, gnss_gated_ = 0;
  std::string init_state_ = "waiting for IMU";
};

}  // namespace brightdrive_localization

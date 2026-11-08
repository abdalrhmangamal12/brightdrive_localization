// Global initialization on the map: from a position seed (GNSS fix or
// /initialpose) and an optional heading hint to the full pose of the IMU at one
// sweep's time, by a heading grid on the coarse voxel map refined on the fine
// one; plus what the start needs besides the pose: the still test, the GNSS
// velocity, x0 and P0. Stateless apart from its settings; the matcher thread
// calls search() without the localizer's lock. The confirmation (confirm_n
// accepted sweeps through the normal update) and the catch-up are the
// Localizer's.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/scan_matcher.hpp"
#include "brightdrive_localization/types.hpp"
#include "eskf/types.hpp"

namespace brightdrive_localization {

enum class SeedSource { kGnss, kInitialPose };
enum class YawHintSource { kNone, kCog, kInitialPose };
const char* toString(SeedSource s);
const char* toString(YawHintSource s);

struct InitSeed {
  SeedSource source = SeedSource::kGnss;
  // One point of the vehicle in map [m], and the same point in IMU axes: the
  // GNSS antenna (lever arm) or the base_link origin. The IMU position for a
  // heading R is p_map - R p_imu.
  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_imu = Eigen::Vector3d::Zero();
  std::optional<double> yaw_hint;  // IMU yaw [rad]; the grid starts here
  YawHintSource yaw_hint_source = YawHintSource::kNone;
  // GNSS course over ground as an IMU yaw [rad] and the GNSS speed [m/s], set
  // whenever the GNSS velocity is ok: the winner is checked against it.
  std::optional<double> cog;
  double speed = 0.0;
};

struct InitResult {
  bool found = false;
  std::string reason;  // why not, when found is false
  Eigen::Isometry3d T_map_imu =
      Eigen::Isometry3d::Identity();  // at the sweep time
  MatchResult match;                  // the winner's fine match
  double error_per_inlier = std::numeric_limits<double>::infinity();
  // The best fine result that is a different pose; inf when every refined
  // candidate converged to the winner.
  double second_error_per_inlier = std::numeric_limits<double>::infinity();
  int candidates = 0;          // coarse results that passed the gross checks
  int distinct = 0;            // distinct poses among the refined ones
  double cog_diff_deg = -1.0;  // |winner yaw - COG|; -1 when not checked
  double search_ms = 0.0;
};

// Least-squares velocity of the GNSS antenna over the fixes of a window.
struct GnssVelocity {
  bool ok = false;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();  // map [m/s], vz = 0
  double sigma = 0.0;  // per horizontal axis [m/s], floored
  int fixes = 0;
  double span_s = 0.0;
  double speed() const { return v.head<2>().norm(); }
};

// Sums over IMU samples for the still test, filled one sample at a time.
struct ImuStillSums {
  int n = 0;
  double t_first = 0.0;
  double t_last = 0.0;
  Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
  double gyro_max = 0.0;  // largest |gyro| [rad/s]
  double acc_sum = 0.0;   // of |acc|
  double acc_sq = 0.0;
  void add(const eskf::ImuSample& s);
};

// The still verdict and the numbers behind it.
struct StillWindow {
  bool still = false;
  std::string why;  // the first failed condition; empty when still
  Eigen::Vector3d b_g =
      Eigen::Vector3d::Zero();  // mean gyro when still [rad/s]
  int samples = 0;
  double gyro_max = 0.0;      // [rad/s]
  double acc_std = 0.0;       // of |acc| [m/s^2]
  double acc_mean_dev = 0.0;  // |mean |acc| - |g|| [m/s^2]
  bool gnss_used = false;     // the GNSS velocity was ok
  double gnss_speed = 0.0;    // [m/s]
  double gnss_bound = 0.0;    // [m/s]
};

// v0 and its sigma for P0.
struct InitVelocity {
  Eigen::Vector3d v = Eigen::Vector3d::Zero();  // map [m/s]
  double sigma = 0.0;                           // per axis [m/s]
};

class Initializer {
 public:
  Initializer(const InitConfig& init, const AcceptConfig& accept,
              const Eigen::Isometry3d& T_imu_lidar, double gravity_mps2);

  // Estimator core: the heading grid, its scoring and the COG check. source =
  // the raw sweep, preprocessed (no deskew: no poses yet).
  InitResult search(const small_gicp::PointCloud& source, const InitSeed& seed,
                    const ScanMatcher& matcher) const;

  // Estimator core. fixes = the raw fixes of the window (antenna, map); ok
  // needs vel_min_fixes spanning vel_min_span_s.
  GnssVelocity gnssVelocity(const std::vector<eskf::GnssMeas>& fixes) const;

  // Estimator core. Still = every gyro sample below still_max_gyro_radps,
  // |acc| spread and mean deviation from |g| below their limits, and, when gv
  // is given and ok, the GNSS speed below its noise-scaled bound.
  // min_span_s: the samples must cover at least this long.
  StillWindow still(const ImuStillSums& sums, double min_span_s,
                    const GnssVelocity* gv) const;
  // The still test over the window before the sweep (needs half of it).
  StillWindow stillWindow(const std::vector<eskf::ImuSample>& imu,
                          const GnssVelocity& gv) const;

  // Estimator core. Standing: v0 = 0 with p0_sigma_vel_mps; moving with a GNSS
  // velocity: that velocity and its sigma; else v0 = 0 with
  // p0_sigma_vel_moving_mps.
  InitVelocity initVelocity(const StillWindow& w, const GnssVelocity& gv) const;
  // x0 from the found pose and v0; b_g from the still window, else b_g_prev.
  eskf::NominalState x0(const InitResult& r, const StillWindow& w,
                        const InitVelocity& v0,
                        const Eigen::Vector3d& b_g_prev) const;
  // The inflated P0 of the config with sigma_v of v0.
  eskf::Matrix15d P0(const InitVelocity& v0) const;

 private:
  InitConfig init_;
  AcceptConfig accept_;
  Eigen::Isometry3d T_imu_lidar_;
  Eigen::Isometry3d T_lidar_imu_;
  double gravity_;  // |g| [m/s^2]
};

}  // namespace brightdrive_localization

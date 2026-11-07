// Global initialization on the map: from a GNSS fix (position) and an
// optional heading hint (/initialpose) to the full pose of the IMU at one
// sweep's time, by a heading grid on the coarse voxel map refined on the fine
// one. Stateless apart from its settings; the matcher thread calls it without
// the localizer's lock. The confirmation (confirm_n accepted sweeps through
// the normal update) and the catch-up are the Localizer's.
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

struct InitSeed {
  eskf::GnssMeas fix;              // raw: antenna position in map [m]
  std::optional<double> yaw_hint;  // [rad], from /initialpose
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
  int candidates = 0;  // coarse results that passed the gross checks
  int distinct = 0;    // distinct poses among the refined ones
  double search_ms = 0.0;
};

// What the IMU says over the still window before the sweep.
struct StillWindow {
  bool still = false;
  Eigen::Vector3d b_g =
      Eigen::Vector3d::Zero();  // mean gyro when still [rad/s]
  int samples = 0;
};

class Initializer {
 public:
  Initializer(const InitConfig& init, const AcceptConfig& accept,
              const Eigen::Vector3d& lever_arm,
              const Eigen::Isometry3d& T_imu_lidar);

  // Estimator core: the heading grid and its scoring. source = the raw sweep,
  // preprocessed (no deskew: no poses yet).
  InitResult search(const small_gicp::PointCloud& source, const InitSeed& seed,
                    const ScanMatcher& matcher) const;

  // Standing still over the window: no gyro sample above 0.05 rad/s and a
  // specific-force spread below 0.2 m/s^2. Needs half the window covered.
  StillWindow stillWindow(const std::vector<eskf::ImuSample>& imu) const;

  // x0 from the found pose; v0 = 0; b_g from the still window, else b_g_prev.
  eskf::NominalState x0(const InitResult& r, const StillWindow& w,
                        const Eigen::Vector3d& b_g_prev) const;
  // The inflated P0 of the config; sigma_v small when still, large when not.
  eskf::Matrix15d P0(const StillWindow& w) const;

 private:
  InitConfig init_;
  AcceptConfig accept_;
  Eigen::Vector3d lever_arm_;
  Eigen::Isometry3d T_imu_lidar_;
  Eigen::Isometry3d T_lidar_imu_;
};

}  // namespace brightdrive_localization

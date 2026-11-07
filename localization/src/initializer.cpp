#include "brightdrive_localization/initializer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace brightdrive_localization {

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kDeg = M_PI / 180.0;
// Two matches are the same solution when their lidar poses are this close;
// adjacent headings of the grid usually converge to one pose.
constexpr double kSameSolutionM = 1.0;
constexpr double kSameSolutionRad = 5.0 * kDeg;
constexpr double kStillMaxGyro = 0.05;   // [rad/s]
constexpr double kStillMaxAccStd = 0.2;  // [m/s^2]

bool samePose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm() < kSameSolutionM &&
         rotationAngle(a.linear().transpose() * b.linear()) < kSameSolutionRad;
}

std::string fmt(const char* f, double v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

}  // namespace

Initializer::Initializer(const InitConfig& init, const AcceptConfig& accept,
                         const Eigen::Vector3d& lever_arm,
                         const Eigen::Isometry3d& T_imu_lidar)
    : init_(init),
      accept_(accept),
      lever_arm_(lever_arm),
      T_imu_lidar_(T_imu_lidar),
      T_lidar_imu_(T_imu_lidar.inverse()) {}

// Estimator core. Scoring = error per inlier (lower is better): the inlier
// ratio sits near 1.0 for every aligned sweep and does not separate headings.
InitResult Initializer::search(const small_gicp::PointCloud& source,
                               const InitSeed& seed,
                               const ScanMatcher& matcher) const {
  const auto t0 = Clock::now();
  InitResult res;
  if (static_cast<int>(source.size()) < accept_.min_source_points) {
    res.reason = "too few points";
    return res;
  }
  // 1. Heading grid on the coarse map at the seed. The grid starts at the
  //    hint, so the hinted heading is one of the candidates.
  const double base = seed.yaw_hint.value_or(0.0);
  std::vector<MatchResult> coarse;
  for (int k = 0; k < init_.yaw_steps; ++k) {
    const double yaw = base + 2.0 * M_PI * k / init_.yaw_steps;
    Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
    T_map_imu.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // The IMU position for this heading: antenna minus the rotated lever arm.
    // Only for this search; the stored fix stays raw.
    T_map_imu.translation() = seed.fix.z - T_map_imu.linear() * lever_arm_;
    MatchResult m = matcher.align(source, T_map_imu * T_imu_lidar_,
                                  ScanMatcher::Level::kCoarse);
    if (m.num_inliers > 0 && m.inlierRatio() >= accept_.min_inlier_ratio) {
      coarse.push_back(m);
    }
  }
  res.candidates = static_cast<int>(coarse.size());
  std::sort(coarse.begin(), coarse.end(),
            [](const MatchResult& a, const MatchResult& b) {
              return a.errorPerInlier() < b.errorPerInlier();
            });

  // 2. The best refine_top DISTINCT coarse solutions, refined on the fine map.
  std::vector<MatchResult> fine;
  std::vector<Eigen::Isometry3d> kept;
  for (const MatchResult& c : coarse) {
    if (static_cast<int>(kept.size()) >= init_.refine_top) break;
    if (std::any_of(kept.begin(), kept.end(), [&](const Eigen::Isometry3d& k) {
          return samePose(k, c.T_map_lidar);
        })) {
      continue;
    }
    kept.push_back(c.T_map_lidar);
    MatchResult f =
        matcher.align(source, c.T_map_lidar, ScanMatcher::Level::kFine);
    if (f.converged && f.num_inliers > 0) fine.push_back(f);
  }
  std::sort(fine.begin(), fine.end(),
            [](const MatchResult& a, const MatchResult& b) {
              return a.errorPerInlier() < b.errorPerInlier();
            });
  res.search_ms =
      std::chrono::duration<double>(Clock::now() - t0).count() * 1e3;
  if (fine.empty()) {
    res.reason = "no candidate converged";
    return res;
  }

  // 3. Winner and the best different pose behind it.
  const MatchResult& win = fine.front();
  res.match = win;
  res.error_per_inlier = win.errorPerInlier();
  res.distinct = 1;
  for (size_t i = 1; i < fine.size(); ++i) {
    if (samePose(fine[i].T_map_lidar, win.T_map_lidar)) continue;
    if (res.distinct == 1)
      res.second_error_per_inlier = fine[i].errorPerInlier();
    ++res.distinct;
  }
  res.T_map_imu = win.T_map_lidar * T_lidar_imu_;
  if (res.error_per_inlier > accept_.max_error_per_inlier) {
    res.reason = "best e/inl " + fmt("%.3f", res.error_per_inlier) +
                 " above the acceptance limit";
    return res;
  }
  if (win.inlierRatio() < accept_.min_inlier_ratio) {
    res.reason = "best inlier ratio " + fmt("%.2f", win.inlierRatio());
    return res;
  }
  if (res.second_error_per_inlier - res.error_per_inlier < init_.margin) {
    res.reason = "ambiguous: second e/inl " +
                 fmt("%.3f", res.second_error_per_inlier) + " vs best " +
                 fmt("%.3f", res.error_per_inlier);
    return res;
  }
  res.found = true;
  return res;
}

StillWindow Initializer::stillWindow(
    const std::vector<eskf::ImuSample>& imu) const {
  StillWindow w;
  w.samples = static_cast<int>(imu.size());
  if (imu.size() < 10 ||
      imu.back().t - imu.front().t < 0.5 * init_.still_window_s) {
    return w;
  }
  Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
  double gyro_max = 0.0, acc_sum = 0.0, acc_sq = 0.0;
  for (const auto& s : imu) {
    gyro_sum += s.gyro;
    gyro_max = std::max(gyro_max, s.gyro.norm());
    const double a = s.acc.norm();
    acc_sum += a;
    acc_sq += a * a;
  }
  const double n = static_cast<double>(imu.size());
  const double mean = acc_sum / n;
  const double std = std::sqrt(std::max(0.0, acc_sq / n - mean * mean));
  w.still = gyro_max < kStillMaxGyro && std < kStillMaxAccStd;
  if (w.still) w.b_g = gyro_sum / n;
  return w;
}

eskf::NominalState Initializer::x0(const InitResult& r, const StillWindow& w,
                                   const Eigen::Vector3d& b_g_prev) const {
  eskf::NominalState x;
  x.p = r.T_map_imu.translation();
  x.q = Eigen::Quaterniond(r.T_map_imu.linear()).normalized();
  x.b_g = w.still ? w.b_g : b_g_prev;
  return x;
}

eskf::Matrix15d Initializer::P0(const StillWindow& w) const {
  const double sv =
      w.still ? init_.p0_sigma_vel_mps : init_.p0_sigma_vel_moving_mps;
  const double srp = init_.p0_sigma_rollpitch_deg * kDeg;
  const double syaw = init_.p0_sigma_yaw_deg * kDeg;
  eskf::Vector15d sig;
  sig << Eigen::Vector3d::Constant(init_.p0_sigma_pos_m),
      Eigen::Vector3d::Constant(sv), srp, srp, syaw,
      Eigen::Vector3d::Constant(init_.p0_sigma_ba),
      Eigen::Vector3d::Constant(init_.p0_sigma_bg);
  return sig.cwiseAbs2().asDiagonal();
}

}  // namespace brightdrive_localization

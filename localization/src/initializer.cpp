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
// Fewer samples than this say nothing about standing still (0.1 s at 100 Hz).
constexpr int kStillMinSamples = 10;

bool samePose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm() < kSameSolutionM &&
         rotationAngle(a.linear().transpose() * b.linear()) < kSameSolutionRad;
}

double wrapPi(double a) { return std::remainder(a, 2.0 * M_PI); }

std::string fmt(const char* f, double v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

}  // namespace

const char* toString(SeedSource s) {
  return s == SeedSource::kGnss ? "gnss" : "initialpose";
}

const char* toString(YawHintSource s) {
  switch (s) {
    case YawHintSource::kCog:
      return "cog";
    case YawHintSource::kInitialPose:
      return "initialpose";
    default:
      return "none";
  }
}

void ImuStillSums::add(const eskf::ImuSample& s) {
  if (n == 0) t_first = s.t;
  t_last = s.t;
  ++n;
  gyro_sum += s.gyro;
  gyro_max = std::max(gyro_max, s.gyro.norm());
  const double a = s.acc.norm();
  acc_sum += a;
  acc_sq += a * a;
}

Initializer::Initializer(const InitConfig& init, const AcceptConfig& accept,
                         const Eigen::Isometry3d& T_imu_lidar,
                         double gravity_mps2)
    : init_(init),
      accept_(accept),
      T_imu_lidar_(T_imu_lidar),
      T_lidar_imu_(T_imu_lidar.inverse()),
      gravity_(gravity_mps2) {}

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
  // 1. Every heading of the grid on the coarse map at the seed. The grid
  //    starts at the hint, so the hinted heading is one of the candidates.
  const double base = seed.yaw_hint.value_or(0.0);
  std::vector<MatchResult> coarse;
  for (int k = 0; k < init_.yaw_steps; ++k) {
    const double yaw = base + 2.0 * M_PI * k / init_.yaw_steps;
    Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
    T_map_imu.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // The IMU position for this heading: the seeded point minus its rotated
    // offset (antenna lever arm or base_link origin).
    T_map_imu.translation() = seed.p_map - T_map_imu.linear() * seed.p_imu;
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
  // 4. At speed the car moves along its heading: a winner that points
  //    elsewhere is a wrong match, however good its score.
  // Below cog_min_speed_mps the course is GNSS noise: no difference reported.
  if (seed.cog && seed.speed > init_.cog_min_speed_mps) {
    const double yaw = yawOf(res.T_map_imu.linear());
    res.cog_diff_deg = std::abs(wrapPi(yaw - *seed.cog)) / kDeg;
    if (seed.speed > init_.cog_check_min_speed_mps &&
        res.cog_diff_deg > init_.cog_max_diff_deg) {
      res.reason = "winner yaw " + fmt("%.1f", yaw / kDeg) +
                   " deg disagrees with COG " + fmt("%.1f", *seed.cog / kDeg) +
                   " deg at " + fmt("%.1f m/s", seed.speed);
      return res;
    }
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

// Estimator core. Per horizontal axis, v = sum (t - tm)(x - xm) / sum (t -
// tm)^2; the sigma of that slope is the fix sigma / sqrt(sum (t - tm)^2).
// Antenna velocity = IMU velocity + R (w x lever arm); with the lever arm
// straight up only roll and pitch rates enter, ignored here.
GnssVelocity Initializer::gnssVelocity(
    const std::vector<eskf::GnssMeas>& fixes) const {
  GnssVelocity g;
  g.fixes = static_cast<int>(fixes.size());
  if (g.fixes < init_.vel_min_fixes) return g;
  double t_min = fixes.front().t, t_max = fixes.front().t;
  double tm = 0.0, var_h = 0.0;
  Eigen::Vector2d xm = Eigen::Vector2d::Zero();
  for (const auto& f : fixes) {
    t_min = std::min(t_min, f.t);
    t_max = std::max(t_max, f.t);
    tm += f.t;
    xm += f.z.head<2>();
    var_h += 0.5 * (f.V(0, 0) + f.V(1, 1));
  }
  g.span_s = t_max - t_min;
  if (g.span_s < init_.vel_min_span_s) return g;
  const double n = static_cast<double>(fixes.size());
  tm /= n;
  xm /= n;
  double stt = 0.0;
  Eigen::Vector2d stx = Eigen::Vector2d::Zero();
  for (const auto& f : fixes) {
    const double d = f.t - tm;
    stt += d * d;
    stx += d * (f.z.head<2>() - xm);
  }
  g.v.head<2>() = stx / stt;
  const double sigma_fix = std::sqrt(var_h / n);
  g.sigma = std::max(init_.vel_sigma_floor_mps, sigma_fix / std::sqrt(stt));
  g.ok = true;
  return g;
}

// Estimator core.
StillWindow Initializer::still(const ImuStillSums& s, double min_span_s,
                               const GnssVelocity* gv) const {
  StillWindow w;
  w.samples = s.n;
  if (s.n < kStillMinSamples || s.t_last - s.t_first < min_span_s) {
    w.why = "too few IMU samples";
    return w;
  }
  const double n = static_cast<double>(s.n);
  const double mean = s.acc_sum / n;
  w.gyro_max = s.gyro_max;
  w.acc_std = std::sqrt(std::max(0.0, s.acc_sq / n - mean * mean));
  w.acc_mean_dev = std::abs(mean - gravity_);
  if (gv && gv->ok) {
    w.gnss_used = true;
    w.gnss_speed = gv->speed();
    w.gnss_bound = std::max(init_.still_max_gnss_speed_mps,
                            init_.still_gnss_speed_sigmas * gv->sigma);
  }
  if (w.gyro_max >= init_.still_max_gyro_radps) {
    w.why = "gyro " + fmt("%.3f rad/s", w.gyro_max);
  } else if (w.acc_std >= init_.still_max_acc_std_mps2) {
    w.why = "acc std " + fmt("%.3f m/s^2", w.acc_std);
  } else if (w.acc_mean_dev >= init_.still_max_acc_mean_dev_mps2) {
    w.why = "|acc| - g " + fmt("%.3f m/s^2", w.acc_mean_dev);
  } else if (w.gnss_used && w.gnss_speed >= w.gnss_bound) {
    w.why = "gnss speed " + fmt("%.2f m/s", w.gnss_speed);
  }
  w.still = w.why.empty();
  if (w.still) w.b_g = s.gyro_sum / n;
  return w;
}

StillWindow Initializer::stillWindow(const std::vector<eskf::ImuSample>& imu,
                                     const GnssVelocity& gv) const {
  ImuStillSums s;
  for (const auto& x : imu) s.add(x);
  return still(s, 0.5 * init_.still_window_s, &gv);
}

// Estimator core.
InitVelocity Initializer::initVelocity(const StillWindow& w,
                                       const GnssVelocity& gv) const {
  InitVelocity v;
  if (w.still) {
    v.sigma = init_.p0_sigma_vel_mps;
  } else if (gv.ok) {
    v.v = gv.v;
    v.sigma = gv.sigma;
  } else {
    v.sigma = init_.p0_sigma_vel_moving_mps;
  }
  return v;
}

eskf::NominalState Initializer::x0(const InitResult& r, const StillWindow& w,
                                   const InitVelocity& v0,
                                   const Eigen::Vector3d& b_g_prev) const {
  eskf::NominalState x;
  x.p = r.T_map_imu.translation();
  x.q = Eigen::Quaterniond(r.T_map_imu.linear()).normalized();
  x.v = v0.v;
  x.b_g = w.still ? w.b_g : b_g_prev;
  return x;
}

eskf::Matrix15d Initializer::P0(const InitVelocity& v0) const {
  const double srp = init_.p0_sigma_rollpitch_deg * kDeg;
  const double syaw = init_.p0_sigma_yaw_deg * kDeg;
  eskf::Vector15d sig;
  sig << Eigen::Vector3d::Constant(init_.p0_sigma_pos_m),
      Eigen::Vector3d::Constant(v0.sigma), srp, srp, syaw,
      Eigen::Vector3d::Constant(init_.p0_sigma_ba),
      Eigen::Vector3d::Constant(init_.p0_sigma_bg);
  return sig.cwiseAbs2().asDiagonal();
}

}  // namespace brightdrive_localization

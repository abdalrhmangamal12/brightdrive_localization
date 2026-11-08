#include "brightdrive_localization/localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "eskf/so3.hpp"

namespace brightdrive_localization {

namespace {

using Clock = std::chrono::steady_clock;

// Two stamps closer than this are the same instant.
constexpr double kSameTime = 1e-9;
constexpr double kDeg = M_PI / 180.0;

const LocalizerConfig& checked(const LocalizerConfig& c) {
  validate(c);
  return c;
}

Eigen::Isometry3d isometry(const eskf::NominalState& x) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = x.q.normalized().toRotationMatrix();
  T.translation() = x.p;
  return T;
}

Eigen::Quaterniond unitQuaternion(const Eigen::Matrix3d& R) {
  return Eigen::Quaterniond(R).normalized();
}

std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

// Per-axis horizontal sigma of a fix [m].
double horizontalSigma(const Eigen::Matrix3d& V) {
  return std::sqrt(0.5 * (V(0, 0) + V(1, 1)));
}

std::string describe(const StillWindow& w) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), "gyro %.3f, acc std %.3f, |acc|-g %.3f",
                w.gyro_max, w.acc_std, w.acc_mean_dev);
  std::string d = buf;
  if (w.gnss_used) {
    std::snprintf(buf, sizeof(buf), ", gnss %.2f/%.2f m/s", w.gnss_speed,
                  w.gnss_bound);
    d += buf;
  }
  return (w.still ? "yes (" : "no: " + w.why + " (") + d + ")";
}

}  // namespace

Localizer::Localizer(const LocalizerConfig& cfg)
    : cfg_(checked(cfg)),
      T_imu_base_(cfg.extrinsics.T_base_imu.inverse()),
      T_imu_lidar_(T_imu_base_ * cfg.extrinsics.T_base_lidar),
      T_lidar_imu_(T_imu_lidar_.inverse()),
      V_pose_(Matrix6d::Zero()),
      deskewer_(T_imu_lidar_),
      initializer_(cfg.init, cfg.accept, T_imu_lidar_, cfg.eskf.g.norm()),
      health_(cfg.health),
      eskf_(cfg.eskf),
      history_(cfg.history.keep_s) {
  const double sp = cfg.meas.sigma_pos_m * cfg.meas.sigma_pos_m;
  const double sr = cfg.meas.sigma_rot_rad * cfg.meas.sigma_rot_rad;
  V_pose_.diagonal() << sp, sp, sp, sr, sr, sr;
}

// ---- executor thread
// ----------------------------------------------------------------------------------------

void Localizer::onImu(const eskf::ImuSample& s_in) {
  eskf::ImuSample s = s_in;
  s.t += cfg_.extrinsics.imu_time_offset_s;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    Event e;
    e.t = s.t;
    e.kind = Event::Kind::kImu;
    e.imu = s;
    history_.pushEvent(e);
    if (!have_imu_) {
      have_init_start_t_ = true;
      init_start_t_ = s.t;
    }
    have_imu_ = true;
    last_imu_t_ = s.t;
    last_gyro_ = s.gyro;
    if (have_initialpose_ && s.t >= initialpose_t_ - kSameTime) {
      initialpose_still_.add(s);
    }
    if (lifecycle_ == Lifecycle::kWaitData) {
      lifecycle_ = Lifecycle::kInitializing;
      init_phase_start_t_ = s.t;
      init_state_ = "waiting for GNSS or /initialpose";
    }
    if (filter_active_) {
      predictLocked(s);
      releaseHeldGnssLocked();
      if (lifecycle_ == Lifecycle::kTracking) evaluateHealthLocked();
    }
  }
  imu_cv_.notify_all();
}

void Localizer::onGnss(const eskf::GnssMeas& z) {
  std::lock_guard<std::mutex> lk(mtx_);
  Event e;
  e.t = z.t;
  e.kind = Event::Kind::kGnss;
  e.gnss = z;  // raw: antenna position, as correctGnss takes it
  history_.pushEvent(e);
  if (!have_fix_ || z.t >= latest_fix_.t) latest_fix_ = z;
  have_fix_ = true;
  if (filter_active_) handleGnssLocked(z);
}

void Localizer::onInitialPose(const Eigen::Isometry3d& T_map_base,
                              const Matrix6d& /*cov*/, double t) {
  std::lock_guard<std::mutex> lk(mtx_);
  have_initialpose_ = true;
  initialpose_t_ = t;
  initialpose_T_map_base_ = T_map_base;
  // The samples since the stamp that already arrived are in the history; when
  // it no longer reaches back to the stamp (or to the first sample, for a
  // stamp older than the node), stillness since then cannot be checked.
  initialpose_still_ = ImuStillSums();
  initialpose_covered_ =
      !have_imu_ ||
      (history_.hasEvents() &&
       history_.oldestEvent() <= std::max(t, init_start_t_) + kSameTime);
  if (!have_imu_) return;
  for (const Event& e : history_.eventsAfter(t - kSameTime)) {
    if (e.kind == Event::Kind::kImu) initialpose_still_.add(e.imu);
  }
}

// ---- filter steps (mtx_ held)
// -------------------------------------------------------------------------------

Eigen::Isometry3d Localizer::mapImuLocked() const {
  return isometry(eskf_.state());
}

Eigen::Isometry3d Localizer::mapBaseLocked() const {
  return mapImuLocked() * T_imu_base_;
}

void Localizer::predictLocked(const eskf::ImuSample& s) {
  if (!(s.t > eskf_.time() + kSameTime)) return;  // the filter would skip it
  const Eigen::Isometry3d before = mapBaseLocked();
  eskf_.predict(s);
  const Eigen::Isometry3d after = mapBaseLocked();
  // odom -> base_link moves only by prediction, so it never jumps; every
  // correction lands in map -> odom.
  T_odom_base_ = T_odom_base_ * (before.inverse() * after);
  T_odom_base_.linear() =
      unitQuaternion(T_odom_base_.linear()).toRotationMatrix();
  history_.pushPose(eskf_.time(), eskf_.state());
}

void Localizer::handleGnssLocked(const eskf::GnssMeas& z) {
  if (z.t > eskf_.time() + kSameTime) {
    // Newer than the filter: wait until predict reaches it.
    auto it = std::upper_bound(
        held_gnss_.begin(), held_gnss_.end(), z.t,
        [](double t, const eskf::GnssMeas& g) { return t < g.t; });
    held_gnss_.insert(it, z);
    ++gnss_held_;
    return;
  }
  applyGnssLocked(z);
}

void Localizer::applyGnssLocked(const eskf::GnssMeas& z) {
  eskf::GnssMeas zz = z;
  if (z.t < eskf_.time() - kSameTime) {
    // Older than the filter: move the fix forward by the filter's own
    // displacement since z.t.
    const std::optional<Eigen::Isometry3d> T_then = history_.poseAt(z.t);
    if (!T_then) {
      ++gnss_dropped_;
      return;
    }
    zz.z += eskf_.state().p - T_then->translation();
    ++gnss_late_;
  } else {
    ++gnss_exact_;
  }
  const Eigen::Isometry3d before = mapImuLocked();
  const eskf::UpdateResult r = eskf_.correctGnss(zz);
  last_nis_gnss_ = r.d2;
  UpdateRecord rec;
  rec.t = eskf_.time();
  rec.source = UpdateRecord::Source::kGnss;
  if (r.accepted) {
    history_.patchFrom(z.t, mapImuLocked() * before.inverse());
    rec.outcome = UpdateRecord::Outcome::kAccepted;
    ++gnss_accepted_;
  } else {
    rec.outcome = UpdateRecord::Outcome::kGated;
    ++gnss_gated_;
  }
  health_.onUpdate(rec);
}

void Localizer::releaseHeldGnssLocked() {
  while (!held_gnss_.empty() &&
         held_gnss_.front().t <= eskf_.time() + kSameTime) {
    const eskf::GnssMeas z = held_gnss_.front();
    held_gnss_.pop_front();
    applyGnssLocked(z);
  }
}

// ---- matcher thread
// -----------------------------------------------------------------------------------------

SweepReport Localizer::processScan(Scan scan, const ScanMatcher& matcher) {
  bool active;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    active = filter_active_;
  }
  if (active) return trackSweep(std::move(scan), matcher);
  return initSweep(std::move(scan), matcher);
}

SweepReport Localizer::trackSweep(Scan scan, const ScanMatcher& matcher) {
  SweepReport rep;
  rep.kind = SweepReport::Kind::kMatched;
  const double t_ref = scan.tMax();
  rep.t_ref = t_ref;
  std::vector<PoseSample> poses;
  Eigen::Isometry3d T_pred = Eigen::Isometry3d::Identity();
  {
    std::unique_lock<std::mutex> lk(mtx_);
    // The sweep is published when its last slice is measured, so the IMU
    // sample at t_ref may still be on its way.
    const bool ready = imu_cv_.wait_for(
        lk, std::chrono::duration<double>(cfg_.scan.imu_wait_s), [&] {
          return !filter_active_ ||
                 (history_.hasPoses() &&
                  history_.newestPose() >= t_ref - kSameTime);
        });
    if (!filter_active_) {
      rep.kind = SweepReport::Kind::kNotReady;
      return rep;
    }
    if (!ready) {
      ++sweeps_skipped_no_imu_;
      rep.kind = SweepReport::Kind::kSkippedNoImu;
      return rep;
    }
    poses = history_.posesCovering(scan.tMin(), t_ref);
    if (poses.empty()) {
      ++sweeps_skipped_no_pose_;
      rep.kind = SweepReport::Kind::kSkippedNoPose;
      return rep;
    }
    T_pred = *history_.poseAt(t_ref);
  }

  deskewer_.apply(scan, poses);
  const small_gicp::PointCloud::Ptr source = matcher.preprocess(scan);
  rep.source = source;
  MatchResult m;
  m.num_source = static_cast<int>(source->size());
  if (m.num_source >= cfg_.accept.min_source_points) {
    m = matcher.align(*source, T_pred * T_imu_lidar_,
                      ScanMatcher::Level::kFine);
  }
  rep.match = m;

  std::lock_guard<std::mutex> lk(mtx_);
  if (!filter_active_) {
    rep.kind = SweepReport::Kind::kNotReady;
    return rep;
  }
  onMatchLocked(m, t_ref, rep);
  if (confirming_) {
    // Confirmation: confirm_n accepted sweeps in a row through the normal
    // update, else the candidate is discarded.
    if (rep.reject == Reject::kNone) {
      ++confirm_count_;
      init_state_ = "confirming " + std::to_string(confirm_count_) + "/" +
                    std::to_string(cfg_.init.confirm_n);
      if (confirm_count_ >= cfg_.init.confirm_n) {
        confirming_ = false;
        lifecycle_ = Lifecycle::kTracking;
        recovering_ = false;
        health_now_ = Health::kNominal;
        init_state_ = "done";
        if (time_to_tracking_s_ < 0.0 && have_init_start_t_) {
          time_to_tracking_s_ = eskf_.time() - init_start_t_;
        }
      }
    } else {
      ++init_confirm_failures_;
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "sweep %d at t %.3f: %s, e/inl %.3f, nis %.2f, age %.0f ms",
                    confirm_count_ + 1, t_ref, toString(rep.reject),
                    m.errorPerInlier(), rep.nis, rep.age_s * 1e3);
      last_confirm_failure_ = buf;
      discardFilterLocked(std::string("confirmation failed: ") +
                          toString(rep.reject));
    }
  }
  return rep;
}

void Localizer::discardFilterLocked(const std::string& why) {
  // A diverged filter's gyro bias would spoil the next start: keep it only
  // when it is finite and plausible.
  const Eigen::Vector3d b_g = eskf_.state().b_g;
  b_g_prev_ = b_g.allFinite() &&
                      (b_g.array().abs() < cfg_.init.carry_bg_max_radps).all()
                  ? b_g
                  : Eigen::Vector3d::Zero();
  // The init timeout counts from leaving TRACKING, not from a failed
  // confirmation.
  if (lifecycle_ == Lifecycle::kTracking) init_phase_start_t_ = last_imu_t_;
  filter_active_ = false;
  confirming_ = false;
  confirm_count_ = 0;
  history_.clearPoses();
  held_gnss_.clear();
  lifecycle_ = Lifecycle::kInitializing;
  init_state_ = "searching (" + why + ")";
}

// Estimator core: acceptance, forwarding, pose update, history patch.
void Localizer::onMatchLocked(const MatchResult& m, double t_ref,
                              SweepReport& rep) {
  const auto& a = cfg_.accept;
  rep.age_s = eskf_.time() - t_ref;
  last_match_ms_ = m.time_ms;
  last_age_ms_ = rep.age_s * 1e3;
  last_error_per_inlier_ = m.errorPerInlier();
  last_inlier_ratio_ = m.inlierRatio();

  // Cheap checks first; the first failure is the reject code.
  Reject code = Reject::kNone;
  std::optional<Eigen::Isometry3d> T_hist;
  Eigen::Isometry3d T_meas = Eigen::Isometry3d::Identity();
  if (m.num_source < a.min_source_points) {
    code = Reject::kTooFewPoints;
  } else if (!m.converged) {
    code = Reject::kNotConverged;
  } else if (m.errorPerInlier() > a.max_error_per_inlier) {
    code = Reject::kErrorPerInlier;
  } else if (m.inlierRatio() < a.min_inlier_ratio) {
    code = Reject::kInlierRatio;
  } else if (!(T_hist = history_.poseAt(t_ref))) {
    code = Reject::kNoHistory;  // the filter's pose at t_ref is gone
  } else {
    // The match is a pose of the IMU at t_ref. Compare it with the filter's
    // pose at the same time, as patched by any correction since prepare.
    T_meas = m.T_map_lidar * T_lidar_imu_;
    const double jump_m = (T_meas.translation() - T_hist->translation()).norm();
    const double jump_rad =
        rotationAngle(T_hist->linear().transpose() * T_meas.linear());
    if (jump_m > a.max_jump_m || jump_rad > a.max_jump_rad) {
      code = Reject::kJump;
    } else if (rep.age_s > a.max_age_s) {
      code = Reject::kAge;
    }
  }

  if (code == Reject::kNone) {
    // Forward to now with the filter's own motion since t_ref:
    // T_meas(now) = T_meas(t_ref) T_hist(t_ref)^-1 T_now.
    const Eigen::Isometry3d T_now = mapImuLocked();
    const Eigen::Isometry3d T_fwd = T_meas * T_hist->inverse() * T_now;
    eskf::PoseMeas z;
    z.t = eskf_.time();
    z.p = T_fwd.translation();
    z.q = unitQuaternion(T_fwd.linear());
    z.V = V_pose_;
    const eskf::UpdateResult r = eskf_.correctPose(z);
    rep.nis = r.d2;
    last_nis_pose_ = r.d2;
    nis_pose_sum_ += r.d2;
    ++nis_pose_n_;
    if (r.accepted) {
      // The correction moved the newest pose by J (map frame). Every stored
      // pose from t_ref on gets the same jump, so the next sweep (older than
      // this correction) is forwarded through motion that contains it.
      history_.patchFrom(t_ref, mapImuLocked() * T_now.inverse());
    } else {
      code = Reject::kGated;
    }
  }
  // Health: accepted and gated matches count; quality rejects only let the
  // LiDAR timeout run.
  UpdateRecord rec;
  rec.t = eskf_.time();
  rec.source = UpdateRecord::Source::kLidar;
  rec.outcome = code == Reject::kNone ? UpdateRecord::Outcome::kAccepted
                : code == Reject::kGated
                    ? UpdateRecord::Outcome::kGated
                    : UpdateRecord::Outcome::kQualityReject;
  health_.onUpdate(rec);
  rep.reject = code;
  ++reject_counts_[static_cast<size_t>(code)];
}

// Initialization: position seed from a recent GNSS fix (else /initialpose if
// the car has not moved since it was given), heading hint from the GNSS course
// at speed (else the /initialpose heading), search on this raw sweep,
// initialize at the sweep time with v0 from the still test or the GNSS
// velocity, catch up with the stored raw events, then confirm through the
// normal update.
SweepReport Localizer::initSweep(Scan scan, const ScanMatcher& matcher) {
  SweepReport rep;
  rep.kind = SweepReport::Kind::kNotReady;
  const double t_scan = scan.tMax();
  rep.t_ref = t_scan;
  const auto& ini = cfg_.init;

  InitSeed seed;
  StillWindow still;
  GnssVelocity gv;
  InitVelocity v0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_imu_) return rep;
    // The raw inputs before the sweep: fixes for the GNSS velocity, IMU
    // samples for the still test.
    std::vector<eskf::GnssMeas> fixes;
    std::vector<eskf::ImuSample> imu;
    const double t_from =
        t_scan - std::max(ini.still_window_s, ini.vel_window_s) - kSameTime;
    for (const Event& e : history_.eventsAfter(t_from)) {
      if (e.t > t_scan + kSameTime) break;  // time order
      if (e.kind == Event::Kind::kGnss) {
        if (e.t >= t_scan - ini.vel_window_s - kSameTime) {
          fixes.push_back(e.gnss);
        }
      } else if (e.t >= t_scan - ini.still_window_s - kSameTime) {
        imu.push_back(e.imu);
      }
    }
    gv = initializer_.gnssVelocity(fixes);

    // Position seed.
    const double fix_age = std::abs(latest_fix_.t - t_scan);
    const double fix_sigma = horizontalSigma(latest_fix_.V);
    if (have_fix_ && fix_age <= ini.seed_gnss_max_age_s &&
        fix_sigma <= ini.max_seed_gnss_sigma_m) {
      seed.source = SeedSource::kGnss;
      seed.p_map = latest_fix_.z;  // raw antenna position
      seed.p_imu = cfg_.eskf.lever_arm;
    } else {
      std::string why = !have_fix_ ? "no fix"
                        : fix_age > ini.seed_gnss_max_age_s
                            ? fmt("fix %.1f s old", fix_age)
                            : fmt("fix sigma %.1f m", fix_sigma);
      const StillWindow ip =
          initializer_.still(initialpose_still_, 0.0, nullptr);
      if (have_initialpose_ && initialpose_covered_ && ip.still) {
        seed.source = SeedSource::kInitialPose;
        seed.p_map = initialpose_T_map_base_.translation();
        seed.p_imu = T_imu_base_.translation();
      } else {
        why += !have_initialpose_ ? "; no /initialpose"
               : !initialpose_covered_
                   ? "; /initialpose older than the IMU history"
                   : "; moved since /initialpose: " + ip.why;
        init_state_ = "waiting for GNSS or /initialpose (" + why + ")";
        return rep;
      }
    }
    // The still window must be in the history before the first search, so a
    // cold start sees the car standing and takes b_g from it.
    if (!history_.hasEvents() ||
        history_.oldestEvent() > t_scan - ini.still_window_s) {
      init_state_ = "waiting for the IMU window";
      return rep;
    }
    still = initializer_.stillWindow(imu, gv);
    v0 = initializer_.initVelocity(still, gv);

    // Heading hint (IMU yaw): the GNSS course at speed, else the /initialpose
    // heading. Every heading of the grid is searched either way.
    const Eigen::Matrix3d R_base_imu = cfg_.extrinsics.T_base_imu.linear();
    if (gv.ok) {
      // Course over ground = heading of base_link driving forward.
      const Eigen::Matrix3d R_cog =
          Eigen::AngleAxisd(std::atan2(gv.v.y(), gv.v.x()),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      seed.cog = yawOf(R_cog * R_base_imu);
      seed.speed = gv.speed();
    }
    if (seed.cog && seed.speed > ini.cog_min_speed_mps) {
      seed.yaw_hint = seed.cog;
      seed.yaw_hint_source = YawHintSource::kCog;
    } else if (have_initialpose_) {
      seed.yaw_hint = yawOf(initialpose_T_map_base_.linear() * R_base_imu);
      seed.yaw_hint_source = YawHintSource::kInitialPose;
    }

    last_seed_source_ = toString(seed.source);
    last_yaw_hint_source_ = toString(seed.yaw_hint_source);
    last_still_ = describe(still);
    have_init_velocity_ = true;
    last_v0_mps_ = v0.v.norm();
    last_sigma_v_mps_ = v0.sigma;
    ++init_searches_;
    init_state_ = "searching";
  }

  // Raw sweep: there are no filter poses to deskew with yet.
  const small_gicp::PointCloud::Ptr source = matcher.preprocess(scan);
  rep.source = source;
  const InitResult r = initializer_.search(*source, seed, matcher);
  rep.match = r.match;

  std::lock_guard<std::mutex> lk(mtx_);
  char buf[400];
  std::snprintf(
      buf, sizeof(buf),
      "t_scan %.3f, seed %s, e/inl %.3f, second %.3f, %d candidates, %d "
      "distinct, yaw %.1f deg, hint %s %s, cog diff %s, still %s, v0 %.2f "
      "sigma %.2f m/s, search %.0f ms%s%s",
      t_scan, toString(seed.source), r.error_per_inlier,
      r.second_error_per_inlier, r.candidates, r.distinct,
      yawOf(r.T_map_imu.linear()) / kDeg, toString(seed.yaw_hint_source),
      seed.yaw_hint ? fmt("%.1f deg", *seed.yaw_hint / kDeg).c_str() : "-",
      r.cog_diff_deg >= 0.0 ? fmt("%.1f deg", r.cog_diff_deg).c_str() : "-",
      still.still ? "yes" : "no", v0.v.norm(), v0.sigma, r.search_ms,
      r.found ? "" : ": ", r.found ? "" : r.reason.c_str());
  init_last_result_ = buf;
  if (!r.found) {
    rep.kind = SweepReport::Kind::kStartFailed;
    init_state_ = "searching (" + r.reason + ")";
    return rep;
  }
  if (!history_.hasEvents() || history_.oldestEvent() > t_scan) {
    rep.kind = SweepReport::Kind::kStartFailed;
    init_state_ = "searching (the search outlived the event history)";
    return rep;
  }

  const eskf::NominalState x0 = initializer_.x0(r, still, v0, b_g_prev_);
  eskf_.initialize(x0, initializer_.P0(v0), t_scan);
  health_.reset(t_scan);
  history_.clearPoses();
  history_.pushPose(t_scan, x0);
  T_odom_base_ = mapBaseLocked();  // odom starts on map
  held_gnss_.clear();
  filter_active_ = true;
  confirming_ = true;
  confirm_count_ = 0;
  // Catch up from the sweep to now with the stored raw inputs.
  for (const Event& e : history_.eventsAfter(t_scan)) {
    if (e.kind == Event::Kind::kImu) {
      predictLocked(e.imu);
      releaseHeldGnssLocked();
    } else {
      handleGnssLocked(e.gnss);
    }
  }
  init_state_ = "confirming 0/" + std::to_string(ini.confirm_n) +
                (still.still ? " (still: v0 = 0, b_g from the gyro mean)"
                 : gv.ok ? fmt(" (moving: v0 %.2f m/s from GNSS)", v0.v.norm())
                         : fmt(" (moving, no GNSS velocity: sigma_v %.0f m/s)",
                               v0.sigma));
  rep.kind = SweepReport::Kind::kStarted;
  return rep;
}

// ---- outputs
// ------------------------------------------------------------------------------------------------

double Localizer::sigmaPosLocked() const {
  return std::sqrt(eskf_.covariance().block<3, 3>(eskf::kP, eskf::kP).trace() /
                   3.0);
}

double Localizer::sigmaYawDegLocked() const {
  return std::sqrt(eskf_.covariance()(eskf::kTh + 2, eskf::kTh + 2)) / kDeg;
}

bool Localizer::stateFiniteLocked() const {
  const eskf::NominalState& x = eskf_.state();
  return x.p.allFinite() && x.v.allFinite() && x.q.coeffs().allFinite() &&
         x.b_a.allFinite() && x.b_g.allFinite() &&
         eskf_.covariance().allFinite();
}

// FAILED sends the localizer back to initialization (warm: the still window
// decides sigma_v, the last b_g is kept).
void Localizer::evaluateHealthLocked() {
  const HealthEval e =
      health_.evaluate(eskf_.time(), sigmaPosLocked(), stateFiniteLocked());
  health_now_ = e.health;
  if (e.health == Health::kFailed) {
    ++failures_;
    last_fail_reason_ = e.fail_reason;
    recovering_ = true;
    discardFilterLocked("health FAILED: " + e.fail_reason);
  }
}

Output Localizer::output() const {
  std::lock_guard<std::mutex> lk(mtx_);
  Output o;
  o.lifecycle = lifecycle_;
  o.valid = lifecycle_ == Lifecycle::kTracking;
  if (!o.valid) return o;
  o.t = eskf_.time();
  o.health = health_now_;
  const eskf::NominalState& x = eskf_.state();
  const eskf::Matrix15d& P = eskf_.covariance();
  const Eigen::Matrix3d R = x.q.normalized().toRotationMatrix();
  o.T_map_base = mapBaseLocked();
  o.T_odom_base = T_odom_base_;

  // Pose of base_link: p_b = p + R t_ib, R_b = R R_ib. The filter's rotation
  // error is on the right (body axes); ROS wants it about the map axes:
  // dtheta_map = R dtheta, dp_b = dp - R [t_ib]x dtheta.
  const Eigen::Vector3d t_ib = T_imu_base_.translation();
  const Eigen::Matrix3d R_ib = T_imu_base_.linear();
  Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
  J.block<3, 3>(0, eskf::kP) = Eigen::Matrix3d::Identity();
  J.block<3, 3>(0, eskf::kTh) = -R * eskf::So3::skew(t_ib);
  J.block<3, 3>(3, eskf::kTh) = R;
  o.pose_cov = J * P * J.transpose();

  // Twist of base_link in its own axes. v_body = R^T v; with R_t = R Exp(e),
  // d(v_body) = R^T dv + [R^T v]x e. omega = gyro - b_g.
  const Eigen::Vector3d w_imu = last_gyro_ - x.b_g;
  const Eigen::Vector3d v_imu_body = R.transpose() * x.v;
  o.v_base = R_ib.transpose() * (v_imu_body + w_imu.cross(t_ib));
  o.w_base = R_ib.transpose() * w_imu;
  Eigen::Matrix<double, 3, 15> A = Eigen::Matrix<double, 3, 15>::Zero();
  A.block<3, 3>(0, eskf::kV) = R_ib.transpose() * R.transpose();
  A.block<3, 3>(0, eskf::kTh) = R_ib.transpose() * eskf::So3::skew(v_imu_body);
  o.twist_cov.topLeftCorner<3, 3>() = A * P * A.transpose();
  // Angular rate: one gyro reading's noise plus the bias uncertainty.
  const double sg2 = cfg_.eskf.sigma_g * cfg_.eskf.sigma_g;
  o.twist_cov.bottomRightCorner<3, 3>() =
      R_ib.transpose() *
      (sg2 * Eigen::Matrix3d::Identity() +
       P.block<3, 3>(eskf::kBg, eskf::kBg)) *
      R_ib;
  return o;
}

Lifecycle Localizer::lifecycle() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return lifecycle_;
}

std::vector<std::pair<std::string, std::string>> Localizer::statusValues()
    const {
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<std::pair<std::string, std::string>> kv;
  const bool tracking = lifecycle_ == Lifecycle::kTracking;
  const double t = eskf_.time();
  kv.emplace_back("lifecycle", toString(lifecycle_));
  kv.emplace_back("health", tracking ? toString(health_now_) : "-");
  kv.emplace_back("failures", std::to_string(failures_));
  kv.emplace_back("last_fail_reason", last_fail_reason_);
  kv.emplace_back("recovering_from_failure", recovering_ ? "yes" : "no");
  kv.emplace_back("init", init_state_);
  kv.emplace_back("init_last_search", init_last_result_);
  kv.emplace_back("seed_source", last_seed_source_);
  kv.emplace_back("yaw_hint_source", last_yaw_hint_source_);
  kv.emplace_back("still", last_still_);
  kv.emplace_back("init_v0",
                  have_init_velocity_ ? fmt("%.2f", last_v0_mps_) : "-");
  kv.emplace_back("init_sigma_v",
                  have_init_velocity_ ? fmt("%.2f", last_sigma_v_mps_) : "-");
  const bool initializing = lifecycle_ == Lifecycle::kInitializing;
  const double init_elapsed = last_imu_t_ - init_phase_start_t_;
  kv.emplace_back("init_elapsed_s",
                  initializing ? fmt("%.1f", init_elapsed) : "-");
  kv.emplace_back(
      "init_timed_out",
      initializing && init_elapsed > cfg_.init.timeout_s ? "yes" : "no");
  kv.emplace_back("init_searches", std::to_string(init_searches_));
  kv.emplace_back("init_confirm_failures",
                  std::to_string(init_confirm_failures_));
  kv.emplace_back("init_last_confirm_failure", last_confirm_failure_);
  kv.emplace_back("time_to_tracking_s", time_to_tracking_s_ >= 0.0
                                            ? fmt("%.2f", time_to_tracking_s_)
                                            : "-");
  kv.emplace_back("filter_time_s", filter_active_ ? fmt("%.3f", t) : "-");
  kv.emplace_back("nis_pose", fmt("%.3f", last_nis_pose_));
  kv.emplace_back(
      "nis_pose_mean",
      nis_pose_n_ > 0 ? fmt("%.3f", nis_pose_sum_ / nis_pose_n_) : "-");
  kv.emplace_back("nis_gnss", fmt("%.3f", last_nis_gnss_));
  kv.emplace_back("error_per_inlier", fmt("%.4f", last_error_per_inlier_));
  kv.emplace_back("inlier_ratio", fmt("%.3f", last_inlier_ratio_));
  kv.emplace_back("match_time_ms", fmt("%.1f", last_match_ms_));
  kv.emplace_back("match_age_ms", fmt("%.1f", last_age_ms_));
  kv.emplace_back("gnss_age_s",
                  filter_active_ ? fmt("%.2f", health_.gnssAge(t)) : "-");
  kv.emplace_back("scan_age_s",
                  filter_active_ ? fmt("%.2f", health_.lidarAge(t)) : "-");
  kv.emplace_back("consecutive_gated",
                  std::to_string(health_.consecutiveGated()));
  kv.emplace_back("sigma_pos_m",
                  tracking ? fmt("%.3f", sigmaPosLocked()) : "-");
  kv.emplace_back("sigma_yaw_deg",
                  tracking ? fmt("%.3f", sigmaYawDegLocked()) : "-");
  kv.emplace_back("sweeps_skipped_no_imu",
                  std::to_string(sweeps_skipped_no_imu_));
  kv.emplace_back("sweeps_skipped_no_pose",
                  std::to_string(sweeps_skipped_no_pose_));
  for (int c = 0; c < kNumRejectCodes; ++c) {
    kv.emplace_back(std::string("match.") + toString(static_cast<Reject>(c)),
                    std::to_string(reject_counts_[static_cast<size_t>(c)]));
  }
  kv.emplace_back("gnss.accepted", std::to_string(gnss_accepted_));
  kv.emplace_back("gnss.gated", std::to_string(gnss_gated_));
  kv.emplace_back("gnss.exact", std::to_string(gnss_exact_));
  kv.emplace_back("gnss.late", std::to_string(gnss_late_));
  kv.emplace_back("gnss.held", std::to_string(gnss_held_));
  kv.emplace_back("gnss.dropped", std::to_string(gnss_dropped_));
  std::string ip = "-";
  if (have_initialpose_) {
    const StillWindow w = initializer_.still(initialpose_still_, 0.0, nullptr);
    ip = fmt("yaw %.1f deg", yawOf(initialpose_T_map_base_.linear()) / kDeg) +
         fmt(" at t %.2f s, still since: ", initialpose_t_) +
         (!initialpose_covered_ ? "unknown (older than the IMU history)"
          : w.still             ? "yes"
                                : "no (" + w.why + ")");
  }
  kv.emplace_back("initialpose", ip);
  return kv;
}

std::vector<double> Localizer::healthNumbers() const {
  std::lock_guard<std::mutex> lk(mtx_);
  const double t = eskf_.time();
  const bool tracking = lifecycle_ == Lifecycle::kTracking;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return {static_cast<double>(lifecycle_),
          tracking ? static_cast<double>(health_now_) : -1.0,
          filter_active_ ? sigmaPosLocked() : nan,
          filter_active_ ? health_.gnssAge(t) : nan,
          last_nis_pose_,
          last_error_per_inlier_,
          last_age_ms_,
          static_cast<double>(health_.consecutiveGated()),
          filter_active_ ? health_.lidarAge(t) : nan,
          last_nis_gnss_,
          filter_active_ ? sigmaYawDegLocked() : nan};
}

}  // namespace brightdrive_localization

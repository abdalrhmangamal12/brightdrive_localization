// validate(): the ">0" rule for every core key, plus the Eskf's own checks.
#include "brightdrive_localization/params.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "eskf/eskf.hpp"

namespace brightdrive_localization {

namespace {

void positive(double v, const char* key) {
  if (!(v > 0.0) || !std::isfinite(v)) {
    throw std::invalid_argument(std::string("localization.yaml: ") + key +
                                " must be > 0 (missing or bad value)");
  }
}

void positive(int v, const char* key) {
  if (v <= 0) {
    throw std::invalid_argument(std::string("localization.yaml: ") + key +
                                " must be >= 1 (missing or bad value)");
  }
}

}  // namespace

void validate(const LocalizerConfig& c) {
  // The filter rejects a missing noise model, sigmas <= 0, gate <= 0.
  eskf::Eskf check(c.eskf);
  positive(c.eskf.gate_pose,
           "eskf.gate_pose");  // the Eskf accepts 0, we do not
  positive(c.gnss_default_sigma_m, "gnss_default_sigma_m");
  if (!std::isfinite(c.extrinsics.imu_time_offset_s)) {
    throw std::invalid_argument(
        "localization.yaml: imu_time_offset_s must be finite");
  }

  if (c.points.time_field.empty()) {
    throw std::invalid_argument(
        "localization.yaml: points.time_field is empty");
  }
  positive(c.points.time_scale, "points.time_scale");
  if (c.points.stamp_is != "start" && c.points.stamp_is != "end") {
    throw std::invalid_argument(
        "localization.yaml: points.stamp_is must be 'start' or 'end', got '" +
        c.points.stamp_is + "'");
  }

  const auto& m = c.matcher;
  positive(m.map_voxel_m, "matcher.map_voxel_m");
  positive(m.coarse_voxel_m, "matcher.coarse_voxel_m");
  positive(m.source_leaf_m, "matcher.source_leaf_m");
  positive(m.max_corr_dist_m, "matcher.max_corr_dist_m");
  positive(m.coarse_max_corr_dist_m, "matcher.coarse_max_corr_dist_m");
  positive(m.max_iterations, "matcher.max_iterations");
  positive(m.num_threads, "matcher.num_threads");
  positive(m.cov_neighbors, "matcher.cov_neighbors");
  positive(m.min_range_m, "matcher.min_range_m");
  positive(m.max_range_m, "matcher.max_range_m");
  if (m.max_range_m <= m.min_range_m) {
    throw std::invalid_argument(
        "localization.yaml: matcher.max_range_m must exceed "
        "matcher.min_range_m");
  }

  positive(c.meas.sigma_pos_m, "meas.sigma_pos_m");
  positive(c.meas.sigma_rot_rad, "meas.sigma_rot_rad");

  positive(c.accept.min_source_points, "accept.min_source_points");
  positive(c.accept.min_inlier_ratio, "accept.min_inlier_ratio");
  positive(c.accept.max_error_per_inlier, "accept.max_error_per_inlier");
  positive(c.accept.max_jump_m, "accept.max_jump_m");
  positive(c.accept.max_jump_rad, "accept.max_jump_rad");
  positive(c.accept.max_age_s, "accept.max_age_s");

  positive(c.scan.imu_wait_s, "scan.imu_wait_s");
  positive(c.history.keep_s, "history.keep_s");

  const auto& i = c.init;
  positive(i.yaw_steps, "init.yaw_steps");
  positive(i.refine_top, "init.refine_top");
  if (i.refine_top > i.yaw_steps) {
    throw std::invalid_argument(
        "localization.yaml: init.refine_top must not exceed init.yaw_steps");
  }
  positive(i.margin, "init.margin");
  positive(i.confirm_n, "init.confirm_n");
  positive(i.timeout_s, "init.timeout_s");
  positive(i.seed_gnss_max_age_s, "init.seed_gnss_max_age_s");
  positive(i.max_seed_gnss_sigma_m, "init.max_seed_gnss_sigma_m");
  positive(i.vel_window_s, "init.vel_window_s");
  positive(i.vel_min_fixes, "init.vel_min_fixes");
  if (i.vel_min_fixes < 2) {
    throw std::invalid_argument(
        "localization.yaml: init.vel_min_fixes must be >= 2");
  }
  positive(i.vel_min_span_s, "init.vel_min_span_s");
  if (i.vel_min_span_s > i.vel_window_s) {
    throw std::invalid_argument(
        "localization.yaml: init.vel_min_span_s must not exceed "
        "init.vel_window_s");
  }
  positive(i.vel_sigma_floor_mps, "init.vel_sigma_floor_mps");
  positive(i.still_window_s, "init.still_window_s");
  positive(i.still_max_gyro_radps, "init.still_max_gyro_radps");
  positive(i.still_max_acc_std_mps2, "init.still_max_acc_std_mps2");
  positive(i.still_max_acc_mean_dev_mps2, "init.still_max_acc_mean_dev_mps2");
  positive(i.still_max_gnss_speed_mps, "init.still_max_gnss_speed_mps");
  positive(i.still_gnss_speed_sigmas, "init.still_gnss_speed_sigmas");
  positive(i.cog_min_speed_mps, "init.cog_min_speed_mps");
  positive(i.cog_check_min_speed_mps, "init.cog_check_min_speed_mps");
  if (i.cog_check_min_speed_mps < i.cog_min_speed_mps) {
    throw std::invalid_argument(
        "localization.yaml: init.cog_check_min_speed_mps must not be below "
        "init.cog_min_speed_mps");
  }
  positive(i.cog_max_diff_deg, "init.cog_max_diff_deg");
  positive(i.carry_bg_max_radps, "init.carry_bg_max_radps");
  positive(i.p0_sigma_pos_m, "init.p0_sigma_pos_m");
  positive(i.p0_sigma_yaw_deg, "init.p0_sigma_yaw_deg");
  positive(i.p0_sigma_rollpitch_deg, "init.p0_sigma_rollpitch_deg");
  positive(i.p0_sigma_vel_mps, "init.p0_sigma_vel_mps");
  positive(i.p0_sigma_vel_moving_mps, "init.p0_sigma_vel_moving_mps");
  positive(i.p0_sigma_ba, "init.p0_sigma_ba");
  positive(i.p0_sigma_bg, "init.p0_sigma_bg");

  const auto& h = c.health;
  positive(h.scan_timeout_s, "health.scan_timeout_s");
  positive(h.gnss_timeout_s, "health.gnss_timeout_s");
  positive(h.dr_timeout_s, "health.dr_timeout_s");
  positive(h.degraded_sigma_m, "health.degraded_sigma_m");
  positive(h.fail_consecutive_gated, "health.fail_consecutive_gated");
  positive(h.fail_timeout_s, "health.fail_timeout_s");
  positive(h.fail_sigma_m, "health.fail_sigma_m");
  if (h.fail_sigma_m <= h.degraded_sigma_m) {
    throw std::invalid_argument(
        "localization.yaml: health.fail_sigma_m must exceed "
        "health.degraded_sigma_m");
  }
}

}  // namespace brightdrive_localization

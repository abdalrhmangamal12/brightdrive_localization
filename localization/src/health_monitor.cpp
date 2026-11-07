#include "brightdrive_localization/health_monitor.hpp"

#include <algorithm>
#include <cstdio>

namespace brightdrive_localization {

HealthMonitor::HealthMonitor(const HealthConfig& p) : p_(p) {}

void HealthMonitor::reset(double t) {
  last_lidar_ = t;
  last_gnss_ = t;
  consecutive_gated_ = 0;
}

// Estimator core (the gate lock-out rule): consecutive gated updates of
// either source, reset by any accepted one. A filter whose covariance has
// collapsed around a wrong state gates everything it is shown; quality
// rejects (bad matches) say nothing about the filter and do not count.
void HealthMonitor::onUpdate(const UpdateRecord& r) {
  switch (r.outcome) {
    case UpdateRecord::Outcome::kAccepted:
      consecutive_gated_ = 0;
      if (r.source == UpdateRecord::Source::kLidar) {
        last_lidar_ = std::max(last_lidar_, r.t);
      } else {
        last_gnss_ = std::max(last_gnss_, r.t);
      }
      break;
    case UpdateRecord::Outcome::kGated:
      ++consecutive_gated_;
      break;
    case UpdateRecord::Outcome::kQualityReject:
      break;
  }
}

HealthEval HealthMonitor::evaluate(double now, double sigma_pos_m,
                                   bool finite) const {
  HealthEval e;
  char buf[128];
  const double last_any = std::max(last_lidar_, last_gnss_);
  if (!finite) {
    e.health = Health::kFailed;
    e.fail_reason = "non-finite state";
    return e;
  }
  if (consecutive_gated_ >= p_.fail_consecutive_gated) {
    std::snprintf(buf, sizeof(buf), "gate lock-out: %d gated updates in a row",
                  consecutive_gated_);
    e.health = Health::kFailed;
    e.fail_reason = buf;
    return e;
  }
  if (now - last_any >= p_.fail_timeout_s) {
    std::snprintf(buf, sizeof(buf), "no accepted update for %.1f s",
                  now - last_any);
    e.health = Health::kFailed;
    e.fail_reason = buf;
    return e;
  }
  if (sigma_pos_m >= p_.fail_sigma_m) {
    std::snprintf(buf, sizeof(buf), "sigma_pos %.2f m", sigma_pos_m);
    e.health = Health::kFailed;
    e.fail_reason = buf;
    return e;
  }
  const bool lidar = now - last_lidar_ <= p_.scan_timeout_s;
  const bool gnss = now - last_gnss_ <= p_.gnss_timeout_s;
  if (lidar && gnss && sigma_pos_m < p_.degraded_sigma_m) {
    e.health = Health::kNominal;
  } else if (lidar || gnss) {
    e.health = Health::kDegraded;  // one source alive, or sigma too large
  } else if (now - last_any > p_.dr_timeout_s) {
    e.health = Health::kDeadReckoning;
  } else {
    e.health = Health::kDegraded;
  }
  return e;
}

}  // namespace brightdrive_localization

// One health level from the recent update record: which absolute sources
// (LiDAR matches, GNSS fixes) were accepted lately, the position sigma, and
// the gate lock-out rule. FAILED sends the localizer back to initialization.
#pragma once

#include <string>

#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

struct UpdateRecord {
  enum class Source { kLidar, kGnss };
  // kQualityReject: a match that failed a check before the filter (feeds the
  // timeouts only). kGated: reached the chi-square gate and was rejected.
  enum class Outcome { kAccepted, kGated, kQualityReject };
  double t = 0.0;  // filter time [s]
  Source source = Source::kLidar;
  Outcome outcome = Outcome::kAccepted;
};

struct HealthEval {
  Health health = Health::kNominal;
  std::string fail_reason;  // set when health == kFailed
};

class HealthMonitor {
 public:
  explicit HealthMonitor(const HealthConfig& p);

  // A (re)initialized filter at time t: the timers start there, so a young
  // filter is not dead reckoning before its first update.
  void reset(double t);
  void onUpdate(const UpdateRecord& r);
  // now = filter time; finite = the state and covariance are finite.
  HealthEval evaluate(double now, double sigma_pos_m, bool finite) const;

  int consecutiveGated() const { return consecutive_gated_; }
  double lidarAge(double now) const { return now - last_lidar_; }
  double gnssAge(double now) const { return now - last_gnss_; }

 private:
  HealthConfig p_;
  double last_lidar_ = 0.0;  // last accepted match (filter time)
  double last_gnss_ = 0.0;   // last accepted fix
  int consecutive_gated_ = 0;
};

}  // namespace brightdrive_localization

// The filter's recent past: its IMU poses over the last keep_s seconds, and
// the raw IMU samples and GNSS fixes of the same span. The poses serve deskew,
// moving a late measurement to the filter's time, and the TF-free pose at any
// recent time; the events let a freshly initialized filter catch up from the
// scan it was initialized on to now. Not thread-safe: Localizer holds it
// under its one mutex.
#pragma once

#include <Eigen/Geometry>
#include <deque>
#include <optional>
#include <vector>

#include "brightdrive_localization/types.hpp"
#include "eskf/types.hpp"

namespace brightdrive_localization {

// One raw input, kept for the initializer's catch-up.
struct Event {
  enum class Kind { kImu, kGnss };
  double t = 0.0;
  Kind kind = Kind::kImu;
  eskf::ImuSample imu;  // kind == kImu
  eskf::GnssMeas gnss;  // kind == kGnss: IMU position in map, V [m^2]
};

class StateHistory {
 public:
  // Throws std::invalid_argument when keep_s <= 0.
  explicit StateHistory(double keep_s);

  // The filter's pose after a predict at time t. A sample at the time of the
  // newest one replaces it; an older one is ignored (returns false).
  bool pushPose(double t, const eskf::NominalState& x);
  // Raw inputs, kept in time order even when a fix arrives after newer IMU
  // samples, whether the filter is initialized or not.
  void pushEvent(const Event& e);

  // Interpolated pose at t; nullopt outside [oldest, newest] of the poses.
  std::optional<Eigen::Isometry3d> poseAt(double t) const;
  // The stored poses that cover [t0, t1]: every sample inside plus the one at
  // or before t0 and the one at or after t1, so a caller can interpolate
  // anywhere in the span. Empty when the poses do not cover it.
  std::vector<PoseSample> posesCovering(double t0, double t1) const;

  // After a correction that moved the newest pose by J (map frame,
  // T_after = J * T_before): T <- J * T for every pose at or after t, so a
  // later measurement older than the correction is forwarded through motion
  // that already contains the jump.
  void patchFrom(double t, const Eigen::Isometry3d& J);

  // Events strictly after t, oldest first.
  std::vector<Event> eventsAfter(double t) const;

  void clearPoses();  // a re-initialized filter has a new past
  void clear();       // poses and events
  bool hasPoses() const { return !poses_.empty(); }
  double oldestPose() const;  // pose times; call only when hasPoses()
  double newestPose() const;
  size_t numPoses() const { return poses_.size(); }
  size_t numEvents() const { return events_.size(); }

 private:
  void trim();

  double keep_s_;
  std::deque<PoseSample> poses_;  // time order
  std::deque<Event> events_;      // time order
};

}  // namespace brightdrive_localization

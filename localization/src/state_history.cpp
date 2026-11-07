#include "brightdrive_localization/state_history.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace brightdrive_localization {

namespace {

// Two stamps closer than this are the same instant (stamps are ns integers
// turned into double seconds).
constexpr double kSameTime = 1e-9;

}  // namespace

StateHistory::StateHistory(double keep_s) : keep_s_(keep_s) {
  if (!(keep_s > 0.0)) {
    throw std::invalid_argument("StateHistory: keep_s must be > 0, got " +
                                std::to_string(keep_s));
  }
}

bool StateHistory::pushPose(double t, const eskf::NominalState& x) {
  PoseSample s;
  s.t = t;
  s.T_map_imu.linear() = x.q.normalized().toRotationMatrix();
  s.T_map_imu.translation() = x.p;
  if (!poses_.empty()) {
    const double newest = poses_.back().t;
    if (std::abs(t - newest) < kSameTime) {
      poses_.back() = s;
      return true;
    }
    if (t < newest) return false;
  }
  poses_.push_back(s);
  trim();
  return true;
}

void StateHistory::pushEvent(const Event& e) {
  // Usually the newest; a late GNSS fix walks back a few IMU samples.
  auto it = events_.end();
  while (it != events_.begin() && std::prev(it)->t > e.t) --it;
  events_.insert(it, e);
  trim();
}

void StateHistory::trim() {
  // Keep one pose at or before newest - keep_s so the whole window can still
  // be interpolated.
  if (!poses_.empty()) {
    const double cut = poses_.back().t - keep_s_;
    while (poses_.size() >= 2 && poses_[1].t <= cut) poses_.pop_front();
  }
  if (!events_.empty()) {
    const double cut = events_.back().t - keep_s_;
    while (!events_.empty() && events_.front().t < cut) events_.pop_front();
  }
}

std::optional<Eigen::Isometry3d> StateHistory::poseAt(double t) const {
  if (poses_.empty() || t < poses_.front().t - kSameTime ||
      t > poses_.back().t + kSameTime) {
    return std::nullopt;
  }
  // First sample with time >= t.
  auto hi =
      std::lower_bound(poses_.begin(), poses_.end(), t - kSameTime,
                       [](const PoseSample& s, double v) { return s.t < v; });
  if (hi == poses_.end()) return poses_.back().T_map_imu;
  if (hi == poses_.begin() || std::abs(hi->t - t) < kSameTime) {
    return hi->T_map_imu;
  }
  return interpolatePose(*std::prev(hi), *hi, t);
}

std::vector<PoseSample> StateHistory::posesCovering(double t0,
                                                    double t1) const {
  if (poses_.empty() || t1 < t0 || t0 < poses_.front().t - kSameTime ||
      t1 > poses_.back().t + kSameTime) {
    return {};
  }
  // Last sample at or before t0 ... first sample at or after t1.
  auto first =
      std::upper_bound(poses_.begin(), poses_.end(), t0 + kSameTime,
                       [](double v, const PoseSample& s) { return v < s.t; });
  if (first != poses_.begin()) --first;
  auto last =
      std::lower_bound(poses_.begin(), poses_.end(), t1 - kSameTime,
                       [](const PoseSample& s, double v) { return s.t < v; });
  if (last == poses_.end()) --last;
  return std::vector<PoseSample>(first, std::next(last));
}

void StateHistory::patchFrom(double t, const Eigen::Isometry3d& J) {
  for (auto it = poses_.rbegin(); it != poses_.rend() && it->t >= t - kSameTime;
       ++it) {
    it->T_map_imu = J * it->T_map_imu;
    // Keep the rotation orthonormal after repeated patches.
    it->T_map_imu.linear() = Eigen::Quaterniond(it->T_map_imu.linear())
                                 .normalized()
                                 .toRotationMatrix();
  }
}

std::vector<Event> StateHistory::eventsAfter(double t) const {
  auto first =
      std::upper_bound(events_.begin(), events_.end(), t,
                       [](double v, const Event& e) { return v < e.t; });
  return std::vector<Event>(first, events_.end());
}

void StateHistory::clearPoses() { poses_.clear(); }

void StateHistory::clear() {
  poses_.clear();
  events_.clear();
}

double StateHistory::oldestPose() const { return poses_.front().t; }
double StateHistory::newestPose() const { return poses_.back().t; }
double StateHistory::oldestEvent() const { return events_.front().t; }

}  // namespace brightdrive_localization

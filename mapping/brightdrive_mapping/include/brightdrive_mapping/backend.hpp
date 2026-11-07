// The mapping backend: turns paired scans, odometry and GNSS fixes into
// keyframes, graph factors and the map. Owns all state and the one mutex.
// Design and call sequences: mapping/brightdrive_mapping/README.md.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "brightdrive_mapping/params.hpp"
#include "brightdrive_mapping/pose_graph.hpp"
#include "brightdrive_mapping/types.hpp"

namespace brightdrive_mapping {

class Backend {
 public:
  // Throws std::invalid_argument on a bad config: a value <= 0, a sigma <= 0,
  // max_range <= min_range, an empty frame or map name.
  explicit Backend(const BackendConfig& cfg);

  // Every front-end pose, keyframe or not: the history interpolate() reads.
  // t [s] is the message stamp.
  void addOdometry(double t, const Eigen::Isometry3d& T_odom);
  // A body-frame scan with the odometry pose of the same stamp. Returns true
  // when it became a keyframe.
  bool processScan(double t, const Eigen::Isometry3d& T_odom,
                   const CloudPtr& cloud);
  // A fix already in map metres. Buffered until a keyframe is near enough in
  // time (attachGnss) or, before alignment, until the path is long enough
  // (tryAlign). A zero cov is replaced by diag(gnss_default_sigma^2).
  void addGnssFix(const GnssFix& fix);
  // One loop-closure attempt on the latest keyframe; ICP runs outside the
  // lock. Returns true when a loop factor was added.
  bool tryLoopClosure();

  // map_frame -> origin_frame = T_mo * T_corr. Empty while GNSS is enabled
  // and T_mo is not yet known; with GNSS disabled T_mo = I.
  std::optional<Eigen::Isometry3d> mapToOdomTransform() const;
  // Optimized keyframe poses in the map frame, T_mo * X_k. Empty under the
  // same condition as mapToOdomTransform().
  std::vector<Eigen::Isometry3d> keyframePathInMap() const;
  // Every keyframe cloud at its optimized map pose, voxelled at leaf [m].
  // Empty under the same condition as mapToOdomTransform().
  CloudPtr buildGlobalMap(double leaf) const;
  // Batch solve, then <map_name>.pcd, trajectory_optimized_tum.txt,
  // trajectory_fastlio_tum.txt (the raw front-end poses), map.yaml and
  // loops.txt in dir. Before alignment the files are written in origin_frame
  // and map.yaml says so. False when there is no keyframe yet; throws
  // std::runtime_error when a file cannot be written.
  bool saveMap(const std::string& dir, const MapSource& source);

  bool aligned() const;
  int numKeyframes() const;
  int numLoops() const;
  // Loop candidates ICP did not confirm (not converged or fitness too high).
  int numLoopsRejected() const;

 private:
  // What the loop thread copies under the lock and works on without it.
  struct LoopSnapshot {
    int cur = 0;
    std::vector<double> times;
    std::vector<Eigen::Isometry3d> poses;  // optimized, graph frame
    std::vector<CloudConstPtr> clouds;     // body frame
  };

  // Caller holds mtx_ for all of these.
  bool isKeyframe(const Eigen::Isometry3d& T_odom) const;
  CloudPtr preprocess(const CloudPtr& cloud) const;
  // Odometry pose at t: linear in translation, slerp in rotation between the
  // two history poses around t; clamped at the ends of the history.
  Eigen::Isometry3d interpolate(double t) const;
  // First T_mo from geo::align4Dof over the buffered fixes, G(0) into the
  // graph, every buffered fix attached, one update with extra iterations.
  void tryAlign();
  // The fix becomes a factor on the keyframe nearest in time (within
  // max_time_shift, else dropped), moved to that keyframe's time.
  void attachGnss(const GnssFix& fix);
  // Attaches every buffered fix not newer than t_latest and drops them from
  // the buffer. Fixes after the latest keyframe wait for the next one, which
  // may be nearer in time.
  void attachBufferedUpTo(double t_latest);

  // Loop thread only, on the snapshot.
  std::optional<int> findLoopCandidate(const LoopSnapshot& s) const;
  bool matchScans(const LoopSnapshot& s, int prev, Eigen::Isometry3d& T_rel,
                  double& fitness) const;

  BackendConfig cfg_;
  PoseGraph graph_;
  std::vector<Keyframe> keyframes_;
  std::vector<LoopEdge> loops_;
  std::map<double, Eigen::Isometry3d> odom_hist_;  // the whole run
  std::vector<GnssFix> gnss_buf_;                  // waiting for a keyframe
  // Graph frame <- odometry frame of the front-end: X_k * T_odom,k^-1 of the
  // latest keyframe, refreshed after every update.
  Eigen::Isometry3d T_corr_ = Eigen::Isometry3d::Identity();
  double path_len_ = 0.0;  // keyframe-to-keyframe odometry distance [m]
  bool aligned_ = false;
  double align_time_ = 0.0;  // keyframe stamp at which T_mo was first set
  // Attached fixes (keyframe, z_k) and the dropped count, for map.yaml.
  std::vector<std::pair<int, Eigen::Vector3d>> gnss_attached_;
  int n_gnss_dropped_ = 0;
  int n_loops_rejected_ = 0;
  std::set<int> loop_checked_;  // loop thread only, no lock
  mutable std::mutex mtx_;
};

}  // namespace brightdrive_mapping

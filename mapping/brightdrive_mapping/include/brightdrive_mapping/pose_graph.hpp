// The GTSAM pose graph behind the backend: keyframe nodes X(k), the map-origin
// node G(0) = T_mo, prior / odometry / loop / GNSS factors, iSAM2 updates and
// the batch solve for saveMap. The only place GTSAM types appear; this header
// has no GTSAM include (pimpl).
// Not thread-safe: Backend holds its mutex around every call.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

#include "brightdrive_mapping/types.hpp"  // Vector6d

namespace brightdrive_mapping {

class PoseGraph {
 public:
  // ISAM2Params: relinearize threshold and skip (0.1 and 1 in the design).
  PoseGraph(double relin_threshold, int relin_skip);
  ~PoseGraph();  // in the .cpp, where Impl is complete

  // *Node adds a value with its initial guess, *Factor adds an edge. Every
  // sigmas vector is in GTSAM order: rot x, y, z [rad] then trans x, y, z [m].
  void addKeyframeNode(int k, const Eigen::Isometry3d& guess);
  void addPriorFactor(int k, const Eigen::Isometry3d& T,
                      const Vector6d& sigmas);
  // T_rel = pose of `to` expressed in the frame of `from`.
  void addOdomFactor(int from, int to, const Eigen::Isometry3d& T_rel,
                     const Vector6d& sigmas);
  // Same measurement as addOdomFactor under a Cauchy(1.0) kernel, so one
  // wrong loop cannot bend the map against the odometry chain.
  void addLoopFactor(int prev, int cur, const Eigen::Isometry3d& T_rel,
                     const Vector6d& sigmas);
  // G(0) = T_mo together with its (weak) prior. Throws std::logic_error when
  // called twice.
  void addMapOriginNode(const Eigen::Isometry3d& T_mo0, const Vector6d& sigmas);
  // Antenna position in map: T_mo * X_k * lever_arm = p_map, Gaussian(cov)
  // under Huber(huber_k). Throws std::logic_error before addMapOriginNode.
  void addGnssFactor(int k, const Eigen::Vector3d& p_map,
                     const Eigen::Matrix3d& cov,
                     const Eigen::Vector3d& lever_arm, double huber_k);

  // Push everything added since the last call into iSAM2, run
  // extra_iterations more passes (5 after a loop closure), refresh the
  // estimate.
  void update(int extra_iterations = 0);
  // Levenberg-Marquardt over every factor ever added, from the current
  // estimate; the result replaces the estimate. For saveMap only.
  void batchOptimize();

  // The optimized pose, or the initial guess if k was added but not yet
  // optimized. Throws std::out_of_range for an unknown k.
  Eigen::Isometry3d keyframePose(int k) const;
  std::vector<Eigen::Isometry3d> keyframePoses() const;  // index == k
  std::optional<Eigen::Isometry3d> mapOrigin() const;    // empty before G(0)
  int numKeyframes() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace brightdrive_mapping

// VGICP scan-to-map matching with small_gicp. The map is turned once into two
// Gaussian voxel maps (fine for tracking, coarse for the initializer's yaw
// grid); a sweep is preprocessed once (range crop, voxel downsample, per-point
// covariances) and aligned from any number of initial guesses.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>  // before small_gicp: incremental_voxelmap.hpp uses std::shared_ptr without including it
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <vector>

#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

class ScanMatcher {
 public:
  enum class Level { kFine, kCoarse };

  // What setMap cost and produced.
  struct MapStats {
    size_t points = 0;
    size_t voxels_fine = 0;
    size_t voxels_coarse = 0;
    double covariance_s = 0.0;  // KdTree + per-point covariances
    double voxelmap_s = 0.0;    // both voxel maps
  };

  explicit ScanMatcher(const MatcherConfig& p);

  // Per-point covariances from cov_neighbors over the whole map, then the
  // fine and coarse voxel maps. The map cloud is released afterwards.
  void setMap(const std::vector<Eigen::Vector4d>& map);
  bool hasMap() const { return fine_ != nullptr; }
  const MapStats& mapStats() const { return stats_; }

  // Range crop, voxel downsample (deterministic, single thread), covariances.
  small_gicp::PointCloud::Ptr preprocess(const Scan& scan) const;

  // T_init and the result are T_map_lidar. Const and without shared mutable
  // state, so any thread may call it while the node's lock is free.
  MatchResult align(const small_gicp::PointCloud& source,
                    const Eigen::Isometry3d& T_init,
                    Level level = Level::kFine) const;

  const MatcherConfig& config() const { return p_; }

 private:
  MatcherConfig p_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> fine_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> coarse_;
  MapStats stats_;
};

}  // namespace brightdrive_localization

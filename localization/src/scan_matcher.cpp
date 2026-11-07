#include "brightdrive_localization/scan_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <stdexcept>

namespace brightdrive_localization {

namespace {

using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point from) {
  return std::chrono::duration<double>(Clock::now() - from).count();
}

// One insert() per map; the LRU sweep of IncrementalVoxelMap must never run.
std::shared_ptr<small_gicp::GaussianVoxelMap> buildVoxelMap(
    const small_gicp::PointCloud& cloud, double voxel_m) {
  auto map = std::make_shared<small_gicp::GaussianVoxelMap>(voxel_m);
  map->lru_horizon = std::numeric_limits<size_t>::max() / 2;
  map->insert(cloud);
  return map;
}

}  // namespace

ScanMatcher::ScanMatcher(const MatcherConfig& p) : p_(p) {}

void ScanMatcher::setMap(const std::vector<Eigen::Vector4d>& map) {
  if (map.size() < 100) {
    throw std::invalid_argument("ScanMatcher::setMap: the map has " +
                                std::to_string(map.size()) + " points");
  }
  stats_ = MapStats{};
  stats_.points = map.size();

  auto t0 = Clock::now();
  auto cloud = std::make_shared<small_gicp::PointCloud>(map);
  small_gicp::KdTree<small_gicp::PointCloud> tree(
      cloud, small_gicp::KdTreeBuilderOMP(p_.num_threads));
  small_gicp::estimate_covariances_omp(*cloud, tree, p_.cov_neighbors,
                                       p_.num_threads);
  stats_.covariance_s = seconds(t0);

  t0 = Clock::now();
  fine_ = buildVoxelMap(*cloud, p_.map_voxel_m);
  coarse_ = buildVoxelMap(*cloud, p_.coarse_voxel_m);
  stats_.voxelmap_s = seconds(t0);
  stats_.voxels_fine = fine_->size();
  stats_.voxels_coarse = coarse_->size();
}

small_gicp::PointCloud::Ptr ScanMatcher::preprocess(const Scan& scan) const {
  const double min_sq = p_.min_range_m * p_.min_range_m;
  const double max_sq = p_.max_range_m * p_.max_range_m;
  std::vector<Eigen::Vector4d> kept;
  kept.reserve(scan.points.size());
  for (const auto& p : scan.points) {
    const double r_sq = p.head<3>().squaredNorm();
    if (r_sq >= min_sq && r_sq <= max_sq && std::isfinite(r_sq)) {
      kept.push_back(p);
    }
  }
  small_gicp::PointCloud raw(kept);
  auto down = small_gicp::voxelgrid_sampling(raw, p_.source_leaf_m);
  if (down->size() >= 5) {
    small_gicp::KdTree<small_gicp::PointCloud> tree(
        down, small_gicp::KdTreeBuilderOMP(p_.num_threads));
    small_gicp::estimate_covariances_omp(*down, tree, p_.cov_neighbors,
                                         p_.num_threads);
  }
  return down;
}

MatchResult ScanMatcher::align(const small_gicp::PointCloud& source,
                               const Eigen::Isometry3d& T_init,
                               Level level) const {
  if (!hasMap()) {
    throw std::logic_error("ScanMatcher::align before setMap");
  }
  const bool fine = level == Level::kFine;
  const auto& target = fine ? *fine_ : *coarse_;
  const double max_corr = fine ? p_.max_corr_dist_m : p_.coarse_max_corr_dist_m;

  small_gicp::Registration<small_gicp::GICPFactor,
                           small_gicp::ParallelReductionOMP>
      reg;
  reg.reduction.num_threads = p_.num_threads;
  reg.rejector.max_dist_sq = max_corr * max_corr;
  reg.optimizer.max_iterations = p_.max_iterations;

  const auto t0 = Clock::now();
  const small_gicp::RegistrationResult r =
      reg.align(target, source, target, T_init);
  MatchResult m;
  m.time_ms = seconds(t0) * 1e3;
  m.converged = r.converged;
  m.iterations = static_cast<int>(r.iterations);
  m.num_inliers = static_cast<int>(r.num_inliers);
  m.num_source = static_cast<int>(source.size());
  m.error = r.error;
  m.T_map_lidar = r.T_target_source;
  m.H = r.H;
  return m;
}

}  // namespace brightdrive_localization

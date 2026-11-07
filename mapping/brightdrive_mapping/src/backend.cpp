// Backend implementation. Call sequences: README.md ("One keyframe", "One
// loop step", "Save").
#include "brightdrive_mapping/backend.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "brightdrive_mapping/geo.hpp"

namespace brightdrive_mapping {

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// ISAM2Params of the design: relinearize when a delta exceeds 0.1, every
// update.
constexpr double kRelinearizeThreshold = 0.1;
constexpr int kRelinearizeSkip = 1;
// iSAM2 passes after a change that moves many nodes at once (alignment, loop).
constexpr int kExtraIterations = 5;
// ICP settings, the LIO-SAM values.
constexpr int kIcpMaxIterations = 100;
constexpr double kIcpEpsilon = 1e-6;
// Below these sizes a scan pair cannot be trusted to converge.
constexpr size_t kIcpMinSourcePoints = 300;
constexpr size_t kIcpMinTargetPoints = 1000;
// Floor on the fitness that scales the loop sigma: a perfect ICP score must
// not give a zero sigma.
constexpr double kMinLoopFitness = 1e-4;

// ---- config -------------------------------------------------------------

void requirePositive(double value, const char* name) {
  if (!(value > 0.0)) {  // !(x > 0) also rejects NaN
    throw std::invalid_argument(std::string("BackendConfig::") + name +
                                " must be > 0");
  }
}

void requirePositive(const Vector6d& sigmas, const char* name) {
  for (int i = 0; i < 6; ++i) requirePositive(sigmas[i], name);
}

// !(lo <= value <= hi) also rejects NaN.
void requireWithin(double value, double lo, double hi, const char* name) {
  if (!(value >= lo && value <= hi)) {
    throw std::invalid_argument(
        std::string("BackendConfig::") + name + " must be finite and within [" +
        std::to_string(lo) + ", " + std::to_string(hi) + "]");
  }
}

void requireNonEmpty(const std::string& value, const char* name) {
  if (value.empty()) {
    throw std::invalid_argument(std::string("BackendConfig::") + name +
                                " must not be empty");
  }
}

void validate(const BackendConfig& cfg) {
  if (!cfg.lever_arm.allFinite()) {
    throw std::invalid_argument("BackendConfig::lever_arm has NaN or Inf");
  }
  requirePositive(cfg.kf_dist, "kf_dist");
  requirePositive(cfg.kf_angle, "kf_angle");
  requirePositive(cfg.kf_leaf, "kf_leaf");
  requirePositive(cfg.min_range, "min_range");
  requirePositive(cfg.max_range, "max_range");
  if (!(cfg.max_range > cfg.min_range)) {
    throw std::invalid_argument("BackendConfig::max_range must be > min_range");
  }
  requirePositive(cfg.prior_sigma, "prior_sigma");
  requirePositive(cfg.odom_sigma, "odom_sigma");
  requirePositive(cfg.origin_sigma, "origin_sigma");
  requirePositive(cfg.align_distance, "align_distance");
  requirePositive(cfg.max_time_shift, "max_time_shift");
  requirePositive(cfg.gnss_huber_k, "gnss_huber_k");
  requirePositive(cfg.gnss_default_sigma, "gnss_default_sigma");
  // Georeference: the origin may be 0, so range checks instead of "> 0".
  // geo::parseProjection names an unknown projection itself.
  geo::parseProjection(cfg.geo_projection);
  requireWithin(cfg.geo_origin_lat_deg, -90.0, 90.0, "geo_origin_lat_deg");
  requireWithin(cfg.geo_origin_lon_deg, -180.0, 180.0, "geo_origin_lon_deg");
  if (!std::isfinite(cfg.geo_origin_alt_m)) {
    throw std::invalid_argument(
        "BackendConfig::geo_origin_alt_m must be finite");
  }
  requirePositive(cfg.loop_radius, "loop_radius");
  requirePositive(cfg.loop_time_diff, "loop_time_diff");
  if (cfg.loop_search_num < 0) {
    throw std::invalid_argument("BackendConfig::loop_search_num must be >= 0");
  }
  requirePositive(cfg.loop_icp_leaf, "loop_icp_leaf");
  requirePositive(cfg.loop_fitness, "loop_fitness");
  requirePositive(cfg.loop_rot_scale, "loop_rot_scale");
  requirePositive(cfg.loop_trans_scale, "loop_trans_scale");
  requireNonEmpty(cfg.map_frame, "map_frame");
  requireNonEmpty(cfg.origin_frame, "origin_frame");
  requireNonEmpty(cfg.map_name, "map_name");
  requirePositive(cfg.save_resolution, "save_resolution");
}

// ---- clouds -------------------------------------------------------------

// Centroid per voxel. Not pcl::VoxelGrid: it numbers voxels with a 32-bit
// int and, when a town-sized map at 0.2 m overflows that, warns and returns
// the input unfiltered. Here each axis gets 21 bits of a 64-bit key, offset
// so negative coordinates stay positive: +-200 km at 0.2 m.
CloudPtr voxelDownsample(const Cloud& in, double leaf) {
  struct Cell {
    double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
    uint32_t n = 0;
  };
  constexpr int64_t kOffset = int64_t{1} << 20;
  constexpr int64_t kMask = (int64_t{1} << 21) - 1;
  const double inv = 1.0 / leaf;
  std::unordered_map<uint64_t, Cell> cells;
  cells.reserve(in.size() / 2);
  for (const auto& p : in.points) {
    const int64_t ix = static_cast<int64_t>(std::floor(p.x * inv)) + kOffset;
    const int64_t iy = static_cast<int64_t>(std::floor(p.y * inv)) + kOffset;
    const int64_t iz = static_cast<int64_t>(std::floor(p.z * inv)) + kOffset;
    const uint64_t key = (static_cast<uint64_t>(ix & kMask) << 42) |
                         (static_cast<uint64_t>(iy & kMask) << 21) |
                         static_cast<uint64_t>(iz & kMask);
    Cell& c = cells[key];
    c.x += p.x;
    c.y += p.y;
    c.z += p.z;
    c.intensity += p.intensity;
    ++c.n;
  }
  CloudPtr out(new Cloud);
  out->reserve(cells.size());
  for (const auto& kv : cells) {
    const Cell& c = kv.second;
    pcl::PointXYZI q;
    q.x = static_cast<float>(c.x / c.n);
    q.y = static_cast<float>(c.y / c.n);
    q.z = static_cast<float>(c.z / c.n);
    q.intensity = static_cast<float>(c.intensity / c.n);
    out->push_back(q);
  }
  out->is_dense = true;
  return out;
}

// Points with min_range <= |p| <= max_range, finite only.
CloudPtr rangeCrop(const Cloud& in, double min_range, double max_range) {
  CloudPtr out(new Cloud);
  out->reserve(in.size());
  const float min2 = static_cast<float>(min_range * min_range);
  const float max2 = static_cast<float>(max_range * max_range);
  for (const auto& p : in.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 < min2 || r2 > max2) continue;
    out->push_back(p);
  }
  out->is_dense = true;
  return out;
}

// cloud at pose T, appended to sum.
void appendTransformed(const Cloud& cloud, const Eigen::Isometry3d& T,
                       Cloud& sum) {
  Cloud moved;
  pcl::transformPointCloud(cloud, moved, T.matrix().cast<float>());
  sum += moved;
}

// Keyframes [centre - num, centre + num] of the snapshot at their poses,
// voxelled at leaf.
CloudPtr buildSubmap(const std::vector<Eigen::Isometry3d>& poses,
                     const std::vector<CloudConstPtr>& clouds, int centre,
                     int num, double leaf) {
  Cloud sum;
  const int n = static_cast<int>(poses.size());
  for (int i = std::max(0, centre - num); i <= std::min(n - 1, centre + num);
       ++i) {
    appendTransformed(*clouds[i], poses[i], sum);
  }
  return voxelDownsample(sum, leaf);
}

// Point-to-point ICP of source onto target. On success T_icp maps source
// points onto the target and fitness is the mean squared distance [m^2] of
// the matched points.
bool icpAlign(const CloudPtr& source, const CloudPtr& target,
              double max_correspondence, Eigen::Isometry3d& T_icp,
              double& fitness) {
  if (source->size() < kIcpMinSourcePoints ||
      target->size() < kIcpMinTargetPoints) {
    return false;
  }
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(max_correspondence);
  icp.setMaximumIterations(kIcpMaxIterations);
  icp.setTransformationEpsilon(kIcpEpsilon);
  icp.setEuclideanFitnessEpsilon(kIcpEpsilon);
  icp.setRANSACIterations(0);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  Cloud aligned;
  icp.align(aligned);
  if (!icp.hasConverged()) return false;
  T_icp = Eigen::Isometry3d(icp.getFinalTransformation().cast<double>());
  fitness = icp.getFitnessScore();
  return true;
}

// ---- odometry history -----------------------------------------------------

bool covers(const std::map<double, Eigen::Isometry3d>& hist, double t) {
  return !hist.empty() && t >= hist.begin()->first && t <= hist.rbegin()->first;
}

// ---- writers --------------------------------------------------------------

// TUM: t x y z qx qy qz qw, one line per keyframe.
void writeTum(const std::string& path, const std::vector<double>& times,
              const std::vector<Eigen::Isometry3d>& poses) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  out << std::fixed;
  for (size_t i = 0; i < poses.size(); ++i) {
    const Eigen::Quaterniond q(poses[i].linear());
    const Eigen::Vector3d p = poses[i].translation();
    out << std::setprecision(6) << times[i] << " " << p.x() << " " << p.y()
        << " " << p.z() << " " << std::setprecision(9) << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << "\n";
  }
}

// loops.txt: cur prev fitness.
void writeLoops(const std::string& path, const std::vector<LoopEdge>& loops) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  for (const auto& l : loops) {
    out << l.cur << " " << l.prev << " " << l.fitness << "\n";
  }
}

void writePcd(const std::string& path, const Cloud& cloud) {
  if (cloud.empty()) throw std::runtime_error("empty map, not writing " + path);
  if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
    throw std::runtime_error("cannot write " + path);
  }
}

// One keyframe cloud in the map frame with the sensor pose in the PCD header
// (VIEWPOINT = translation, quaternion w x y z): the per-frame layout that
// DUFOMap and the DynamicMap_Benchmark tools read.
void writeKeyframePcd(const std::string& path, const Cloud& body,
                      const Eigen::Isometry3d& T_map_body) {
  Cloud map;
  pcl::transformPointCloud(body, map, T_map_body.matrix().cast<float>());
  const Eigen::Vector3f t = T_map_body.translation().cast<float>();
  map.sensor_origin_ = Eigen::Vector4f(t.x(), t.y(), t.z(), 0.0f);
  map.sensor_orientation_ =
      Eigen::Quaternionf(T_map_body.linear().cast<float>());
  writePcd(path, map);
}

std::string keyframeFileName(int k) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%06d.pcd", k);
  return buf;
}

// What a reader of the map needs to place it: frames, resolution, the
// georeference, T_mo, and the counts that describe how it was built.
struct MapSummary {
  bool georeferenced = false;
  Eigen::Isometry3d T_mo = Eigen::Isometry3d::Identity();
  size_t map_points = 0;
  size_t keyframes = 0;
  double path_length = 0.0;
  size_t loops = 0;
  size_t loops_rejected = 0;
  size_t gnss_factors = 0;
  int gnss_dropped = 0;
  Eigen::Vector3d gnss_residual_rms = Eigen::Vector3d::Zero();
  double align_time = 0.0;
  MapSource source;
};

// A double with at least one decimal, so every number in map.yaml parses as
// a float: "0.0" not "0". digits > 0: that many decimals, rounded; 0: 10
// significant digits, the full precision the pose needs.
std::string num(double v, int digits = 0) {
  std::ostringstream os;
  if (digits > 0) {
    os << std::fixed << std::setprecision(digits) << v;
  } else {
    os << std::setprecision(10) << v;
  }
  std::string out = os.str();
  if (out.find_first_of(".einN") == std::string::npos) out += ".0";
  return out;
}

std::string num3(const Eigen::Vector3d& v, int digits = 0) {
  return "[" + num(v.x(), digits) + ", " + num(v.y(), digits) + ", " +
         num(v.z(), digits) + "]";
}

// The "map" block is the contract a localizer reads; the rest is provenance
// and quality. Shape and an example: README.md ("map.yaml contract").
void writeMapYaml(const std::string& path, const BackendConfig& cfg,
                  const MapSummary& s) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  // ZYX angles by hand: eulerAngles() may return the flipped solution
  // (yaw +- 180 with roll and pitch near 180) for a nearly level T_mo.
  const Eigen::Matrix3d& R = s.T_mo.linear();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const Eigen::Quaterniond q(R);
  const Eigen::Vector3d& t = s.T_mo.translation();
  const double rad2deg = 180.0 / M_PI;
  const Eigen::Vector3d rpy_deg(roll * rad2deg, pitch * rad2deg, yaw * rad2deg);
  // An empty provenance string stays a string, not a YAML null.
  const auto str = [](const std::string& v) { return v.empty() ? "\"\"" : v; };

  out << "# Map description, written by brightdrive_mapping backend "
         "(saveMap).\n"
         "# The \"map\" block is the contract read by localization. Everything "
         "below it is provenance and quality only.\n"
      << "map:\n"
      << "  pcd: " << cfg.map_name << ".pcd"
      << "                  # file in the same folder as this yaml\n"
      << "  frame: " << (s.georeferenced ? cfg.map_frame : cfg.origin_frame)
      << "\n"
      << "  georeferenced: " << (s.georeferenced ? "true " : "false")
      << "             # false -> projection.type: Local, no origin keys\n";
  if (s.georeferenced) {
    out << "  projection:                      # from backend.yaml geo_*\n"
        << "    type: " << cfg.geo_projection << "\n"
        << "    origin_lat_deg: " << num(cfg.geo_origin_lat_deg) << "\n"
        << "    origin_lon_deg: " << num(cfg.geo_origin_lon_deg) << "\n"
        << "    origin_alt_m: " << num(cfg.geo_origin_alt_m) << "\n";
  } else {
    out << "  projection:\n"
        << "    type: Local\n";
  }
  out << "  resolution_m: " << num(cfg.save_resolution) << "\n"
      << "\n"
      << "source:                            # passed by the launch file\n"
      << "  bag: " << str(s.source.bag) << "\n"
      << "  frontend_commit: " << str(s.source.frontend_commit) << "\n"
      << "  backend_commit: " << str(s.source.backend_commit) << "\n"
      << "\n"
      << "mapping:\n"
      << "  origin_frame: " << cfg.origin_frame << "\n"
      << "  T_map_origin:                    # " << cfg.map_frame << " <- "
      << cfg.origin_frame << ", full precision\n"
      << "    xyz_m: " << num3(t) << "\n"
      << "    quaternion_xyzw: [" << num(q.x()) << ", " << num(q.y()) << ", "
      << num(q.z()) << ", " << num(q.w()) << "]\n"
      << "    rpy_deg: " << num3(rpy_deg, 4) << "\n"
      << "  keyframes: " << s.keyframes << "\n"
      << "  path_length_m: " << num(s.path_length, 2) << "\n"
      << "  map_points: " << s.map_points << "\n"
      << "  loops: {accepted: " << s.loops << ", rejected: " << s.loops_rejected
      << "}\n"
      << "\n"
      << "gnss:\n"
      << "  lever_arm_m: " << num3(cfg.lever_arm) << "\n"
      << "  factors: " << s.gnss_factors << "\n"
      << "  dropped: " << s.gnss_dropped << "\n"
      << "  residual_rms_m: " << num3(s.gnss_residual_rms, 3)
      << "   # final solve, per axis; compare with the GNSS sigma\n"
      << "  aligned_at_t: " << num(s.align_time, 2) << "\n";
}

}  // namespace

// ---- construction and inputs ----------------------------------------------

Backend::Backend(const BackendConfig& cfg)
    : cfg_(cfg), graph_(kRelinearizeThreshold, kRelinearizeSkip) {
  validate(cfg_);
}

void Backend::addOdometry(double t, const Eigen::Isometry3d& T_odom) {
  std::lock_guard<std::mutex> lock(mtx_);
  odom_hist_[t] = T_odom;
}

bool Backend::processScan(double t, const Eigen::Isometry3d& T_odom,
                          const CloudPtr& cloud) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!isKeyframe(T_odom)) return false;

  Keyframe kf;
  kf.id = static_cast<int>(keyframes_.size());
  kf.t = t;
  kf.T_odom = T_odom;
  kf.cloud_body = preprocess(cloud);
  const int k = kf.id;

  if (k == 0) {
    // The first keyframe pins the graph frame onto the odometry frame.
    graph_.addKeyframeNode(0, T_odom);
    graph_.addPriorFactor(0, T_odom, cfg_.prior_sigma);
  } else {
    const Eigen::Isometry3d T_rel = keyframes_.back().T_odom.inverse() * T_odom;
    path_len_ += T_rel.translation().norm();
    // Guess from the optimized previous pose, not its raw odometry: after a
    // loop closure the two differ by the whole correction.
    graph_.addKeyframeNode(k, graph_.keyframePose(k - 1) * T_rel);
    graph_.addOdomFactor(k - 1, k, T_rel, cfg_.odom_sigma);
  }
  keyframes_.push_back(kf);
  graph_.update();
  T_corr_ = graph_.keyframePose(k) * T_odom.inverse();

  if (cfg_.gnss_enable) {
    if (aligned_) {
      // The factors enter iSAM2 with the next keyframe's update.
      attachBufferedUpTo(t);
    } else if (path_len_ >= cfg_.align_distance) {
      tryAlign();
    }
  }
  return true;
}

void Backend::addGnssFix(const GnssFix& fix) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!cfg_.gnss_enable) return;
  GnssFix stored = fix;
  if (stored.cov.isZero()) {
    // The message carried no covariance; Gaussian::Covariance(0) is singular.
    stored.cov = Eigen::Matrix3d::Identity() *
                 (cfg_.gnss_default_sigma * cfg_.gnss_default_sigma);
  }
  gnss_buf_.push_back(stored);
}

// ---- keyframes --------------------------------------------------------------

bool Backend::isKeyframe(const Eigen::Isometry3d& T_odom) const {
  if (keyframes_.empty()) return true;
  const Eigen::Isometry3d d = keyframes_.back().T_odom.inverse() * T_odom;
  const double dist = d.translation().norm();
  const double angle = Eigen::AngleAxisd(d.linear()).angle();
  return dist >= cfg_.kf_dist || std::fabs(angle) >= cfg_.kf_angle;
}

CloudPtr Backend::preprocess(const CloudPtr& cloud) const {
  return voxelDownsample(*rangeCrop(*cloud, cfg_.min_range, cfg_.max_range),
                         cfg_.kf_leaf);
}

// ---- GNSS -------------------------------------------------------------------

Eigen::Isometry3d Backend::interpolate(double t) const {
  if (odom_hist_.empty()) {
    throw std::logic_error("Backend::interpolate with no odometry");
  }
  const auto hi = odom_hist_.lower_bound(t);
  if (hi == odom_hist_.end()) return odom_hist_.rbegin()->second;
  if (hi == odom_hist_.begin()) return hi->second;
  const auto lo = std::prev(hi);
  const double w = (t - lo->first) / (hi->first - lo->first);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() =
      (1.0 - w) * lo->second.translation() + w * hi->second.translation();
  T.linear() = Eigen::Quaterniond(lo->second.linear())
                   .slerp(w, Eigen::Quaterniond(hi->second.linear()))
                   .toRotationMatrix();
  return T;
}

void Backend::tryAlign() {
  // Antenna positions in the odometry frame for every fix the history covers.
  std::vector<Eigen::Vector3d> p_odom, p_map;
  for (const auto& fix : gnss_buf_) {
    if (!covers(odom_hist_, fix.t)) continue;
    p_odom.push_back(interpolate(fix.t) * cfg_.lever_arm);
    p_map.push_back(fix.p_map);
  }
  if (p_odom.size() < 2) return;  // more fixes will come
  Eigen::Isometry3d T_mo0;
  try {
    T_mo0 = geo::align4Dof(p_odom, p_map);
  } catch (const std::invalid_argument&) {
    return;  // no spread yet; the next keyframe tries again
  }
  graph_.addMapOriginNode(T_mo0, cfg_.origin_sigma);
  aligned_ = true;
  align_time_ = keyframes_.back().t;
  attachBufferedUpTo(keyframes_.back().t);
  graph_.update(kExtraIterations);
  T_corr_ = graph_.keyframePose(keyframes_.back().id) *
            keyframes_.back().T_odom.inverse();
}

void Backend::attachBufferedUpTo(double t_latest) {
  auto it = gnss_buf_.begin();
  while (it != gnss_buf_.end()) {
    if (it->t > t_latest) {
      ++it;
      continue;
    }
    attachGnss(*it);
    it = gnss_buf_.erase(it);
  }
}

void Backend::attachGnss(const GnssFix& fix) {
  // Nearest keyframe in time: keyframes_ is sorted by t.
  const auto after =
      std::lower_bound(keyframes_.begin(), keyframes_.end(), fix.t,
                       [](const Keyframe& kf, double t) { return kf.t < t; });
  const Keyframe* nearest = nullptr;
  if (after != keyframes_.end()) nearest = &*after;
  if (after != keyframes_.begin()) {
    const Keyframe* before = &*std::prev(after);
    if (nearest == nullptr ||
        std::fabs(before->t - fix.t) < std::fabs(nearest->t - fix.t)) {
      nearest = before;
    }
  }
  if (nearest == nullptr ||
      std::fabs(nearest->t - fix.t) > cfg_.max_time_shift ||
      !covers(odom_hist_, fix.t) || !covers(odom_hist_, nearest->t)) {
    ++n_gnss_dropped_;
    return;
  }
  // Move the fix to the keyframe time: the antenna travelled
  // interpolate(t_k) l - interpolate(t_g) l in the odometry frame, which is
  // R_mo times that in map. At 15 m/s and 1 deg of R_mo error this adds
  // under 1 cm.
  const Eigen::Matrix3d R_mo = graph_.mapOrigin()->linear();
  const Eigen::Vector3d antenna_k = interpolate(nearest->t) * cfg_.lever_arm;
  const Eigen::Vector3d antenna_g = interpolate(fix.t) * cfg_.lever_arm;
  const Eigen::Vector3d z_k = fix.p_map + R_mo * (antenna_k - antenna_g);
  graph_.addGnssFactor(nearest->id, z_k, fix.cov, cfg_.lever_arm,
                       cfg_.gnss_huber_k);
  gnss_attached_.emplace_back(nearest->id, z_k);
}

// ---- loop closure
// ------------------------------------------------------------

bool Backend::tryLoopClosure() {
  if (!cfg_.loop_enable) return false;
  LoopSnapshot s;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (keyframes_.size() < 2) return false;
    s.cur = static_cast<int>(keyframes_.size()) - 1;
    s.poses = graph_.keyframePoses();
    s.times.reserve(keyframes_.size());
    s.clouds.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) {
      s.times.push_back(kf.t);
      s.clouds.push_back(kf.cloud_body);  // immutable, sharing is safe
    }
  }
  if (loop_checked_.count(s.cur)) return false;
  const std::optional<int> prev = findLoopCandidate(s);
  if (!prev) return false;
  loop_checked_.insert(s.cur);  // one ICP attempt per keyframe

  Eigen::Isometry3d T_rel;
  double fitness = 0.0;
  if (!matchScans(s, *prev, T_rel, fitness)) {
    std::lock_guard<std::mutex> lock(mtx_);
    ++n_loops_rejected_;
    return false;
  }

  // Sigma from the ICP score: variance = fitness x scale, so a poorer match
  // counts less.
  const double f = std::max(fitness, kMinLoopFitness);
  Vector6d sigmas;
  sigmas.head<3>().setConstant(std::sqrt(f * cfg_.loop_rot_scale));
  sigmas.tail<3>().setConstant(std::sqrt(f * cfg_.loop_trans_scale));

  std::lock_guard<std::mutex> lock(mtx_);
  graph_.addLoopFactor(*prev, s.cur, T_rel, sigmas);
  graph_.update(kExtraIterations);
  T_corr_ = graph_.keyframePose(keyframes_.back().id) *
            keyframes_.back().T_odom.inverse();
  loops_.push_back(LoopEdge{s.cur, *prev, T_rel, fitness});
  return true;
}

std::optional<int> Backend::findLoopCandidate(const LoopSnapshot& s) const {
  // Nearest keyframe within loop_radius that is at least loop_time_diff
  // older: close in space, far in time, so the odometry has drifted between.
  const Eigen::Vector3d p_cur = s.poses[s.cur].translation();
  int best = -1;
  double best_dist = cfg_.loop_radius;
  for (int i = 0; i < s.cur; ++i) {
    if (s.times[s.cur] - s.times[i] < cfg_.loop_time_diff) continue;
    const double dist = (s.poses[i].translation() - p_cur).norm();
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }
  if (best < 0) return std::nullopt;
  return best;
}

bool Backend::matchScans(const LoopSnapshot& s, int prev,
                         Eigen::Isometry3d& T_rel, double& fitness) const {
  // Source: the current scan at its estimate. Target: the submap around prev.
  // Both in the graph frame, so T_icp is the correction of X_cur.
  const CloudPtr source =
      buildSubmap(s.poses, s.clouds, s.cur, 0, cfg_.loop_icp_leaf);
  const CloudPtr target = buildSubmap(s.poses, s.clouds, prev,
                                      cfg_.loop_search_num, cfg_.loop_icp_leaf);
  Eigen::Isometry3d T_icp;
  if (!icpAlign(source, target, 2.0 * cfg_.loop_radius, T_icp, fitness)) {
    return false;
  }
  if (fitness >= cfg_.loop_fitness) return false;
  // cur seen from prev, with the current scan where ICP put it.
  T_rel = s.poses[prev].inverse() * T_icp * s.poses[s.cur];
  return true;
}

// ---- outputs
// -----------------------------------------------------------------

std::optional<Eigen::Isometry3d> Backend::mapToOdomTransform() const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!cfg_.gnss_enable) return T_corr_;
  if (!aligned_) return std::nullopt;
  return *graph_.mapOrigin() * T_corr_;
}

std::vector<Eigen::Isometry3d> Backend::keyframePathInMap() const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (cfg_.gnss_enable && !aligned_) return {};
  const Eigen::Isometry3d T_mo =
      aligned_ ? *graph_.mapOrigin() : Eigen::Isometry3d::Identity();
  std::vector<Eigen::Isometry3d> path = graph_.keyframePoses();
  for (auto& T : path) T = T_mo * T;
  return path;
}

CloudPtr Backend::buildGlobalMap(double leaf) const {
  std::vector<Eigen::Isometry3d> poses;
  std::vector<CloudConstPtr> clouds;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (cfg_.gnss_enable && !aligned_) return CloudPtr(new Cloud);
    const Eigen::Isometry3d T_mo =
        aligned_ ? *graph_.mapOrigin() : Eigen::Isometry3d::Identity();
    poses = graph_.keyframePoses();
    for (auto& T : poses) T = T_mo * T;
    clouds.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) clouds.push_back(kf.cloud_body);
  }
  Cloud sum;
  for (size_t i = 0; i < poses.size(); ++i) {
    appendTransformed(*clouds[i], poses[i], sum);
  }
  return voxelDownsample(sum, leaf);
}

bool Backend::saveMap(const std::string& dir, const MapSource& source) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (keyframes_.empty()) return false;
  graph_.batchOptimize();
  MapSummary summary;
  summary.georeferenced = aligned_;
  if (aligned_) summary.T_mo = *graph_.mapOrigin();
  summary.source = source;

  // Poses and clouds in the map frame; the raw front-end poses as they came,
  // in origin_frame, for a before/after comparison.
  std::vector<Eigen::Isometry3d> poses = graph_.keyframePoses();
  std::vector<Eigen::Isometry3d> raw;
  std::vector<double> times;
  raw.reserve(poses.size());
  times.reserve(poses.size());
  Cloud sum;
  const std::string kf_dir = dir + "/keyframes";
  if (cfg_.save_keyframes) std::filesystem::create_directories(kf_dir);
  for (size_t k = 0; k < poses.size(); ++k) {
    poses[k] = summary.T_mo * poses[k];
    raw.push_back(keyframes_[k].T_odom);
    times.push_back(keyframes_[k].t);
    appendTransformed(*keyframes_[k].cloud_body, poses[k], sum);
    if (cfg_.save_keyframes) {
      writeKeyframePcd(kf_dir + "/" + keyframeFileName(static_cast<int>(k)),
                       *keyframes_[k].cloud_body, poses[k]);
    }
  }
  const CloudPtr map = voxelDownsample(sum, cfg_.save_resolution);

  // GNSS residuals of the final solve: T_mo X_k l - z_k.
  Eigen::Vector3d sq = Eigen::Vector3d::Zero();
  for (const auto& [k, z_k] : gnss_attached_) {
    const Eigen::Vector3d r = poses[k] * cfg_.lever_arm - z_k;
    sq += r.cwiseAbs2();
  }
  if (!gnss_attached_.empty()) {
    summary.gnss_residual_rms =
        (sq / static_cast<double>(gnss_attached_.size())).cwiseSqrt();
  }
  summary.map_points = map->size();
  summary.keyframes = keyframes_.size();
  summary.path_length = path_len_;
  summary.loops = loops_.size();
  summary.loops_rejected = n_loops_rejected_;
  summary.gnss_factors = gnss_attached_.size();
  summary.gnss_dropped = n_gnss_dropped_;
  summary.align_time = align_time_;

  std::filesystem::create_directories(dir);
  writePcd(dir + "/" + cfg_.map_name + ".pcd", *map);
  writeTum(dir + "/trajectory_optimized_tum.txt", times, poses);
  writeTum(dir + "/trajectory_fastlio_tum.txt", times, raw);
  writeMapYaml(dir + "/map.yaml", cfg_, summary);
  writeLoops(dir + "/loops.txt", loops_);
  return true;
}

bool Backend::aligned() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return aligned_;
}

int Backend::numKeyframes() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(keyframes_.size());
}

int Backend::numLoops() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(loops_.size());
}

int Backend::numLoopsRejected() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return n_loops_rejected_;
}

}  // namespace brightdrive_mapping

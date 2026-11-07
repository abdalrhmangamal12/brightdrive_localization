// match_bag: the scan matcher alone, against ground truth, on a recorded bag.
//
// For every N-th sweep: the GT pose at the sweep's reference time gives
// T_map_lidar; the sweep is deskewed with GT poses (one rigid transform per
// distinct point time), preprocessed once, then aligned from the true pose
// perturbed by d metres (random horizontal direction) and a degrees (yaw,
// random sign) for every --perturb case. Reports convergence, success
// (position < 0.3 m and yaw < 1 deg), error quantiles and timing per case,
// plus the map load and voxel-map build times.
//
//   ros2 run brightdrive_localization match_bag --bag /bags/<bag> --params
//   <localization.yaml>
//       [--map-yaml <map.yaml>] [--perturb 0.5:2 --perturb 1:5 --perturb 3:15]
//       [--every 5]
//       [--deskew gt|off] [--coarse-from 3.0] [--seed 1] [--csv out.csv]
//
// --coarse-from d: cases with d >= this value are also run coarse -> fine.
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "brightdrive_localization/config_yaml.hpp"
#include "brightdrive_localization/map_io.hpp"
#include "brightdrive_localization/ros_conversions.hpp"
#include "brightdrive_localization/scan_matcher.hpp"
#include "brightdrive_localization/types.hpp"

namespace bl = brightdrive_localization;

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kDeg = M_PI / 180.0;
constexpr double kSuccessPosM = 0.3;
constexpr double kSuccessYawRad = 1.0 * kDeg;

struct Args {
  std::string bag;
  std::string params;
  std::string map_yaml;                            // empty: the one in params
  std::vector<std::pair<double, double>> perturb;  // (m, deg)
  int every = 5;
  bool deskew = true;
  double coarse_from = 3.0;
  unsigned seed = 1;
  std::string csv;
  std::string gt_topic = "/eval/ground_truth";
  std::string points_topic = "/points_raw";
};

Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + k);
      return argv[++i];
    };
    if (k == "--bag")
      a.bag = next();
    else if (k == "--params")
      a.params = next();
    else if (k == "--map-yaml")
      a.map_yaml = next();
    else if (k == "--every")
      a.every = std::stoi(next());
    else if (k == "--seed")
      a.seed = static_cast<unsigned>(std::stoul(next()));
    else if (k == "--csv")
      a.csv = next();
    else if (k == "--coarse-from")
      a.coarse_from = std::stod(next());
    else if (k == "--gt-topic")
      a.gt_topic = next();
    else if (k == "--points-topic")
      a.points_topic = next();
    else if (k == "--deskew") {
      const std::string v = next();
      if (v == "gt")
        a.deskew = true;
      else if (v == "off")
        a.deskew = false;
      else
        throw std::runtime_error("--deskew takes gt or off");
    } else if (k == "--perturb") {
      const std::string v = next();
      const size_t colon = v.find(':');
      if (colon == std::string::npos)
        throw std::runtime_error("--perturb takes m:deg");
      a.perturb.emplace_back(std::stod(v.substr(0, colon)),
                             std::stod(v.substr(colon + 1)));
    } else {
      throw std::runtime_error("unknown argument " + k);
    }
  }
  if (a.bag.empty() || a.params.empty()) {
    throw std::runtime_error("--bag and --params are required");
  }
  if (a.perturb.empty()) {
    a.perturb = {{0.5, 2.0}, {1.0, 5.0}, {3.0, 15.0}};
  }
  if (a.every < 1) a.every = 1;
  return a;
}

// Ground-truth poses sorted by time, interpolated (linear + slerp).
class GtTrajectory {
 public:
  void add(double t, const Eigen::Isometry3d& T) { poses_[t] = T; }
  size_t size() const { return poses_.size(); }
  bool covers(double t) const {
    return !poses_.empty() && t >= poses_.begin()->first &&
           t <= poses_.rbegin()->first;
  }
  Eigen::Isometry3d at(double t) const {
    auto hi = poses_.lower_bound(t);
    if (hi == poses_.end()) return poses_.rbegin()->second;
    if (hi == poses_.begin() || hi->first == t) return hi->second;
    auto lo = std::prev(hi);
    const double a = (t - lo->first) / (hi->first - lo->first);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() =
        (1.0 - a) * lo->second.translation() + a * hi->second.translation();
    T.linear() = Eigen::Quaterniond(lo->second.linear())
                     .slerp(a, Eigen::Quaterniond(hi->second.linear()))
                     .toRotationMatrix();
    return T;
  }

 private:
  std::map<double, Eigen::Isometry3d> poses_;
};

// Every point moves into the lidar frame at t_ref = max(time) with
// T_rel(t) = T_ml(t_ref)^-1 T_ml(t): one transform per distinct point time.
void deskewWithGt(bl::Scan& scan, const GtTrajectory& gt,
                  const Eigen::Isometry3d& T_base_lidar) {
  const double t_ref = scan.tMax();
  const Eigen::Isometry3d T_ref_inv = (gt.at(t_ref) * T_base_lidar).inverse();
  std::unordered_map<int64_t, Eigen::Isometry3d> rel;
  for (size_t i = 0; i < scan.size(); ++i) {
    const int64_t key = std::llround(scan.time[i] * 1e7);  // 0.1 us buckets
    auto it = rel.find(key);
    if (it == rel.end()) {
      it = rel.emplace(key, T_ref_inv * gt.at(scan.time[i]) * T_base_lidar)
               .first;
    }
    scan.points[i] = it->second * scan.points[i];
    scan.points[i].w() = 1.0;
  }
}

struct Sample {
  double t_ref;
  std::string label;
  bool converged;
  bool success;
  int iterations;
  double pos_err_m;
  double yaw_err_deg;
  double rot_err_deg;
  double inlier_ratio;
  double error_per_inlier;
  double time_ms;
};

double quantile(std::vector<double> v, double q) {
  if (v.empty()) return std::nan("");
  const size_t k =
      static_cast<size_t>(std::clamp(q, 0.0, 1.0) * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

void report(const std::string& label, const std::vector<Sample>& all) {
  std::vector<double> pos, yaw, tm, inl, epi, its;
  size_t n = 0, conv = 0, succ = 0;
  for (const auto& s : all) {
    if (s.label != label) continue;
    ++n;
    conv += s.converged;
    succ += s.success;
    pos.push_back(s.pos_err_m);
    yaw.push_back(s.yaw_err_deg);
    tm.push_back(s.time_ms);
    inl.push_back(s.inlier_ratio);
    epi.push_back(s.error_per_inlier);
    its.push_back(s.iterations);
  }
  if (n == 0) return;
  std::printf(
      "| %-18s | %5zu | %6.1f | %6.1f | %7.3f | %7.3f | %7.2f | %7.2f | %6.1f "
      "| %6.1f | %5.2f | %6.2f | %4.0f |\n",
      label.c_str(), n, 100.0 * conv / n, 100.0 * succ / n, quantile(pos, 0.5),
      quantile(pos, 0.95), quantile(yaw, 0.5), quantile(yaw, 0.95),
      quantile(tm, 0.5), quantile(tm, 0.95), quantile(inl, 0.5),
      quantile(epi, 0.5), quantile(its, 0.5));
}

Sample evaluate(const std::string& label, double t_ref,
                const bl::MatchResult& m, const Eigen::Isometry3d& T_true) {
  Sample s;
  s.t_ref = t_ref;
  s.label = label;
  s.converged = m.converged;
  s.iterations = m.iterations;
  s.pos_err_m = (m.T_map_lidar.translation() - T_true.translation()).norm();
  const Eigen::Matrix3d dR =
      T_true.linear().transpose() * m.T_map_lidar.linear();
  s.rot_err_deg = bl::rotationAngle(dR) / kDeg;
  s.yaw_err_deg = std::fabs(bl::yawOf(dR)) / kDeg;
  s.success =
      s.pos_err_m < kSuccessPosM && s.yaw_err_deg < kSuccessYawRad / kDeg;
  s.inlier_ratio = m.inlierRatio();
  s.error_per_inlier = m.errorPerInlier();
  s.time_ms = m.time_ms;
  return s;
}

std::string caseLabel(double m, double deg, const char* suffix) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.1fm/%.0fdeg%s", m, deg, suffix);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  try {
    args = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "match_bag: " << e.what() << "\n";
    return 2;
  }

  const bl::LocalizerConfig cfg = bl::loadLocalizerConfig(args.params);
  std::string map_yaml = args.map_yaml;
  if (map_yaml.empty()) {
    // The node's default; the yaml itself holds the node-only key.
    map_yaml = "/maps/town03_static.yaml";
  }
  const bl::MapInfo map_info = bl::loadMapInfo(map_yaml);

  auto t0 = Clock::now();
  const auto map_points = bl::loadMapPoints(map_info.pcd_path);
  const double load_s =
      std::chrono::duration<double>(Clock::now() - t0).count();
  bl::ScanMatcher matcher(cfg.matcher);
  matcher.setMap(map_points);
  const auto& ms = matcher.mapStats();
  std::printf(
      "map: %s, %zu points, load %.2f s, covariances %.2f s, voxel maps %.2f s "
      "(%zu voxels at %.1f m, %zu at %.1f m), %d threads\n",
      map_info.pcd_path.c_str(), ms.points, load_s, ms.covariance_s,
      ms.voxelmap_s, ms.voxels_fine, cfg.matcher.map_voxel_m, ms.voxels_coarse,
      cfg.matcher.coarse_voxel_m, cfg.matcher.num_threads);

  // Pass 1: ground truth (base_link in map).
  GtTrajectory gt;
  {
    rosbag2_cpp::Reader reader;
    reader.open(args.bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {args.gt_topic};
    reader.set_filter(filter);
    rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
    while (reader.has_next()) {
      const auto bag_msg = reader.read_next();
      rclcpp::SerializedMessage sm(*bag_msg->serialized_data);
      nav_msgs::msg::Odometry odom;
      ser.deserialize_message(&sm, &odom);
      gt.add(bl::toSeconds(odom.header.stamp), bl::toEigen(odom.pose.pose));
    }
  }
  std::printf("ground truth: %zu poses from %s\n", gt.size(),
              args.gt_topic.c_str());
  if (gt.size() < 2) {
    std::cerr << "match_bag: no ground truth in the bag\n";
    return 1;
  }

  // Pass 2: sweeps.
  std::mt19937 rng(args.seed);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);
  std::bernoulli_distribution sign(0.5);
  std::vector<Sample> samples;
  std::vector<double> prep_ms;
  std::vector<double> src_pts;
  size_t sweeps = 0, used = 0, skipped_gt = 0;
  std::ofstream csv;
  if (!args.csv.empty()) {
    csv.open(args.csv);
    csv << "t_ref,case,converged,success,iterations,pos_err_m,yaw_err_deg,rot_"
           "err_deg,inlier_ratio,"
           "error_per_inlier,time_ms\n";
  }
  {
    rosbag2_cpp::Reader reader;
    reader.open(args.bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {args.points_topic};
    reader.set_filter(filter);
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
    while (reader.has_next()) {
      const auto bag_msg = reader.read_next();
      if (sweeps++ % static_cast<size_t>(args.every) != 0) continue;
      rclcpp::SerializedMessage sm(*bag_msg->serialized_data);
      sensor_msgs::msg::PointCloud2 msg;
      ser.deserialize_message(&sm, &msg);
      bl::Scan scan = bl::scanFromMsg(msg, cfg.points);
      if (scan.size() == 0 || !gt.covers(scan.tMin()) ||
          !gt.covers(scan.tMax())) {
        ++skipped_gt;
        continue;
      }
      const double t_ref = scan.tMax();
      const Eigen::Isometry3d T_true =
          gt.at(t_ref) * cfg.extrinsics.T_base_lidar;
      if (args.deskew) deskewWithGt(scan, gt, cfg.extrinsics.T_base_lidar);

      const auto tp = Clock::now();
      const auto source = matcher.preprocess(scan);
      prep_ms.push_back(
          std::chrono::duration<double>(Clock::now() - tp).count() * 1e3);
      src_pts.push_back(static_cast<double>(source->size()));
      ++used;

      const size_t first_sample = samples.size();
      for (const auto& [d_m, a_deg] : args.perturb) {
        const double th = angle(rng);
        const double yaw = (sign(rng) ? 1.0 : -1.0) * a_deg * kDeg;
        Eigen::Isometry3d T_init = T_true;
        T_init.translation() +=
            d_m * Eigen::Vector3d(std::cos(th), std::sin(th), 0.0);
        T_init.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix() *
                          T_true.linear();

        const bl::MatchResult fine =
            matcher.align(*source, T_init, bl::ScanMatcher::Level::kFine);
        samples.push_back(
            evaluate(caseLabel(d_m, a_deg, ""), t_ref, fine, T_true));
        if (d_m >= args.coarse_from) {
          bl::MatchResult coarse =
              matcher.align(*source, T_init, bl::ScanMatcher::Level::kCoarse);
          bl::MatchResult refined = matcher.align(
              *source, coarse.T_map_lidar, bl::ScanMatcher::Level::kFine);
          refined.time_ms += coarse.time_ms;
          samples.push_back(
              evaluate(caseLabel(d_m, a_deg, " c>f"), t_ref, refined, T_true));
        }
      }
      if (csv.is_open()) {
        for (size_t i = first_sample; i < samples.size(); ++i) {
          const auto& s = samples[i];
          csv << s.t_ref << ',' << s.label << ',' << s.converged << ','
              << s.success << ',' << s.iterations << ',' << s.pos_err_m << ','
              << s.yaw_err_deg << ',' << s.rot_err_deg << ',' << s.inlier_ratio
              << ',' << s.error_per_inlier << ',' << s.time_ms << '\n';
        }
      }
    }
  }

  std::printf(
      "sweeps: %zu in the bag, every %d -> %zu used, %zu without GT coverage; "
      "deskew %s; "
      "preprocess median %.1f ms, p95 %.1f ms; source points median %.0f\n",
      sweeps, args.every, used, skipped_gt, args.deskew ? "gt" : "off",
      quantile(prep_ms, 0.5), quantile(prep_ms, 0.95), quantile(src_pts, 0.5));
  std::printf(
      "success = position error < %.1f m and yaw error < %.0f deg; c>f = "
      "coarse map then fine\n",
      kSuccessPosM, kSuccessYawRad / kDeg);
  std::printf(
      "| %-18s | %5s | %6s | %6s | %7s | %7s | %7s | %7s | %6s | %6s | %5s | "
      "%6s | %4s |\n",
      "case", "n", "conv%", "succ%", "pos p50", "pos p95", "yaw p50", "yaw p95",
      "ms p50", "ms p95", "inl", "e/inl", "iter");
  std::printf("|%s|\n", std::string(129, '-').c_str());
  std::vector<std::string> labels;
  for (const auto& s : samples) {
    if (std::find(labels.begin(), labels.end(), s.label) == labels.end())
      labels.push_back(s.label);
  }
  for (const auto& l : labels) report(l, samples);
  return 0;
}

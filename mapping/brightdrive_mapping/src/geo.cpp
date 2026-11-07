// Georeferencing helpers. Formulas: geo.hpp.
#include "brightdrive_mapping/geo.hpp"

#include <cmath>
#include <stdexcept>

namespace brightdrive_mapping::geo {

namespace {
// WGS-84 equatorial radius [m]; CARLA's geo-reference uses it as a sphere.
constexpr double kEarthRadius = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

Eigen::Vector3d llaToMap(double lat_deg, double lon_deg, double alt_m) {
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  return Eigen::Vector3d(
      kEarthRadius * lon,
      kEarthRadius * std::log(std::tan(M_PI / 4.0 + lat / 2.0)), alt_m);
}

Eigen::Isometry3d align4Dof(const std::vector<Eigen::Vector3d>& p_odom,
                            const std::vector<Eigen::Vector3d>& p_map) {
  if (p_odom.size() != p_map.size()) {
    throw std::invalid_argument("align4Dof: p_odom and p_map differ in size");
  }
  if (p_odom.size() < 2) {
    throw std::invalid_argument("align4Dof: need at least 2 points");
  }
  const double n = static_cast<double>(p_odom.size());
  Eigen::Vector3d mean_odom = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_map = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < p_odom.size(); ++i) {
    mean_odom += p_odom[i];
    mean_map += p_map[i];
  }
  mean_odom /= n;
  mean_map /= n;

  // Yaw: 2-D Procrustes on the centred horizontal coordinates. With
  // a = p_odom - mean_odom and b = p_map - mean_map, the rotation about z
  // that minimizes sum |b - Rz(yaw) a|^2 is yaw = atan2(sum(a x b)_z,
  // sum(a . b)_xy): the angle of the summed complex ratio b / a.
  double cross = 0.0;
  double dot = 0.0;
  for (size_t i = 0; i < p_odom.size(); ++i) {
    const Eigen::Vector3d a = p_odom[i] - mean_odom;
    const Eigen::Vector3d b = p_map[i] - mean_map;
    cross += a.x() * b.y() - a.y() * b.x();
    dot += a.x() * b.x() + a.y() * b.y();
  }
  if (std::hypot(cross, dot) < 1e-9) {
    throw std::invalid_argument("align4Dof: points have no horizontal spread");
  }
  const double yaw = std::atan2(cross, dot);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // With the rotation fixed, the translation that minimizes the same sum maps
  // centroid onto centroid. z has no rotation term, so t_z is the mean height
  // difference.
  T.translation() = mean_map - T.linear() * mean_odom;
  return T;
}

}  // namespace brightdrive_mapping::geo

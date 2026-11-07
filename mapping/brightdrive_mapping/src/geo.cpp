// Georeferencing helpers. Formulas: geo.hpp.
#include "brightdrive_mapping/geo.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace brightdrive_mapping::geo {

namespace {
// WGS-84 equatorial radius [m]; CARLA's geo-reference uses it as a sphere.
constexpr double kEarthRadius = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double mercatorY(double lat_rad) {
  return std::log(std::tan(M_PI / 4.0 + lat_rad / 2.0));
}

// !(lo <= v <= hi) also rejects NaN.
void requireWithin(double v, double lo, double hi, const char* what) {
  if (!(v >= lo && v <= hi)) {
    throw std::invalid_argument(std::string("geo::Projector: origin ") + what +
                                " must be finite and within [" +
                                std::to_string(lo) + ", " + std::to_string(hi) +
                                "]");
  }
}
}  // namespace

Projection parseProjection(const std::string& name) {
  if (name == "SphericalMercator") return Projection::SphericalMercator;
  if (name == "LocalCartesian") return Projection::LocalCartesian;
  throw std::invalid_argument("geo: unknown projection '" + name +
                              "' (known: SphericalMercator, LocalCartesian)");
}

const char* projectionName(Projection p) {
  switch (p) {
    case Projection::SphericalMercator:
      return "SphericalMercator";
    case Projection::LocalCartesian:
      return "LocalCartesian";
  }
  return "?";
}

Projector::Projector(Projection type, double origin_lat_deg,
                     double origin_lon_deg, double origin_alt_m)
    : type_(type),
      lat0_deg_(origin_lat_deg),
      lon0_deg_(origin_lon_deg),
      alt0_m_(origin_alt_m) {
  requireWithin(origin_lat_deg, -90.0, 90.0, "latitude [deg]");
  requireWithin(origin_lon_deg, -180.0, 180.0, "longitude [deg]");
  if (!std::isfinite(origin_alt_m)) {
    throw std::invalid_argument(
        "geo::Projector: origin altitude [m] must be finite");
  }
  scale_ = std::cos(origin_lat_deg * kDegToRad);
  merc_y0_ = mercatorY(origin_lat_deg * kDegToRad);
  local_.Reset(origin_lat_deg, origin_lon_deg, origin_alt_m);
}

Eigen::Vector3d Projector::llaToMap(double lat_deg, double lon_deg,
                                    double alt_m) const {
  if (type_ == Projection::LocalCartesian) {
    Eigen::Vector3d p;
    local_.Forward(lat_deg, lon_deg, alt_m, p.x(), p.y(), p.z());
    return p;
  }
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double lon0 = lon0_deg_ * kDegToRad;
  return Eigen::Vector3d(scale_ * kEarthRadius * (lon - lon0),
                         scale_ * kEarthRadius * (mercatorY(lat) - merc_y0_),
                         alt_m - alt0_m_);
}

Eigen::Vector3d Projector::mapToLla(const Eigen::Vector3d& p) const {
  if (type_ == Projection::LocalCartesian) {
    Eigen::Vector3d lla;
    local_.Reverse(p.x(), p.y(), p.z(), lla.x(), lla.y(), lla.z());
    return lla;
  }
  const double kR = scale_ * kEarthRadius;
  const double lon = lon0_deg_ * kDegToRad + p.x() / kR;
  const double lat =
      2.0 * std::atan(std::exp(p.y() / kR + merc_y0_)) - M_PI / 2.0;
  return Eigen::Vector3d(lat * kRadToDeg, lon * kRadToDeg, p.z() + alt0_m_);
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

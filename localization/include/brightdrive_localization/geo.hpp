// Georeferencing: lat/lon/alt to map metres, and the 4-DoF fit that gives
// the first T_mo from antenna positions. A copy of mapping/brightdrive_mapping
// geo.hpp (namespace renamed, nothing else): the localizer rebuilds the
// projector the map was georeferenced with from map.yaml, with the same code.
// Keep the two files in sync.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <string>
#include <vector>

namespace brightdrive_localization::geo {

// How a fix becomes map x/y/z. The name is written into map.yaml
// (map.projection.type) together with the origin, so a localizer rebuilds
// the same Projector from the map alone.
enum class Projection {
  // The inverse of CARLA's GeoLocation::Transform: Mercator on a sphere of
  // radius R = 6 378 137 m, scaled by k = cos(lat0) and shifted so that the
  // origin maps to zero (angles in rad):
  //   x = k R (lon - lon0)
  //   y = k R (ln tan(pi/4 + lat/2) - ln tan(pi/4 + lat0/2))
  //   z = alt - alt0
  // Town03's OpenDRIVE geoReference is +lat_0=0 +lon_0=0: k = 1, no shift.
  // The ground-truth comparison in eval/ validated this against CARLA: GNSS
  // minus GT has no bias beyond the 0.3 m noise.
  SphericalMercator,
  // Real receivers and the other simulators (Gazebo, Isaac, AWSIM) give
  // WGS-84 lat/lon. GeographicLib::LocalCartesian at the origin: x east,
  // y north, z up in metres, the ROS map convention (REP-103). The origin
  // belongs inside the mapped area.
  LocalCartesian,
};

// "SphericalMercator" or "LocalCartesian"; throws std::invalid_argument
// naming any other string.
Projection parseProjection(const std::string& name);
const char* projectionName(Projection p);

class Projector {
 public:
  // Throws std::invalid_argument when the origin is not finite or lat is
  // outside [-90, 90], lon outside [-180, 180].
  Projector(Projection type, double origin_lat_deg, double origin_lon_deg,
            double origin_alt_m);

  Eigen::Vector3d llaToMap(double lat_deg, double lon_deg, double alt_m) const;
  // The inverse: (lat_deg, lon_deg, alt_m) of a map point. For checks.
  Eigen::Vector3d mapToLla(const Eigen::Vector3d& p_map) const;

  Projection type() const { return type_; }
  double originLatDeg() const { return lat0_deg_; }
  double originLonDeg() const { return lon0_deg_; }
  double originAltM() const { return alt0_m_; }

 private:
  Projection type_;
  double lat0_deg_;
  double lon0_deg_;
  double alt0_m_;
  // SphericalMercator: k and the origin's Mercator y, fixed at construction.
  double scale_ = 1.0;
  double merc_y0_ = 0.0;
  // LocalCartesian: WGS-84 ENU at the origin, reset once at construction.
  GeographicLib::LocalCartesian local_;
};

// Least-squares T (x, y, z, yaw; roll = pitch = 0) with
// p_map[i] ~ T * p_odom[i] over matched antenna positions.
// Throws std::invalid_argument on a size mismatch, fewer than 2 points, or
// points with no horizontal spread (yaw undefined).
Eigen::Isometry3d align4Dof(const std::vector<Eigen::Vector3d>& p_odom,
                            const std::vector<Eigen::Vector3d>& p_map);

}  // namespace brightdrive_localization::geo

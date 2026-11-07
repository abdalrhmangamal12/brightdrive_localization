// Georeferencing: lat/lon/alt to map metres, and the 4-DoF fit that gives
// the first T_mo from antenna positions.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace brightdrive_mapping::geo {

// CARLA Town03 geo-reference: lat0 = lon0 = 0 on a sphere of radius
// R = 6 378 137 m (Mercator):
//   x = R * lon,  y = R * ln(tan(pi/4 + lat/2)),  z = alt    (lat, lon in rad)
// The formula the ground-truth comparison in eval/ validated: GNSS - GT has
// no bias beyond the 0.3 m noise (decisions.md section 7).
Eigen::Vector3d llaToMap(double lat_deg, double lon_deg, double alt_m);

// Least-squares T (x, y, z, yaw; roll = pitch = 0) with
// p_map[i] ~ T * p_odom[i] over matched antenna positions.
// Throws std::invalid_argument on a size mismatch, fewer than 2 points, or
// points with no horizontal spread (yaw undefined).
Eigen::Isometry3d align4Dof(const std::vector<Eigen::Vector3d>& p_odom,
                            const std::vector<Eigen::Vector3d>& p_map);

}  // namespace brightdrive_mapping::geo

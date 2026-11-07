// map.yaml reader (yaml-cpp) and PCD loader (PCL io).
#include "brightdrive_localization/map_io.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

namespace brightdrive_localization {

namespace {

template <typename T>
T required(const YAML::Node& n, const char* key, const std::string& where) {
  if (!n[key]) {
    throw std::runtime_error("map.yaml: missing key " + where + "." + key);
  }
  return n[key].as<T>();
}

}  // namespace

MapInfo loadMapInfo(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("map.yaml: cannot read " + yaml_path + ": " +
                             e.what());
  }
  if (!root["map"]) {
    throw std::runtime_error("map.yaml: no `map:` block in " + yaml_path);
  }
  const YAML::Node m = root["map"];
  MapInfo info;
  const std::string pcd = required<std::string>(m, "pcd", "map");
  const std::filesystem::path p(pcd);
  info.pcd_path =
      p.is_absolute()
          ? p.string()
          : (std::filesystem::path(yaml_path).parent_path() / p).string();
  info.frame = required<std::string>(m, "frame", "map");
  info.georeferenced = required<bool>(m, "georeferenced", "map");
  if (!info.georeferenced) {
    throw std::runtime_error(
        "map.yaml: map.georeferenced is false; GNSS fixes and seeds have no "
        "map coordinates on this map");
  }
  if (!m["projection"]) {
    throw std::runtime_error("map.yaml: missing key map.projection");
  }
  const YAML::Node proj = m["projection"];
  info.projection = required<std::string>(proj, "type", "map.projection");
  info.origin_lat_deg =
      required<double>(proj, "origin_lat_deg", "map.projection");
  info.origin_lon_deg =
      required<double>(proj, "origin_lon_deg", "map.projection");
  info.origin_alt_m = required<double>(proj, "origin_alt_m", "map.projection");
  info.resolution_m = required<double>(m, "resolution_m", "map");
  return info;
}

std::vector<Eigen::Vector4d> loadMapPoints(const std::string& pcd_path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_path, cloud) < 0) {
    throw std::runtime_error("cannot read PCD " + pcd_path);
  }
  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      points.emplace_back(p.x, p.y, p.z, 1.0);
    }
  }
  return points;
}

}  // namespace brightdrive_localization

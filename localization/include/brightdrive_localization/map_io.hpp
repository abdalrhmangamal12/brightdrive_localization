// The map on disk: map.yaml (the `map:` block) and the PCD it names.
#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

// Reads only the `map:` block of map.yaml (pcd, frame, georeferenced,
// projection{type, origin_*}, resolution_m). Throws std::runtime_error when a
// key is missing or georeferenced is false: without a projection GNSS fixes
// and seeds have no map coordinates. pcd is resolved relative to the yaml.
MapInfo loadMapInfo(const std::string& yaml_path);

// x y z of every point of a PCD (ascii or binary), as x y z 1. Throws
// std::runtime_error when the file cannot be read.
std::vector<Eigen::Vector4d> loadMapPoints(const std::string& pcd_path);

}  // namespace brightdrive_localization

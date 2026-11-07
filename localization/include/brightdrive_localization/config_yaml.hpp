// localization.yaml read without ROS, for the offline tools. The node reads
// the same file through rclcpp parameters; both end in a LocalizerConfig.
#pragma once

#include <string>

#include "brightdrive_localization/params.hpp"

namespace brightdrive_localization {

// Reads the `/**: ros__parameters:` block. Throws std::runtime_error naming a
// missing key; the result is validated (params.hpp) before it is returned.
LocalizerConfig loadLocalizerConfig(const std::string& yaml_path);

}  // namespace brightdrive_localization

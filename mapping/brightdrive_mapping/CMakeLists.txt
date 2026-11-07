cmake_minimum_required(VERSION 3.16)
project(brightdrive_mapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
# NO_SYSTEM_ENVIRONMENT_PATH: do not derive search prefixes from $PATH. A conda/pixi bin dir on PATH would
# otherwise shadow the system package with one built against a different libstdc++.
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE NO_SYSTEM_ENVIRONMENT_PATH)
# ros-humble-gtsam 4.2.0: built against the system Eigen 3.4 (GTSAM_USE_SYSTEM_EIGEN), so one Eigen everywhere.
find_package(GTSAM 4.2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
# GeographicLib, for geo::Projection::LocalCartesian. Ubuntu 22.04's libgeographic-dev 1.52 installs a Find
# module (/usr/share/cmake/geographiclib/FindGeographicLib.cmake) and no config file; it sets
# GeographicLib_INCLUDE_DIRS and GeographicLib_LIBRARIES, no imported target and no version variable.
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

# ---- mapping_core: the estimation core, no ROS -----------------------------------------------------------
# Every .cpp in src/ except the node: geo, pose_graph, backend (types.hpp and params.hpp are header-only).
# Header-only until the first .cpp exists, like eskf/.
file(GLOB MAPPING_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(FILTER MAPPING_CORE_SOURCES EXCLUDE REGEX "backend_node\\.cpp$")
if(MAPPING_CORE_SOURCES)
  add_library(mapping_core STATIC ${MAPPING_CORE_SOURCES})
  target_include_directories(mapping_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # SYSTEM: PCL headers do not pass -Wpedantic; the warnings are theirs, not ours.
  target_include_directories(mapping_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})
  target_link_libraries(mapping_core PUBLIC Eigen3::Eigen gtsam ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})
else()
  add_library(mapping_core INTERFACE)
  target_include_directories(mapping_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_include_directories(mapping_core SYSTEM INTERFACE ${PCL_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})
  target_link_libraries(mapping_core INTERFACE Eigen3::Eigen gtsam ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})
endif()

# ---- geo_check: the projector checks, no GTest (ros2 run brightdrive_mapping geo_check) ------------------
add_executable(geo_check tools/geo_check.cpp)
target_link_libraries(geo_check mapping_core)

# ---- backend_node: the ROS 2 wiring around mapping_core ---------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(backend_node src/backend_node.cpp)
# Plain signature: ament_target_dependencies links with it too, and CMake refuses to mix the two forms.
target_link_libraries(backend_node mapping_core)
ament_target_dependencies(backend_node
  rclcpp nav_msgs sensor_msgs geometry_msgs std_srvs tf2_ros pcl_conversions)
install(TARGETS backend_node geo_check DESTINATION lib/${PROJECT_NAME})
# The two Python tools the replay launch runs through `ros2 run`: the save client and the dynamic-object filter.
install(PROGRAMS scripts/save_map.py scripts/dynamic_filter.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

ament_package()

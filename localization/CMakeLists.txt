cmake_minimum_required(VERSION 3.16)
project(brightdrive_localization LANGUAGES CXX)

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
find_package(OpenMP REQUIRED)
# PCL only reads the map PCD (map_io.cpp).
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
# GeographicLib, for geo::Projector (copied from mapping). Ubuntu 22.04's libgeographic-dev 1.52 installs a Find
# module and no config file; it sets GeographicLib_INCLUDE_DIRS / _LIBRARIES, no imported target.
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)
# small_gicp is header-only. The loc image clones it to /opt/small_gicp and exports SMALL_GICP_INCLUDE_DIR.
find_path(SMALL_GICP_INCLUDE_DIR small_gicp/registration/registration.hpp
  HINTS ENV SMALL_GICP_INCLUDE_DIR /opt/small_gicp/include)
if(NOT SMALL_GICP_INCLUDE_DIR)
  message(FATAL_ERROR "small_gicp headers not found: build in the loc container (docker compose exec loc bash) "
                      "or set SMALL_GICP_INCLUDE_DIR")
endif()

# ---- eskf: the filter, built from ../eskf as it is (its tests and tools stay with it) ---------------------
set(ESKF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ESKF_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../eskf ${CMAKE_CURRENT_BINARY_DIR}/eskf)

# ---- loc_core: the estimation plumbing around eskf, no ROS -------------------------------------------------
# Every .cpp in src/ except the node.
file(GLOB LOC_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(FILTER LOC_CORE_SOURCES EXCLUDE REGEX "localization_node\\.cpp$")
add_library(loc_core STATIC ${LOC_CORE_SOURCES})
target_include_directories(loc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# SYSTEM: PCL and small_gicp headers do not pass -Wpedantic; the warnings are theirs, not ours.
target_include_directories(loc_core SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${SMALL_GICP_INCLUDE_DIR})
target_link_libraries(loc_core PUBLIC
  eskf Eigen3::Eigen OpenMP::OpenMP_CXX yaml-cpp ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})

# ---- localization_node: the ROS 2 wiring around loc_core --------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(localization_node src/localization_node.cpp)
# Plain signature: ament_target_dependencies links with it too, and CMake refuses to mix the two forms.
target_link_libraries(localization_node loc_core)
ament_target_dependencies(localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs std_msgs tf2_ros)
install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})

# ---- match_bag: the matcher alone against ground truth, from a bag (tools/match_bag.cpp) --------------------
find_package(rosbag2_cpp REQUIRED)
add_executable(match_bag tools/match_bag.cpp)
target_link_libraries(match_bag loc_core)
ament_target_dependencies(match_bag rclcpp rosbag2_cpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS match_bag DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz foxglove
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

ament_package()

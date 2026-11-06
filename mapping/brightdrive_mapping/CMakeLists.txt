cmake_minimum_required(VERSION 3.16)
project(brightdrive_mapping)

find_package(ament_cmake REQUIRED)

# No targets yet. config/ and launch/ are installed so ros2 launch finds the package.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

ament_package()

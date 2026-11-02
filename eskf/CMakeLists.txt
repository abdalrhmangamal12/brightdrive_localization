cmake_minimum_required(VERSION 3.16)
project(eskf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# NO_SYSTEM_ENVIRONMENT_PATH: do not derive search prefixes from $PATH. A conda/pixi bin dir on PATH would
# otherwise shadow the system package with one built against a different libstdc++ (test binaries fail to load).
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE NO_SYSTEM_ENVIRONMENT_PATH)

# ---- library: every .cpp in src/ (header-only until the first .cpp exists) ----------------------------------
file(GLOB ESKF_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
if(ESKF_SOURCES)
  add_library(eskf ${ESKF_SOURCES})
  target_include_directories(eskf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(eskf PUBLIC Eigen3::Eigen)
else()
  add_library(eskf INTERFACE)
  target_include_directories(eskf INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(eskf INTERFACE Eigen3::Eigen)
endif()

# ---- tests: every test/test_*.cpp becomes one GTest executable --------------------------------------------
option(ESKF_BUILD_TESTS "Build the unit tests" ON)
if(ESKF_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET NO_SYSTEM_ENVIRONMENT_PATH)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
  include(GoogleTest)

  file(GLOB ESKF_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
  foreach(test_src ${ESKF_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE eskf GTest::gtest_main)
    gtest_discover_tests(${test_name})
  endforeach()
endif()

# One Dockerfile, four stages: base, sim (CARLA client), loc (estimator), mapping.
# Build one with:  docker compose build sim

# pinned by digest like the carla image; retag: docker buildx imagetools inspect ros:humble-ros-base-jammy
FROM ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855 AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# numpy from apt only (1.21.5, same as Humble). Never pip install/upgrade it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Fast DDS profile (64 MiB shm segment). compose.yaml points FASTRTPS_DEFAULT_PROFILES_FILE here.
COPY docker/fastdds_shm.xml /etc/fastdds_shm.xml

# The containers run as the host user (user: in compose.yaml). A passwd entry for that uid/gid, so shells show a name
# instead of "I have no name!"; no home dir, carla map cache and ros logs go to /tmp.
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN groupadd --gid "${HOST_GID}" bd \
    && useradd --uid "${HOST_UID}" --gid "${HOST_GID}" --home-dir /tmp --no-create-home --shell /bin/bash bd
ENV HOME=/tmp \
    ROS_LOG_DIR=/tmp/ros_logs

# so `docker compose exec sim bash` has ros2 ready
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
FROM base AS sim
# ScenarioRunner's Python libraries from apt (the Ubuntu 22.04 builds for Python 3.10: ScenarioRunner's own pins
# predate 3.10, networkx 2.2 does not import there and Shapely 1.7.1 has no wheel), rosbag2's mcap storage for
# the recordings, foxglove_bridge to watch a run live (port 8765), git for the ScenarioRunner clone.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-bridge \
        git \
        python3-yaml \
        python3-networkx \
        python3-shapely \
        python3-psutil \
        python3-ephem \
        python3-tabulate \
        python3-matplotlib \
        python3-six \
        python3-pydot \
        python3-distutils \
        python3-pkg-resources \
    && rm -rf /var/lib/apt/lists/*
# carla wheel + ScenarioRunner's pure-Python pins. --no-deps, otherwise pip pulls in a second numpy.
# opencv: ScenarioRunner pins 4.2.0.32 (no cp310 wheel); 4.7 headless is the last release that imports on numpy 1.21.
RUN python3 -m pip install --no-cache-dir --no-deps \
        carla==0.9.15 \
        py_trees==0.8.3 \
        xmlschema==1.0.18 \
        elementpath==1.3.3 \
        antlr4-python3-runtime==4.10 \
        simple-watchdog-timer==0.1.1 \
        opencv-python-headless==4.7.0.72
# CARLA's `agents` package (route planner, BasicAgent: what our scenario controllers drive with) is not in the
# PyPI wheel; taken from the server image, so it is the server's own release.
COPY --from=carlasim/carla:0.9.15@sha256:1af47c314443f4a2d0869ac399c5b5eabde138a5bc39dd31dcbb1540e22f5588 \
     /home/carla/PythonAPI/carla/agents /opt/carla/PythonAPI/carla/agents
# ScenarioRunner at the server's version. The tag's commit is checked, so a moved tag fails the build.
ARG SCENARIO_RUNNER_COMMIT=d12d8bb788201ebf6f03e55b9c5169f9dcb2ea87
RUN git clone --depth 1 --branch v0.9.15 https://github.com/carla-simulator/scenario_runner.git /opt/scenario_runner \
    && [ "$(git -C /opt/scenario_runner rev-parse HEAD)" = "${SCENARIO_RUNNER_COMMIT}" ]
ENV PYTHONPATH=/opt/carla/PythonAPI/carla:/opt/scenario_runner
# Fail the build, not a run: the wheel next to apt's numpy, agents, ScenarioRunner's imports, mcap storage.
RUN python3 -c "import carla, numpy; from importlib.metadata import version; \
    print('carla', version('carla'), carla.__file__); print('numpy', numpy.__version__, numpy.__file__)" \
    && python3 -c "from agents.navigation.basic_agent import BasicAgent; print('agents OK')" \
    && python3 -c "import srunner.scenariomanager.actorcontrols.simple_vehicle_control; print('srunner OK')" \
    && python3 /opt/scenario_runner/scenario_runner.py --version \
    && source /opt/ros/humble/setup.bash && ros2 bag record --help | grep -q -E -- '--storage \{[^}]*mcap' && echo 'mcap OK'

# ---------------------------------------------------------------------------
FROM base AS loc
# GeographicLib: the localizer copies mapping's geo.hpp/geo.cpp (GeographicLib::LocalCartesian) to rebuild a
# map's projector from map.yaml. The rest of the estimator's deps (loc_ros) are still to come.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
FROM base AS mapping
# FAST-LIO is built into the image as an underlay; brightdrive_mapping is built from the mounted repo at run time.
#
# FAST-LIO: pcl_ros, pcl_conversions, rclcpp_components, std_srvs, visualization_msgs (the rest is in ros-base).
# Backend: GTSAM, PCL (pcl_ros pulls the full libpcl-dev), GeographicLib (geo: LocalCartesian for WGS-84 fixes),
# tf2_eigen / tf2_geometry_msgs / tf2_sensor_msgs, GTest.
# Both: mcap storage to play the bags, foxglove_bridge to watch a run (port 8765).
# ros-humble-gtsam 4.2.0 links the system Eigen 3.4.0 and exports no -march=native, so no source build needed.
# Already in base: build-essential, cmake, git, colcon, python3-dev, libeigen3-dev, ament_cmake_gtest.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gtsam \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        ros-humble-rclcpp-components \
        ros-humble-std-srvs \
        ros-humble-visualization-msgs \
        ros-humble-tf2-eigen \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-sensor-msgs \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-bridge \
        libgeographic-dev \
        libgtest-dev \
        libgmock-dev \
    && rm -rf /var/lib/apt/lists/*

# FAST-LIO at a fixed commit with its ikd-Tree submodule, untouched. Plain clone, then the commit is checked.
# livox_ros_driver2 is optional in this fork and not installed; the CMake warning about it is expected.
ARG FAST_LIO_REPO=https://github.com/OmerMersin/FAST_LIO_OUSTER.git
ARG FAST_LIO_COMMIT=8ffac51a25d33800838e300cd5e04f2330fdcdbb
ENV FAST_LIO_WS=/opt/fast_lio_ws
RUN git clone "${FAST_LIO_REPO}" ${FAST_LIO_WS}/src/fast_lio \
    && git -C ${FAST_LIO_WS}/src/fast_lio -c advice.detachedHead=false checkout "${FAST_LIO_COMMIT}" \
    && git -C ${FAST_LIO_WS}/src/fast_lio submodule update --init --recursive \
    && [ "$(git -C ${FAST_LIO_WS}/src/fast_lio rev-parse HEAD)" = "${FAST_LIO_COMMIT}" ]
# src/ stays: ROOT_DIR is baked in at compile time and the node writes PCD/ and Log/ under it, so those two
# are made writable for the compose uid.
RUN cd ${FAST_LIO_WS} && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log \
    && chmod 1777 src/fast_lio/PCD src/fast_lio/Log

# Pins, for the README.
RUN { echo "fast_lio ${FAST_LIO_REPO} $(git -C ${FAST_LIO_WS}/src/fast_lio rev-parse HEAD)"; \
      echo "ikd-Tree https://github.com/hku-mars/ikd-Tree $(git -C ${FAST_LIO_WS}/src/fast_lio/include/ikd-Tree rev-parse HEAD)"; \
      dpkg-query -W -f '${Package} ${Version}\n' ros-humble-gtsam libpcl-dev libeigen3-dev libgeographic-dev ros-humble-pcl-ros libgtest-dev; \
    } > /opt/VERSIONS.txt

# Build-time checks: package and executable resolve, CMake finds GTSAM / PCL / Eigen, mcap plugin present.
RUN source /opt/ros/humble/setup.bash && source ${FAST_LIO_WS}/install/setup.bash \
    && echo "fast_lio at $(ros2 pkg prefix fast_lio)" \
    && ros2 pkg executables fast_lio | grep -q fastlio_mapping && echo 'fastlio_mapping OK' \
    && ls "$(ros2 pkg prefix fast_lio)/share/fast_lio/config" \
    && mkdir -p /tmp/chk && printf '%s\n' 'cmake_minimum_required(VERSION 3.16)' 'project(chk CXX)' \
        'find_package(GTSAM 4.2 REQUIRED)' 'find_package(PCL REQUIRED COMPONENTS common io registration filters)' \
        'find_package(Eigen3 3.4 REQUIRED)' \
        'list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)' 'find_package(GeographicLib REQUIRED)' \
        'message(STATUS "found GTSAM ${GTSAM_VERSION} PCL ${PCL_VERSION} Eigen3 ${EIGEN3_VERSION_STRING} GeographicLib ${GeographicLib_LIBRARIES}")' \
        > /tmp/chk/CMakeLists.txt \
    && cmake -S /tmp/chk -B /tmp/chk/build | grep 'found GTSAM' && rm -rf /tmp/chk \
    && ros2 bag record --help | grep -q -E -- '--storage \{[^}]*mcap' && echo 'mcap OK' \
    && cat /opt/VERSIONS.txt

# Entry point sources Humble, the underlay and the repo's install/ if it exists. `docker compose exec mapping bash`
# skips the entry point, hence the bashrc lines.
COPY docker/mapping_entrypoint.sh /mapping_entrypoint.sh
RUN chmod 755 /mapping_entrypoint.sh \
    && printf '%s\n' 'source /opt/fast_lio_ws/install/setup.bash' \
         'if [ -f /ws/src/brightdrive_localization/install/setup.bash ]; then source /ws/src/brightdrive_localization/install/setup.bash; fi' \
         >> /etc/bash.bashrc
ENTRYPOINT ["/mapping_entrypoint.sh"]
CMD ["bash"]

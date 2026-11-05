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

# containers run as uid 1000, which has no home dir -> carla map cache and ros logs go to /tmp
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
# TODO estimator deps (loc_ros)

# ---------------------------------------------------------------------------
FROM base AS mapping
# TODO mapping deps (FAST-LIO2, GTSAM, PCL)

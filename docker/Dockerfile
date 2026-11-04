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
# carla wheel only. --no-deps, otherwise pip pulls in a second numpy
RUN python3 -m pip install --no-cache-dir --no-deps carla==0.9.15
# quick import check
RUN python3 -c "import carla, numpy; from importlib.metadata import version; \
    print('carla', version('carla'), carla.__file__); print('numpy', numpy.__version__, numpy.__file__)"
# TODO later: scenario_runner, carla agents, bridge deps, mcap

# ---------------------------------------------------------------------------
FROM base AS loc
# TODO estimator deps (loc_ros)

# ---------------------------------------------------------------------------
FROM base AS mapping
# TODO mapping deps (FAST-LIO2, GTSAM, PCL)

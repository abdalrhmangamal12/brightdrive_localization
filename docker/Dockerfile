# docker/Dockerfile — ROS 2 Humble + the CARLA 0.9.15 Python API. Later steps add layers at the bottom.
# Why Humble (Ubuntu 22.04, Python 3.10): the task requires Humble, and CARLA 0.9.15 publishes Linux wheels for
# Python 3.7-3.10 only.
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

# Python libraries come from apt, not pip, so numpy stays at the 1.21.5 that ROS Humble is built against.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# CARLA Python API: the cp310 wheel from PyPI, pinned. --no-deps so pip cannot pull in a second numpy.
RUN python3 -m pip install --no-cache-dir --no-deps carla==0.9.15

# Fail the build now, not mid-session: the wheel imports next to apt's numpy.
RUN python3 -c "import carla, numpy; from importlib.metadata import version; \
    print('carla', version('carla'), carla.__file__); print('numpy', numpy.__version__, numpy.__file__)"

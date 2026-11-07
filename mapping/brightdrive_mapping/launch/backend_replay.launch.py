# The backend on a recorded FAST-LIO run: backend_node plus `ros2 bag play` of a bag that carries /Odometry,
# /cloud_registered_body and /gnss/fix. When the bag ends the launch waits so the queued scans are processed,
# then shuts down; the SIGINT makes the node leave spin(), and main() saves the map through stop().
#
#   ros2 launch brightdrive_mapping backend_replay.launch.py [bag:=/bags/<run>] [rate:=2] [save_dir:=/maps]
#
# The run bags have no /clock (the recorder stamped wall time), so the node runs on wall time; every header the
# backend reads is sim time and only those stamps are used for pairing, TF and the trajectory.
import os
import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, Shutdown,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Seconds between the backend start and the first bag message: the subscriptions must exist before the bag's
# first /Odometry, or the first scans have no pose to pair with.
BACKEND_WARMUP_S = 3.0
# Seconds after `ros2 bag play` exits before the shutdown: scans still queued in the executor get processed
# and the last GNSS fixes attach to the last keyframe.
BAG_END_GRACE_S = 5.0
# The shutdown SIGINT starts saveMap (batch solve + a map of ~10^7 points). launch escalates to SIGTERM and
# SIGKILL after these many seconds; the default 5 s would kill the save.
SAVE_TIMEOUT_S = '180'

# Where the two code bases live, for the commits map.yaml records. FAST-LIO is the clone the Docker image built
# (FAST_LIO_WS is set in the image); this package is wherever this launch file is checked out.
FRONTEND_SRC = os.path.join(os.environ.get('FAST_LIO_WS', '/opt/fast_lio_ws'), 'src', 'fast_lio')
BACKEND_SRC = os.path.dirname(os.path.realpath(__file__))


def git_short_head(path):
    """Short commit of the repository containing path, '' when git or the repository is not there.
    safe.directory: the FAST-LIO clone belongs to root and the container runs as the host user."""
    try:
        return subprocess.run(['git', '-c', f'safe.directory={path}', '-C', path, 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('brightdrive_mapping'), 'config', 'backend.yaml'])

    backend = Node(
        package='brightdrive_mapping',
        executable='backend_node',
        name='mapping_backend',
        output='screen',
        parameters=[config, {
            'save_dir': LaunchConfiguration('save_dir'),
            'use_sim_time': False,
            'source_bag': LaunchConfiguration('bag'),
            'frontend_commit': git_short_head(FRONTEND_SRC),
            'backend_commit': git_short_head(BACKEND_SRC),
        }],
        sigterm_timeout=SAVE_TIMEOUT_S,
        sigkill_timeout=SAVE_TIMEOUT_S,
    )

    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
             '--rate', LaunchConfiguration('rate'),
             '--disable-keyboard-controls'],   # no TTY under launch
        output='screen',
    )

    shutdown_after_bag = RegisterEventHandler(OnProcessExit(
        target_action=play,
        on_exit=[
            LogInfo(msg=f'bag replay finished, shutting down in {BAG_END_GRACE_S:.0f} s'),
            TimerAction(period=BAG_END_GRACE_S, actions=[Shutdown(reason='bag replay finished')]),
        ],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='/bags/fastlio_run_20261105_133549',
                              description='rosbag2 directory with /Odometry, /cloud_registered_body, /gnss/fix'),
        DeclareLaunchArgument('rate', default_value='2', description='ros2 bag play --rate'),
        DeclareLaunchArgument('save_dir', default_value='/maps', description='where saveMap writes'),
        backend,
        TimerAction(period=BACKEND_WARMUP_S, actions=[play]),
        shutdown_after_bag,
    ])

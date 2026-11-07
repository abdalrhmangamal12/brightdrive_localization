# The whole mapping chain from a raw CARLA bag, in one command: FAST-LIO2 (front-end, unmodified) and the backend on
# `ros2 bag play` of a bag with /points_raw, /imu/data, /gnss/fix and /clock. When the bag ends the launch waits for
# the last scans, calls /mapping/save_map (scripts/save_map.py), with run_dynamic_filter:=true runs
# scripts/dynamic_filter.py on save_dir, and shuts down. Same tail as backend_replay.launch.py, which starts from a
# recorded FAST-LIO run instead.
#
#   ros2 launch brightdrive_mapping mapping_pipeline.launch.py bag:=/bags/<raw mapping bag> save_dir:=/maps/<run>
#       [rate:=0.5] [start_offset:=2.0] [save_keyframes:=true] [run_dynamic_filter:=true] [loop_enable:=true]
#
# start_offset: seconds of the bag skipped. A scenario bag starts while the car is still dropping onto the road
# after its spawn; FAST-LIO initializes its IMU on the first samples and diverges from that drop. The car then stands
# still for 5 s (StartDelay), so 2 s lands FAST-LIO's initialization inside the standstill.
#
# rate 0.5: FAST-LIO keeps up with the 64-channel sweeps and the loop thread sees every keyframe (measured on the
# mapping run of 2026-11-05). The bag's /clock drives both nodes (use_sim_time).
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Seconds between the node starts and the first bag message (FAST-LIO and the backend subscribe first).
WARMUP_S = 5.0
# Seconds after the bag ends before the save: FAST-LIO's queue drains, the last keyframes and fixes reach the graph.
BAG_END_GRACE_S = 10.0
# The shutdown save of a full map can take minutes; launch's default 5 s before SIGKILL would cut it.
SAVE_TIMEOUT_S = '180'
FRONTEND_SRC = os.path.join(os.environ.get('FAST_LIO_WS', '/opt/fast_lio_ws'), 'src', 'fast_lio')
BACKEND_SRC = os.path.dirname(os.path.realpath(__file__))
# Only what the chain reads: a raw scenario bag also carries /eval/* (ground truth stays out of the chain).
PLAY_TOPICS = ['/clock', '/points_raw', '/imu/data', '/gnss/fix']


def git_short_head(path):
    try:
        return subprocess.run(['git', '-c', f'safe.directory={path}', '-C', path, 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def generate_launch_description():
    share = FindPackageShare('brightdrive_mapping')
    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[PathJoinSubstitution([share, 'config', 'fastlio_carla.yaml']), {'use_sim_time': True}],
    )
    backend = Node(
        package='brightdrive_mapping',
        executable='backend_node',
        name='mapping_backend',
        output='screen',
        parameters=[PathJoinSubstitution([share, 'config', 'backend.yaml']), {
            'save_dir': LaunchConfiguration('save_dir'),
            'use_sim_time': True,
            'source_bag': LaunchConfiguration('bag'),
            'frontend_commit': git_short_head(FRONTEND_SRC),
            'backend_commit': git_short_head(BACKEND_SRC),
            'save_keyframes': ParameterValue(LaunchConfiguration('save_keyframes'), value_type=bool),
            'loop_enable': ParameterValue(LaunchConfiguration('loop_enable'), value_type=bool),
        }],
        sigterm_timeout=SAVE_TIMEOUT_S,
        sigkill_timeout=SAVE_TIMEOUT_S,
    )
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--rate', LaunchConfiguration('rate'),
             '--start-offset', LaunchConfiguration('start_offset'), '--disable-keyboard-controls',
             '--topics'] + PLAY_TOPICS,
        name='play', output='screen',
    )
    save_map = ExecuteProcess(cmd=['ros2', 'run', 'brightdrive_mapping', 'save_map.py'], name='save_map',
                              output='screen')
    dynamic_filter = ExecuteProcess(
        cmd=['ros2', 'run', 'brightdrive_mapping', 'dynamic_filter.py', LaunchConfiguration('save_dir')],
        name='dynamic_filter', output='screen')

    def after_save(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='save_map failed, shutting down'), Shutdown(reason='save_map failed')]
        if context.perform_substitution(LaunchConfiguration('run_dynamic_filter')).lower() in ('true', '1'):
            return [LogInfo(msg='map saved, running the dynamic-object filter'), dynamic_filter]
        return [LogInfo(msg='map saved, shutting down'), Shutdown(reason='map saved')]

    def after_filter(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'dynamic filter failed (exit {event.returncode})'),
                    Shutdown(reason='dynamic filter failed')]
        return [LogInfo(msg='dynamic filter finished'), Shutdown(reason='dynamic filter finished')]

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='raw bag with /points_raw, /imu/data, /gnss/fix, /clock'),
        DeclareLaunchArgument('rate', default_value='0.5', description='ros2 bag play --rate'),
        DeclareLaunchArgument('start_offset', default_value='2.0',
                              description='seconds of the bag skipped (the spawn drop, see the header)'),
        DeclareLaunchArgument('save_dir', default_value='/maps', description='where saveMap writes'),
        DeclareLaunchArgument('save_keyframes', default_value='true',
                              description='also write <save_dir>/keyframes/NNNNNN.pcd (the dynamic filter needs them)'),
        DeclareLaunchArgument('loop_enable', default_value='true', description='loop-closure thread on or off'),
        DeclareLaunchArgument('run_dynamic_filter', default_value='true',
                              description='after the save, run scripts/dynamic_filter.py on save_dir'),
        fastlio,
        backend,
        TimerAction(period=WARMUP_S, actions=[play]),
        RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
            LogInfo(msg=f'bag finished, calling save_map in {BAG_END_GRACE_S:.0f} s'),
            TimerAction(period=BAG_END_GRACE_S, actions=[save_map])])),
        RegisterEventHandler(OnProcessExit(target_action=save_map, on_exit=after_save)),
        RegisterEventHandler(OnProcessExit(target_action=dynamic_filter, on_exit=after_filter)),
        RegisterEventHandler(OnProcessExit(target_action=backend, on_exit=[Shutdown(reason='backend exited')])),
        RegisterEventHandler(OnProcessExit(target_action=fastlio, on_exit=[Shutdown(reason='FAST-LIO exited')])),
    ])

# The localizer on a recorded scenario bag: localization_node, then `ros2 bag play` once the map is loaded, and
# optionally `ros2 bag record` of the outputs (plus the bag's ground truth for the evaluation). When the bag
# ends the launch waits a moment so the last sweep is processed, then shuts down.
#
#   ros2 launch brightdrive_localization localization_replay.launch.py bag:=/bags/<scenario bag>
#       [rate:=1.0] [record:=/bags/loc_runs/<name>] [params_file:=<yaml>]
#
# The scenario bags carry /clock, so the node runs on sim time and every output is stamped with data time.
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
                            Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Seconds between the node start and the first bag message: map load, covariances and the two voxel maps take
# about 3 s; the subscriptions exist only after that.
NODE_WARMUP_S = 8.0
# Seconds after the bag ends before shutdown: the last sweep and the recorder's queue drain.
BAG_END_GRACE_S = 3.0
# What a replay records: the outputs, the status, TF, the debug match stream, the IMU (for end-to-end latency
# on one recorder clock) and the bag's ground truth (evaluation only; the node never subscribes to /eval).
RECORD_TOPICS = ['/localization/pose', '/localization/velocity', '/localization/odometry',
                 '/localization/status', '/localization/path', '/localization/debug/match',
                 '/tf', '/tf_static', '/clock', '/imu/data', '/eval/ground_truth', '/eval/gnss_outage',
                 '/eval/init_offset', '/initialpose']


def recorder(context):
    out = context.perform_substitution(LaunchConfiguration('record'))
    if not out:
        return []
    return [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + RECORD_TOPICS,
                           name='record', output='log')]


def generate_launch_description():
    share = FindPackageShare('brightdrive_localization')
    node = Node(
        package='brightdrive_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--rate', LaunchConfiguration('rate'),
             '--disable-keyboard-controls'],
        name='play', output='screen',
    )
    on_bag_end = RegisterEventHandler(OnProcessExit(
        target_action=play,
        on_exit=[LogInfo(msg=f'bag finished, shutting down in {BAG_END_GRACE_S:.0f} s'),
                 TimerAction(period=BAG_END_GRACE_S, actions=[Shutdown(reason='bag finished')])],
    ))
    on_node_exit = RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[Shutdown(reason='localization_node exited')]))
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('record', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([share, 'config', 'localization.yaml'])),
        node,
        OpaqueFunction(function=recorder),
        TimerAction(period=NODE_WARMUP_S, actions=[play]),
        on_bag_end,
        on_node_exit,
    ])

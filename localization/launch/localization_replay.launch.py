# The localizer on a recorded scenario bag: localization_node, then `ros2 bag play` once the map is loaded, and
# optionally `ros2 bag record` of the outputs (plus the bag's ground truth for the evaluation). When the bag
# ends the launch waits a moment so the last sweep is processed, then shuts down.
#
#   ros2 launch brightdrive_localization localization_replay.launch.py bag:=/bags/<scenario bag>
#       [rate:=1.0] [record:=/bags/loc_runs/<name>] [params_file:=<yaml>]
#       [start_offset:=<s into the bag>] [play_duration:=<s of bag, 0 = to the end>]
#
# start_offset starts the localizer on data from the middle of a run (the car already driving).
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


# What a replay plays: the sensors, the clock and the evaluation topics. A bag recorded during a live run also holds
# that run's /localization/* and /tf; playing them would put two localizers' outputs on the same topics.
PLAY_TOPICS = ['/clock', '/imu/data', '/gnss/fix', '/gnss/fix_gated', '/points_raw', '/tf_static', '/initialpose',
               '/eval/ground_truth', '/eval/gnss_outage', '/eval/init_offset']


def recorder(context):
    out = context.perform_substitution(LaunchConfiguration('record'))
    if not out:
        return []
    return [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + RECORD_TOPICS,
                           name='record', output='log')]


def player(context):
    """The bag player, shutdown when the bag ends, and after play_duration s of bag when that is set."""
    bag = context.perform_substitution(LaunchConfiguration('bag'))
    rate = float(context.perform_substitution(LaunchConfiguration('rate')))
    offset = float(context.perform_substitution(LaunchConfiguration('start_offset')))
    duration = float(context.perform_substitution(LaunchConfiguration('play_duration')))
    cmd = ['ros2', 'bag', 'play', bag, '--rate', str(rate), '--disable-keyboard-controls']
    if offset > 0.0:
        cmd += ['--start-offset', str(offset)]
    play = ExecuteProcess(cmd=cmd + ['--topics'] + PLAY_TOPICS, name='play', output='screen')
    actions = [play, RegisterEventHandler(OnProcessExit(
        target_action=play,
        on_exit=[LogInfo(msg=f'bag finished, shutting down in {BAG_END_GRACE_S:.0f} s'),
                 TimerAction(period=BAG_END_GRACE_S, actions=[Shutdown(reason='bag finished')])],
    ))]
    if duration > 0.0:
        actions.append(TimerAction(period=duration / rate,
                                   actions=[Shutdown(reason=f'played {duration:.0f} s of the bag')]))
    return actions


def generate_launch_description():
    share = FindPackageShare('brightdrive_localization')
    node = Node(
        package='brightdrive_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    on_node_exit = RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[Shutdown(reason='localization_node exited')]))
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('record', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([share, 'config', 'localization.yaml'])),
        DeclareLaunchArgument('start_offset', default_value='0.0'),
        DeclareLaunchArgument('play_duration', default_value='0.0'),
        node,
        OpaqueFunction(function=recorder),
        TimerAction(period=NODE_WARMUP_S, actions=[OpaqueFunction(function=player)]),
        on_node_exit,
    ])

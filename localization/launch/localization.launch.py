# The localizer alone: localization_node with config/localization.yaml, for a live CARLA run or a bag that is
# played separately (the bags carry /clock, so use_sim_time stays true).
#
#   ros2 launch brightdrive_localization localization.launch.py [params_file:=<yaml>] [rviz:=true]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('brightdrive_localization')
    node = Node(
        package='brightdrive_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'localization.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([share, 'config', 'localization.yaml'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        node,
        rviz,
    ])

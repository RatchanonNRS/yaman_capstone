"""
slam.launch.py — Run on RPi to build a map
Starts: bringup + slam_toolbox (online async mode)
Drive around with teleop to build the map.
Map is auto-saved to /home/yaman/agv_map on shutdown.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_controller'),
            'launch', 'bringup.launch.py'))
    )

    slam_params = os.path.join(
        get_package_share_directory('robot_controller'),
        'config', 'slam_params.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_lifecycle_manager': False}],
        output='screen',
    )

    # Automatically configure then activate slam_toolbox after it starts
    # Note: RPi needs ~10s to bring up slam_toolbox — shorter timers cause "Node not found"
    configure = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen',
        )]
    )

    activate = TimerAction(
        period=16.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen',
        )]
    )

    return LaunchDescription([bringup, slam, configure, activate])

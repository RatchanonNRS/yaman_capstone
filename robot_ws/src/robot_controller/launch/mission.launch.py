"""
mission.launch.py — bringup + mission_node

Parameters (override at command line):
  target_distance:=3.0
  forward_speed:=0.15
  obstacle_stop_distance:=0.45

Example:
  ros2 launch robot_controller mission.launch.py target_distance:=3.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_controller'),
            'launch', 'bringup.launch.py'))
    )

    mission = Node(
        package='robot_controller',
        executable='mission_node',
        name='mission_node',
        parameters=[{
            'target_distance':        LaunchConfiguration('target_distance'),
            'forward_speed':          LaunchConfiguration('forward_speed'),
            'obstacle_stop_distance': LaunchConfiguration('obstacle_stop_distance'),
            'check_angle_deg':        LaunchConfiguration('check_angle_deg'),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_distance',        default_value='2.0'),
        DeclareLaunchArgument('forward_speed',          default_value='0.15'),
        DeclareLaunchArgument('obstacle_stop_distance', default_value='0.45'),
        DeclareLaunchArgument('check_angle_deg',        default_value='30.0'),
        bringup,
        mission,
    ])

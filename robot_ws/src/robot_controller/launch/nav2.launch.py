"""
nav2.launch.py — Run on RPi to navigate with a saved map
Starts: bringup + AMCL localisation + Nav2
Requires: map saved at /home/yaman/agv_map.yaml (run slam.launch.py first)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_controller'),
            'launch', 'bringup.launch.py'))
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(
        get_package_share_directory('robot_controller'),
        'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': '/home/yaman/agv_map.yaml',
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([bringup, nav2])

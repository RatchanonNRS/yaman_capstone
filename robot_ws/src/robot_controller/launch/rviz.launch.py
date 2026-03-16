"""
rviz.launch.py — Run on VM (not RPi)
Opens RViz2 preconfigured to show map, robot model, laser scan, and path.
The VM and RPi must be on the same network with the same ROS_DOMAIN_ID (default 0).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('robot_controller'),
        'config', 'agv.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

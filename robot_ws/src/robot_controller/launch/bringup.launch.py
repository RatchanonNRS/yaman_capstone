"""
bringup.launch.py — Run on RPi
Starts: robot_state_publisher + serial_bridge + RPLidar C1
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('agv'), 'urdf', 'agv.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Publishes URDF transforms (base_footprint→base_link, base_link→laser_frame, wheels)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Arduino serial bridge — publishes /odom, /imu/data, TF odom→base_footprint
        Node(
            package='robot_controller',
            executable='serial_bridge',
            parameters=[{
                'serial_port': '/dev/ttyUSBArduinoMega',
                'baud_rate': 115200,
            }],
            output='screen',
        ),

        # RPLidar C1 — publishes /scan on frame laser_frame
        Node(
            name='rplidar',
            package='rplidar_ros',
            executable='rplidar_composition',
            parameters=[{
                'serial_port': '/dev/ttyUSBlidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),
    ])

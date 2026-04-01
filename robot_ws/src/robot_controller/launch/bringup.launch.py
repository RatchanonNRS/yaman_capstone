"""
bringup.launch.py — Run on RPi
Starts: robot_state_publisher + serial_bridge + RPLidar C1 (via sllidar_ros2)
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

        # Publishes zero joint states for wheel joints R and L (continuous type)
        # Required so robot_state_publisher can broadcast link_R and Link_L TF
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Arduino serial bridge — publishes /odom, TF odom→base_footprint
        Node(
            package='robot_controller',
            executable='serial_bridge',
            parameters=[{
                'serial_port': '/dev/ttyUSBArduinoMega',
                'baud_rate': 115200,
            }],
            output='screen',
        ),

        # MPU6050 via RPi I2C — publishes /imu/data at 50 Hz
        Node(
            package='robot_controller',
            executable='imu_node',
            parameters=[{
                'i2c_bus':     1,
                'i2c_address': 0x68,
            }],
            output='screen',
        ),

        # Safety watchdog — stops motors when person detected in path
        Node(
            package='robot_controller',
            executable='safety_node',
            output='screen',
        ),

        # RPLidar C1 via sllidar_ros2 — publishes /scan on frame laser_frame
        Node(
            name='sllidar_node',
            package='sllidar_ros2',
            executable='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSBlidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),
    ])

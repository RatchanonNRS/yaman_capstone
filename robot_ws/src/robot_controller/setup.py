from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bringup.launch.py',
             'launch/slam.launch.py',
             'launch/nav2.launch.py',
             'launch/rviz.launch.py',
             'launch/mission.launch.py']),
        ('share/' + package_name + '/config',
            ['config/slam_params.yaml',
             'config/nav2_params.yaml',
             'config/agv.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaman',
    maintainer_email='yaman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge  = robot_controller.serial_bridge:main',
            'imu_node       = robot_controller.imu_node:main',
            'mission_node   = robot_controller.mission_node:main',
            'safety_node    = robot_controller.safety_node:main',
        ],
    },
)

#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# install_ros2.sh  —  ROS2 Jazzy + AGV dependencies setup for Raspberry Pi
#                     Ubuntu 24.04 LTS Server
# Run once on the RPi:  bash install_ros2.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

echo "=== [1/6] Set locale ==="
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== [2/6] Add ROS2 Jazzy apt repository ==="
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "=== [3/6] Install ROS2 Jazzy (base — no GUI, saves RAM on RPi) ==="
sudo apt install -y ros-jazzy-ros-base

echo "=== [4/6] Install AGV-specific packages ==="
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rplidar-ros \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

echo "=== [5/6] Install Python serial library (for Arduino serial bridge) ==="
sudo apt install -y python3-serial

echo "=== [6/6] Source ROS2 in bashrc ==="
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
grep -qxF 'source ~/robot_ws/install/setup.bash' ~/.bashrc || \
    echo 'source ~/robot_ws/install/setup.bash' >> ~/.bashrc

echo ""
echo "=== Done! ==="
echo "Now run:"
echo "  source ~/.bashrc"
echo "  cd ~/robot_ws && colcon build"

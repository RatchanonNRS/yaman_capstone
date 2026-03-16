# AGV Capstone — Claude Session Context

Read this file at the start of every session to understand the full project state.

---

## Project Goal
Autonomous Ground Vehicle (AGV) that navigates via mission (home → destination) using ROS2 Nav2 + SLAM.

---

## Hardware

| Component | Detail |
|---|---|
| Computer | Raspberry Pi, Ubuntu 24.04 LTS Server |
| Microcontroller | Arduino Mega (unofficial, CH340 chip) |
| Lidar | RPLidar C1 |
| Motor Driver | Cytron MDD20A |
| IMU | MPU6050 clone at I2C address 0x70 |
| Encoders | E6B2-CWZ6C 600P/R |

**Wiring:**
- Encoder Left:  OutA→D2,  OutB→D3
- Encoder Right: OutA→D18, OutB→D19
- Motor Left:    PWM→D6,   DIR→D52
- Motor Right:   PWM→D7,   DIR→D53
- IMU:           SCL→D21,  SDA→D20

**Gear train:**
- Motor gear 9T → Encoder gear 9T (1:1)
- Motor gear 9T → Wheel gear 32T
- Counts per wheel rev = 600 × 4 × (32/9) ≈ 8533.33
- Wheel diameter = 200mm, circumference = 628.3mm
- Wheelbase (from SolidWorks CAD) = 0.445m (user originally said 400mm but CAD shows 445mm)

**USB device names (udev rules set, permanent):**
- Arduino Mega  → `/dev/ttyUSBArduinoMega`  (was ttyUSB0, VID=1a86, PID=7523, CH340)
- RPLidar C1    → `/dev/ttyUSBlidar`         (was ttyUSB1, VID=10c4, PID=ea60, CP2102N)
- udev rule file: `/etc/udev/rules.d/99-agv.rules` on RPi

**Network access:**
- RPi IP: 192.168.137.50, user: yaman, password: 12345678
- SSH key already set up: `ssh -i ~/.ssh/id_rpi yaman@192.168.137.50`
- alias `rpi` = ssh to RPi, `pimount` = sshfs mount to ~/pi

---

## Repository Structure

```
~/yaman_capstone/                        ← on RPi at /home/yaman/yaman_capstone/
                                           mounted on VM at /home/yaman/pi/yaman_capstone/
├── CLAUDE.md                            ← this file
├── yamancode.cpp                        ← Arduino Mega firmware (upload via Arduino IDE)
├── install_ros2.sh                      ← ROS2 Jazzy install script (already run on RPi)
└── robot_ws/
    └── src/
        ├── agv/                         ← Robot URDF package (converted from SolidWorks ROS1→ROS2)
        │   ├── urdf/agv.urdf            ← URDF with base_footprint, base_link, laser_frame, wheels
        │   ├── meshes/                  ← STL files from SolidWorks
        │   ├── launch/display.launch.py
        │   ├── package.xml              ← ament_cmake, ROS2
        │   └── CMakeLists.txt           ← ament_cmake
        ├── sllidar_ros2/                ← Cloned from Slamtec GitHub (built from source, not in apt)
        └── robot_controller/            ← Main ROS2 Python package
            ├── robot_controller/
            │   ├── __init__.py
            │   └── serial_bridge.py     ← KEY FILE: Arduino↔ROS2 bridge node
            ├── launch/
            │   ├── bringup.launch.py    ← robot_state_publisher + serial_bridge + sllidar_ros2
            │   ├── slam.launch.py       ← bringup + slam_toolbox (for mapping)
            │   ├── nav2.launch.py       ← bringup + Nav2 (for navigation, needs saved map)
            │   └── rviz.launch.py       ← RViz2 (run on VM, not RPi)
            ├── config/
            │   ├── slam_params.yaml     ← slam_toolbox config, saves map to ~/agv_map
            │   ├── nav2_params.yaml     ← Nav2 full config (AMCL, DWB, costmaps, etc.)
            │   └── agv.rviz             ← RViz2 config (map, scan, odom, TF, robot model)
            ├── setup.py                 ← entry_point: serial_bridge
            └── package.xml
```

---

## Arduino Firmware (`yamancode.cpp`)

- **Library needed:** MPU6050 by Electronic Cats (search "MPU6050" in Arduino Library Manager)
- **IMU address:** 0x70 (unusual — device is a clone, registers match MPU6050)
- **IMU status:** `ERR:IMU_NOT_FOUND` — library cannot connect at 0x70. IMU skipped for now.
  - Robot works without IMU — SLAM uses lidar + odometry only
- **Serial protocol (115200 baud):**
  - Receive: `V:<vx>,<wz>\n` — velocity in m/s and rad/s from ROS2
  - Send:    `O:<x>,<y>,<th>,<vl>,<vr>\n` — odometry at 50Hz
  - Send:    `I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n` — IMU at 50Hz (only if IMU found)
- **Keyboard teleop chars** (send single char over serial): `i`=fwd, `,`=back, `j`=left, `l`=right, `k`=stop
- **PID velocity control** per wheel (gains: Kp=150, Ki=80, Kd=3 — need tuning on real robot)
- **Watchdog:** stops motors if no command for 500ms
- **Odometry confirmed working** — O: lines streaming at 50Hz, values change with wheel movement

---

## Serial Bridge Node (`serial_bridge.py`)

Runs on RPi. Bridges Arduino serial ↔ ROS2:
- Subscribes `/cmd_vel` → sends `V:<vx>,<wz>\n` to Arduino
- Reads `O:` lines → publishes `nav_msgs/Odometry` on `/odom` + broadcasts `odom→base_footprint` TF
- Reads `I:` lines → publishes `sensor_msgs/Imu` on `/imu/data`
- Default serial port: `/dev/ttyUSBArduinoMega`

---

## Lidar

- Package: **sllidar_ros2** (built from source in robot_ws/src/sllidar_ros2)
  - `rplidar_ros` from apt does NOT work with C1 (error: `Cannot start scan: 80008002`)
  - `sllidar_ros2` confirmed working — user tested and saw scan in RViz2
- Launch node: `sllidar_node` from `sllidar_ros2` package
- Port: `/dev/ttyUSBlidar`, baudrate: 460800, scan_mode: `Standard`
- Frame ID: `laser_frame` (matches URDF)
- Publishes: `/scan`

---

## TF Tree

```
odom
 └── base_footprint      ← published by serial_bridge (dynamic, from odometry)
      └── base_link       ← published by robot_state_publisher (fixed, from URDF, z=+0.213m)
           ├── link_R     ← right wheel
           ├── Link_L     ← left wheel
           └── laser_frame ← RPLidar C1 (fixed, z=+0.041m above base_link)
```

---

## ROS2 Setup on RPi

- ROS2 Jazzy installed ✅
- Workspace: `~/yaman_capstone/robot_ws/`
- All packages built: agv, robot_controller, sllidar_ros2 ✅
- `~/.bashrc` sources the workspace automatically ✅
- Installed packages: slam_toolbox, nav2_bringup, rplidar_ros (unused), teleop_twist_keyboard, python3-serial

---

## ROS2 Setup on VM

- ROS2 Jazzy installed ✅
- Workspace: `~/pi/yaman_capstone/robot_ws/` (sshfs mount of RPi)
- Build: `cd ~/pi/yaman_capstone/robot_ws && colcon build` (done)
- Source: `source ~/pi/yaman_capstone/robot_ws/install/setup.bash`
- Note: VM does NOT have sllidar_ros2 built — run `colcon build` on VM too after any changes

---

## Current Status

### ✅ Working
- Arduino odometry (`O:` lines) streaming at 50Hz, values change with movement
- Serial bridge opens Arduino port successfully
- robot_state_publisher running
- udev symlinks (`/dev/ttyUSBArduinoMega`, `/dev/ttyUSBlidar`) permanent
- sllidar_ros2 built and confirmed working (user saw scan in RViz2)
- bringup.launch.py updated to use sllidar_ros2

### ❌ Problem 1: /odom not publishing to ROS topics
- `ros2 topic info /odom` shows Publisher count: 1 (bridge IS registered)
- But `ros2 topic echo /odom` times out — no messages flowing
- Arduino IS sending O: lines (confirmed via direct python3 serial test)
- **Suspected cause:** serial_bridge read thread issue — thread may be blocked or not calling publish correctly
- **Next debug step:** Add logger prints inside `_handle_odom`, check if thread is alive, check rclpy threading

### ❌ Problem 2: IMU not working
- Device at 0x70 does not respond to MPU6050 library commands
- Skipped for now — not needed for SLAM/Nav2

---

## Run Commands

```bash
# On RPi — bringup (serial bridge + lidar + robot_state_publisher)
ros2 launch robot_controller bringup.launch.py

# On RPi — SLAM mapping (drive around to build map)
ros2 launch robot_controller slam.launch.py

# On RPi — Navigation with saved map
ros2 launch robot_controller nav2.launch.py

# On VM — RViz2 visualization
source ~/pi/yaman_capstone/robot_ws/install/setup.bash
ros2 launch robot_controller rviz.launch.py

# Teleop keyboard (any machine)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Rebuild workspace — RPi
cd ~/yaman_capstone/robot_ws && colcon build

# Rebuild workspace — VM
cd ~/pi/yaman_capstone/robot_ws && colcon build
```

---

## Priority Fix Order for Next Session

1. **Fix /odom not publishing** — debug serial_bridge read thread, add logger output inside `_handle_odom`
2. **First full bringup test** — verify /odom and /scan both publish, check TF tree in RViz
3. **PID tuning** — test motor response with teleop, tune Kp/Ki/Kd
4. **SLAM mapping** — drive around, build and save map to ~/agv_map
5. **Nav2 navigation** — set goal in RViz, verify autonomous navigation
6. **IMU (optional)** — investigate actual chip at 0x70, may need different library

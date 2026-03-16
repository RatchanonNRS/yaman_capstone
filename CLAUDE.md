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
| IMU | MPU9250 |
| Encoders | E6B2-CWZ6C 600P/R |

**Wiring:**
- Encoder Left:  OutA→D2,  OutB→D3
- Encoder Right: OutA→D18, OutB→D19
- Motor Left:    PWM→D6,   DIR→D52
- Motor Right:   PWM→D7,   DIR→D53
- IMU MPU9250:   SCL→D21,  SDA→D20

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
        └── robot_controller/            ← Main ROS2 Python package
            ├── robot_controller/
            │   ├── __init__.py
            │   └── serial_bridge.py     ← KEY FILE: Arduino↔ROS2 bridge node
            ├── launch/
            │   ├── bringup.launch.py    ← robot_state_publisher + serial_bridge + rplidar
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

- **Library needed:** MPU9250 by hideakitai (install via Arduino IDE Library Manager)
- **Serial protocol (115200 baud):**
  - Receive: `V:<vx>,<wz>\n` — velocity in m/s and rad/s from ROS2
  - Send:    `O:<x>,<y>,<th>,<vl>,<vr>\n` — odometry at 50Hz
  - Send:    `I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n` — IMU at 50Hz
- **Keyboard teleop chars** (send single char over serial): `i`=fwd, `,`=back, `j`=left, `l`=right, `k`=stop
- **PID velocity control** per wheel (gains: Kp=150, Ki=80, Kd=3 — need tuning on real robot)
- **Watchdog:** stops motors if no command for 500ms

---

## Serial Bridge Node (`serial_bridge.py`)

Runs on RPi. Bridges Arduino serial ↔ ROS2:
- Subscribes `/cmd_vel` → sends `V:<vx>,<wz>\n` to Arduino
- Reads `O:` lines → publishes `nav_msgs/Odometry` on `/odom` + broadcasts `odom→base_footprint` TF
- Reads `I:` lines → publishes `sensor_msgs/Imu` on `/imu/data`
- Default serial port: `/dev/ttyUSBArduinoMega`

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
- Both packages built successfully ✅
- `~/.bashrc` sources the workspace automatically ✅
- Installed packages: slam_toolbox, nav2_bringup, rplidar_ros, teleop_twist_keyboard, python3-serial

---

## ROS2 Setup on VM

- ROS2 Jazzy installed ✅
- Workspace: `~/pi/yaman_capstone/robot_ws/` (sshfs mount of RPi)
- Build: `cd ~/pi/yaman_capstone/robot_ws && colcon build` (done, clean build)
- Source: `source ~/pi/yaman_capstone/robot_ws/install/setup.bash`

---

## Current Status / What Was Being Debugged

### ✅ Working
- Arduino sends `O:` lines at 50Hz (confirmed via python3 serial read test)
- Serial bridge opens port successfully
- robot_state_publisher running
- udev symlinks working

### ❌ Problem 1: /odom not publishing to ROS topics
- `ros2 topic info /odom` shows Publisher count: 1 (bridge IS registered)
- But `ros2 topic echo /odom` times out — no messages flowing
- **Suspected cause:** Issue in serial_bridge read thread not forwarding parsed data to ROS publisher
- **Next debug step:** Check if serial_bridge is parsing `O:` lines correctly, add debug prints, or check if thread is alive

### ❌ Problem 2: RPLidar C1 scan error
- Error: `Cannot start scan: '80008002'` (RESULT_OPERATION_NOT_SUPPORT)
- The `rplidar.launch.py` (generic) doesn't work for C1
- **Fix to try:** Add `scan_mode: 'DenseBoost'` parameter to rplidar node in bringup.launch.py
- OR try baudrate 115200 instead of 460800

### ❌ Problem 3: IMU not found
- Arduino reports: `I2C ERROR CODE: 2` and `ERR:IMU_NOT_FOUND`
- MPU9250 I2C address might be 0x69 (AD0 pin high) instead of 0x68
- **Fix:** Change `imu.setup(0x68)` to `imu.setup(0x69)` in yamancode.cpp and re-upload

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

# Rebuild workspace (run on either RPi or VM)
cd ~/yaman_capstone/robot_ws   # RPi path
cd ~/pi/yaman_capstone/robot_ws  # VM path
colcon build
```

---

## Priority Fix Order for Next Session

1. **Fix serial_bridge /odom issue** — debug why parsed O: lines aren't being published
2. **Fix RPLidar scan mode** — add `scan_mode: 'DenseBoost'` to bringup.launch.py
3. **Fix IMU address** — try 0x69 in yamancode.cpp
4. **First full test** — verify /odom and /scan both publishing, check RViz
5. **PID tuning** — test motor control, tune Kp/Ki/Kd
6. **SLAM mapping** — drive around, build and save map
7. **Nav2 navigation** — set goal in RViz, verify autonomous navigation

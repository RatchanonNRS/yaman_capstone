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
| IMU | MPU6050 clone — I2C address **0x68**, WHO_AM_I returns **0x70** |
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
- Wheelbase (from SolidWorks CAD) = 0.445m

**USB device names (udev rules set, permanent):**
- Arduino Mega  → `/dev/ttyUSBArduinoMega`  (ttyUSB0, VID=1a86, PID=7523, CH340)
- RPLidar C1    → `/dev/ttyUSBlidar`         (ttyUSB1, VID=10c4, PID=ea60, CP2102N)
- udev rule file: `/etc/udev/rules.d/99-agv.rules` on RPi

**Network access:**
- RPi IP: 192.168.137.50, user: yaman, password: 12345678
- SSH key set up: `ssh -i ~/.ssh/id_rpi yaman@192.168.137.50`
- alias `rpi` = ssh to RPi, `pimount` = sshfs mount to ~/pi

---

## Repository Structure

```
~/yaman_capstone/                        ← RPi: /home/yaman/yaman_capstone/
                                           VM:  /home/yaman/pi/yaman_capstone/
├── CLAUDE.md                            ← this file
├── yamancode.cpp                        ← Arduino Mega firmware
├── install_ros2.sh                      ← ROS2 Jazzy install script (already run)
└── robot_ws/
    └── src/
        ├── agv/                         ← URDF package (SolidWorks → ROS2)
        │   ├── urdf/agv.urdf            ← base_footprint, base_link, laser_frame, wheels
        │   ├── meshes/                  ← STL files
        │   ├── launch/display.launch.py
        │   ├── package.xml              ← ament_cmake
        │   └── CMakeLists.txt
        ├── sllidar_ros2/                ← Cloned from Slamtec GitHub (built from source)
        └── robot_controller/            ← Main ROS2 Python package
            ├── robot_controller/
            │   ├── __init__.py
            │   └── serial_bridge.py     ← Arduino↔ROS2 bridge node
            ├── launch/
            │   ├── bringup.launch.py    ← robot_state_publisher + serial_bridge + sllidar
            │   ├── slam.launch.py       ← bringup + slam_toolbox
            │   ├── nav2.launch.py       ← bringup + Nav2 (needs saved map)
            │   └── rviz.launch.py       ← RViz2 (run on VM)
            ├── config/
            │   ├── slam_params.yaml     ← slam_toolbox, saves map to ~/agv_map
            │   ├── nav2_params.yaml     ← Nav2 full config
            │   └── agv.rviz             ← RViz2 config
            ├── setup.py                 ← entry_point: serial_bridge
            └── package.xml
```

---

## Arduino Firmware (`yamancode.cpp`)

- **Library:** MPU6050 by Electronic Cats (Arduino Library Manager)
- **IMU address:** `0x68` (clone returns WHO_AM_I=0x70 — skip `testConnection()`, just use `initialize()`)
- **Serial protocol (115200 baud):**
  - Receive: `V:<vx>,<wz>\n` — velocity m/s and rad/s
  - Send: `O:<x>,<y>,<th>,<vl>,<vr>\n` — odometry 50Hz
  - Send: `I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n` — IMU 50Hz
- **Keyboard teleop:** `i`=fwd, `,`=back, `j`=left, `l`=right, `k`=stop, `+/-`=speed
- **PID velocity control** per wheel (Kp=150, Ki=80, Kd=3 — needs tuning)
- **Watchdog:** stops motors if no command for 500ms

**IMU fix history:**
- Device at 0x68, WHO_AM_I = 0x70 (not standard 0x68)
- `testConnection()` fails because it compares WHO_AM_I to 0x68
- Fix: skip `testConnection()`, call `initialize()` directly, set `imu_ok = true`
- After fix: Serial Monitor shows `INFO:IMU_OK` + `I:` lines ✅

---

## Serial Bridge Node (`serial_bridge.py`)

- Subscribes `/cmd_vel` → sends `V:<vx>,<wz>\n` to Arduino
- Reads `O:` → publishes `nav_msgs/Odometry` on `/odom` + `odom→base_footprint` TF
- Reads `I:` → publishes `sensor_msgs/Imu` on `/imu/data`
- Watchdog: sends `V:0,0` if no /cmd_vel for 500ms
- Debug logging: logs every 50 odom messages received
- Port: `/dev/ttyUSBArduinoMega`

---

## Lidar

- Package: **sllidar_ros2** (built from source — `rplidar_ros` from apt fails on C1)
- Port: `/dev/ttyUSBlidar`, baudrate: 460800, scan_mode: `Standard`, frame: `laser_frame`
- **Current issue:** `SL_RESULT_OPERATION_TIMEOUT` — sllidar_node crashes on startup
- **Next debug step:** Check if lidar is spinning when bringup starts, try different scan_mode or baudrate

---

## TF Tree

```
odom
 └── base_footprint      ← serial_bridge (dynamic, odometry)
      └── base_link       ← robot_state_publisher (fixed, z=+0.213m)
           ├── link_R     ← right wheel
           ├── Link_L     ← left wheel
           └── laser_frame ← RPLidar C1 (fixed, z=+0.041m)
```

---

## ROS2 Setup

**RPi:**
- ROS2 Jazzy ✅
- Workspace: `~/yaman_capstone/robot_ws/`
- All packages built: agv, robot_controller, sllidar_ros2 ✅
- `~/.bashrc` auto-sources workspace ✅

**VM:**
- ROS2 Jazzy ✅
- Workspace: `~/pi/yaman_capstone/robot_ws/` (sshfs mount)
- Source: `source ~/pi/yaman_capstone/robot_ws/install/setup.bash`

---

## Current Status

### ✅ Confirmed Working
- Arduino odometry `O:` lines streaming at 50Hz
- Arduino IMU `I:` lines streaming at 50Hz (after fix)
- Serial bridge opens port, thread starts
- robot_state_publisher running
- udev symlinks permanent
- sllidar_ros2 built successfully

### ❌ Problem 1: `/odom` not publishing to ROS2
- Serial bridge opens port and thread starts
- But `ros2 topic echo /odom` times out — no messages
- Arduino IS sending data (confirmed via Serial Monitor)
- **Root cause suspected:** Multiple serial_bridge instances were splitting data before — need to confirm this is now fixed after killing all old processes
- **Next debug step:** Kill ALL ros2 processes, run ONE clean bringup, check `/tmp/bringup.log` for "Odom published x50" messages from serial_bridge debug logging

### ❌ Problem 2: sllidar_node timeout crash
- Error: `SL_RESULT_OPERATION_TIMEOUT` — node exits with code 255
- **Next debug step:** Check if lidar is physically spinning, try `scan_mode: 'DenseBoost'` or baudrate 115200

### ✅ Problem 3: IMU — SOLVED
- Was: `testConnection()` fails because clone WHO_AM_I=0x70 ≠ 0x68
- Fix: skip `testConnection()`, hardcode `imu_ok = true` after `initialize()`

---

## Run Commands

```bash
# RPi — bringup
ros2 launch robot_controller bringup.launch.py

# RPi — SLAM mapping
ros2 launch robot_controller slam.launch.py

# RPi — Nav2 navigation
ros2 launch robot_controller nav2.launch.py

# VM — RViz2
source ~/pi/yaman_capstone/robot_ws/install/setup.bash
ros2 launch robot_controller rviz.launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Rebuild — RPi
cd ~/yaman_capstone/robot_ws && colcon build

# Rebuild — VM
cd ~/pi/yaman_capstone/robot_ws && colcon build

# Kill all ROS processes (use before clean start)
kill -9 $(ps aux | grep -E 'serial_bridge|sllidar|robot_state|ros2' | grep -v grep | awk '{print $2}') 2>/dev/null
```

---

## Priority Fix Order for Next Session

1. **Fix /odom not publishing** — kill all processes, run clean bringup, check `/tmp/bringup.log` for "Odom published x50"
2. **Fix sllidar timeout** — check lidar is spinning, try `scan_mode: 'DenseBoost'`
3. **First full test** — verify `/odom` and `/scan` both publishing, check TF in RViz2
4. **PID tuning** — test motor response with teleop, tune Kp/Ki/Kd
5. **SLAM mapping** — drive around, save map to `~/agv_map`
6. **Nav2 navigation** — set goal in RViz2, verify autonomous navigation

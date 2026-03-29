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
- Motor Left:    PWM→D6,   DIR→D22
- Motor Right:   PWM→D7,   DIR→D23
- IMU:           SCL→RPi Pin 5 (GPIO 3), SDA→RPi Pin 3 (GPIO 2), VCC→3.3V (Pin 1), GND→Pin 6

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
├── yamancode_sketch/
│   └── yamancode_sketch.ino             ← Arduino Mega firmware (AGV + sequence)
├── motor_test_sketch/
│   └── motor_test_sketch.ino            ← Motor direction test
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
            │   ├── serial_bridge.py     ← Arduino↔ROS2: /odom, /sequence/status, /sequence/command
            │   ├── imu_node.py          ← MPU6050 via RPi I2C → /imu/data at 50Hz
            │   └── mission_node.py      ← Shuttle mission: GOING→SEQUENCING→RETURNING
            ├── launch/
            │   ├── bringup.launch.py    ← robot_state_publisher + serial_bridge + imu_node + sllidar
            │   ├── mission.launch.py    ← bringup + mission_node (param: target_distance)
            │   ├── slam.launch.py       ← bringup + slam_toolbox
            │   ├── nav2.launch.py       ← bringup + Nav2 (needs saved map)
            │   └── rviz.launch.py       ← RViz2 (run on VM)
            ├── config/
            │   ├── slam_params.yaml     ← slam_toolbox, saves map to ~/agv_map
            │   ├── nav2_params.yaml     ← Nav2 full config
            │   └── agv.rviz             ← RViz2 config
            ├── setup.py                 ← entry_points: serial_bridge, imu_node, mission_node
            └── package.xml
```

---

## System Overview

AGV medicine dispensing robot. AGV navigates HOME ↔ SHELF (straight line, always same path).
On top of AGV: vertical rail + horizontal rail + SCARA arm + vacuum EE + USB camera.
HOME position = PHARMACIST position (same point).

Full mission:
1. AGV drives forward HOME → SHELF
2. Arduino executes shelf sequence (retrieve box → pick 1 medicine pack → return box)
3. AGV reverses SHELF → HOME
4. Web dashboard shows sequence steps + camera images

---

## Arduino Firmware (`yamancode.cpp`)

- **Serial protocol (115200 baud):**
  - Receive: `V:<vx>,<wz>\n` — velocity m/s and rad/s
  - Receive: `SEQ:START\n` — trigger medicine dispensing sequence
  - Send: `O:<x>,<y>,<th>,<vl>,<vr>\n` — odometry 50Hz
  - Send: `SEQ:STEP:<n>:<desc>\n` — sequence step update
  - Send: `SEQ:RETRY:<n>\n` — vacuum retry attempt n
  - Send: `SEQ:DONE\n` — sequence complete (AGV can return)
  - Send: `SEQ:FAIL:VACUUM_NO_PRESSURE\n` — max retries exceeded
- **Keyboard teleop:** `i`=fwd, `,`=back, `j`=left, `l`=right, `k`=stop, `+/-`=speed
- **PID velocity control** per wheel (Kp=150, Ki=80, Kd=3 — needs tuning)
- **Watchdog:** stops motors if no command for 500ms

Note: IMU was previously wired to Arduino (D20/D21) and firmware sent `I:` lines.
IMU is now on RPi I2C directly — `I:` lines no longer used.

---

## Sequence Protocol (Arduino ↔ RPi)

- Vacuum pump opens BEFORE EE descends, closes AFTER pack placed in container
- Pressure fail → retry: go down again (or shift SCARA slightly), up to 3 retries
- After 3 fails → `SEQ:FAIL` → AGV stops, operator needed
- 1 medicine pack per mission

**Mock mode** (hardware not ready): Mega uses delays (~800ms/step), random pressure fail once per mission then succeeds on retry.

---

## Serial Bridge Node (`serial_bridge.py`)

- Subscribes `/cmd_vel` → sends `V:<vx>,<wz>\n` to Arduino
- Subscribes `/sequence/command` → forwards raw string to Arduino (e.g. `SEQ:START`)
- Reads `O:` → publishes `nav_msgs/Odometry` on `/odom` + `odom→base_footprint` TF
- Reads `SEQ:` lines → publishes `std_msgs/String` on `/sequence/status`
- Watchdog: sends `V:0,0` if no /cmd_vel for 500ms
- Port: `/dev/ttyUSBArduinoMega`

---

## IMU Node (`imu_node.py`)

- Reads MPU6050 directly via RPi I2C bus 1 (address 0x68)
- Publishes `sensor_msgs/Imu` on `/imu/data` at 50Hz, frame `base_link`
- WHO_AM_I = 0x70 (clone quirk) — handled by skipping testConnection, waking via PWR_MGMT_1
- Accel: ±2g range → converts to m/s², Gyro: ±250°/s range → converts to rad/s
- Parameters: `i2c_bus` (default 1), `i2c_address` (default 0x68)
- Dependency: `python3-smbus2` (installed via apt)
- **Verified 2026-03-24:** `/imu/data` publishing, accel Z ≈ 9.59 m/s² (gravity) ✅

---

## Lidar

- Package: **sllidar_ros2** (built from source — `rplidar_ros` from apt fails on C1)
- Port: `/dev/ttyUSBlidar`, baudrate: 460800, scan_mode: `Standard`, frame: `laser_frame`
- Issue resolved (Session 2): kill stale processes before bringup ✅

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

## Bug Fixes Applied

### Session 2 (2026-03-16)

#### Fix 1: sllidar `SL_RESULT_OPERATION_TIMEOUT` — SOLVED
- **Root cause:** Stale `sllidar_node` process from a previous failed launch was holding `/dev/ttyUSBlidar`
- **Fix:** Kill all stale ROS2 processes before bringup
- **Confirmed:** `sllidar_node` runs, gets device info, publishes `/scan` at 10Hz ✅

#### Fix 2: `/odom` not publishing — SOLVED
- **Root cause:** pyserial 3.5 (Ubuntu 24.04 system package) has a thread-safety bug on Python 3.12
  - Concurrent `readline()` in read thread + `write()` in watchdog timer → `TypeError: 'NoneType' object cannot be interpreted as an integer`
  - The crash killed the read thread silently
- **Fix:** Rewrote `serial_bridge.py` to use a command queue:
  - Single serial thread handles ALL serial I/O (reads AND writes)
  - ROS callbacks (`/cmd_vel`, watchdog timer) put commands in `queue.Queue`
  - Serial thread drains queue before each readline — no concurrent access
- **Confirmed:** `/odom` publishes at 49.5Hz ✅

#### Hardware Note: USB cable
- Arduino USB cable was intermittently loose during session — caused `/odom` to drop out
- **Action needed:** Secure the Arduino USB cable (zip tie or strain relief)
- Power supply warning ("not capable of supplying 5A") still shows with mini560 — Pi 5 requires USB PD negotiation; mini560 may not signal PD properly. USB peripheral power may be restricted.

### Session 5 (2026-03-29) — Teleop direction bug FIXED ✅

#### Root causes found and fixed
1. **DIR pins wrong** — code used D52/D53, actual wiring is D22/D23
2. **DIR logic inverted** — `LOW` was coded as forward, but physical forward = `HIGH`
3. **Both encoders inverted** — forward motion gives negative raw counts for both wheels

#### All fixes applied to `yamancode_sketch.ino`
| What | Old | New |
|------|-----|-----|
| MOT_L_DIR pin | D52 | D22 |
| MOT_R_DIR pin | D53 | D23 |
| setMotor() DIR logic | `speed>=0 ? LOW : HIGH` | `speed>=0 ? HIGH : LOW` |
| Left encoder sign | `dl = dl_counts * M_PER_COUNT` | `dl = -dl_counts * M_PER_COUNT` |
| Right encoder sign | `dr = dr_counts * M_PER_COUNT` | `dr = -dr_counts * M_PER_COUNT` |

#### Verified with motor_test_sketch (PWM=50, 2s)
- FORWARD (L=HIGH, R=HIGH): enc_left=-5933, enc_right=-6035, physical **straight forward** ✅
- BACKWARD (L=LOW,  R=LOW):  enc_left=+6061, enc_right=+6008, physical **straight backward** ✅

---

### Session 3 (2026-03-24)

#### IMU moved from Arduino to RPi I2C — DONE
- **Change:** MPU6050 rewired from Arduino D20/D21 to RPi GPIO 2/3 (I2C bus 1)
- **New node:** `imu_node.py` reads I2C directly, publishes `/imu/data` at 50Hz
- **serial_bridge.py:** Removed IMU publisher and `I:` line handler (no longer needed)
- **bringup.launch.py:** Added `imu_node` alongside `serial_bridge`
- **Confirmed:** `/imu/data` publishing, accel Z ≈ 9.59 m/s² ✅

#### Arduino firmware updated — DONE
- **Change:** Removed all MPU6050 code from `yamancode.cpp` (includes, init, `I:` publish)
- **Arduino only outputs:** `O:<x>,<y>,<th>,<vl>,<vr>\n` at 50Hz
- **Uploaded via:** `arduino-cli` 1.4.1 installed on RPi, sketch at `~/yaman_capstone/yamancode_sketch/`
- **Upload command:** `arduino-cli upload --fqbn arduino:avr:mega --port /dev/ttyUSBArduinoMega ~/yaman_capstone/yamancode_sketch/`

#### SLAM lifecycle + params fixed — DONE
- **Problem 1:** slam_toolbox logged "Failed to compute odometry" (TF timeout) and "message is full" (RPi too slow for 10Hz scans)
  - Fix: Added `transform_timeout: 0.5`, `tf_buffer_duration: 30.0`, `throttle_scans: 2` to `slam_params.yaml`
- **Problem 2:** In ROS2 Jazzy, `async_slam_toolbox_node` is a lifecycle node — it starts unconfigured and subscribes to nothing until activated
  - `lifecycle_autostart: true` in slam_params.yaml does NOT work reliably
  - Fix (Session 4): `slam.launch.py` now uses `TimerAction` + `ExecuteProcess` to auto-run `ros2 lifecycle set /slam_toolbox configure` at t=5s and `activate` at t=8s
  - Manual workaround (if needed): `ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate`
- **Confirmed:** `/map` and `/map_updates` publishing ✅, slam_toolbox subscribed to `/scan` ✅

---

## Current Status

### ✅ ALL SENSORS WORKING
- `/scan` publishing at 10Hz — RPLidar C1, Standard mode, 16m range ✅ (verified 2026-03-16)
- `/odom` publishing at ~50Hz — Arduino encoder odometry ✅ (verified 2026-03-16)
- `/imu/data` publishing at ~50Hz — MPU6050 via RPi I2C ✅ (verified 2026-03-24)
- `/tf` tree publishing — odom→base_footprint→base_link→laser_frame ✅
- `robot_state_publisher` running ✅
- `udev symlinks` permanent ✅

### ⚠️ Known Issues
- **USB cable:** Arduino USB cable must be seated firmly — loose connection causes data dropout
- **Power supply:** Pi 5 power warning still present — consider official RPi5 PSU (5.1V/5A with USB PD)
- **PID tuning:** Kp=150, Ki=80, Kd=3 — robot only reaches ~0.08 m/s with 0.20 m/s target, needs tuning
- **RViz map not yet visually confirmed** — /map is publishing but need to confirm map appears in RViz on VM and robot pose is correct

---

## Run Commands

```bash
# RPi — bringup only
ros2 launch robot_controller bringup.launch.py

# RPi — full mission (measure target_distance first via teleop + odom)
ros2 launch robot_controller mission.launch.py target_distance:=3.0

# RPi — trigger mission
ros2 topic pub /mission/command std_msgs/msg/String "data: 'go'" --once

# RPi — monitor mission + sequence
ros2 topic echo /mission/status
ros2 topic echo /sequence/status

# RPi — SLAM mapping
ros2 launch robot_controller slam.launch.py

# VM — RViz2
source ~/pi/yaman_capstone/robot_ws/install/setup.bash
ros2 launch robot_controller rviz.launch.py

# Teleop (run on RPi via ssh)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Upload Arduino firmware
arduino-cli upload --fqbn arduino:avr:mega --port /dev/ttyUSBArduinoMega ~/yaman_capstone/yamancode_sketch/

# Rebuild — RPi
cd ~/yaman_capstone/robot_ws && colcon build

# Rebuild — VM
cd ~/pi/yaman_capstone/robot_ws && colcon build

# Kill all ROS processes (use before clean start)
kill -9 $(ps aux | grep -E 'serial_bridge|imu_node|sllidar|robot_state|ros2' | grep -v grep | awk '{print $2}') 2>/dev/null
```

---

## Git Convention

When the user says **"push"**, it means the full sequence:
```bash
git add .
git commit -m "your comment"
git push
```

---

## Priority for Next Session

1. ~~**FIX TELEOP DIRECTION BUG**~~ — **DONE** (Session 5) ✅
2. ~~**Confirm map visible in RViz**~~ — **DONE** (Session 4) ✅
3. ~~**VERIFY TELEOP**~~ — **DONE** (Session 5) ✅
4. ~~**Mission node + sequence integration**~~ — **DONE** (Session 6, 2026-03-30) ✅
   - mission_node.py: GOING → SEQUENCING → RETURNING
   - serial_bridge.py: SEQ: protocol added
   - yamancode_sketch.ino: mock sequence with vacuum retry
5. **🔴 PID tuning** — updated to Kp=500, Ki=50, Kd=2 (not yet tested)
   - Test with teleop, check if robot reaches 0.20 m/s
   - Stop behavior fixed: target=0 cuts motor immediately
6. **🔴 Measure target_distance** — use teleop + odom to find HOME→SHELF distance
7. **🔴 Test full mock mission** — `ros2 launch robot_controller mission.launch.py target_distance:=X.X`
8. **SLAM mapping** — drive around room, save map to `~/agv_map`
9. **Web dashboard** — sequence steps + camera image (lightweight, runs on RPi)
10. **Camera setup** — USB camera, integrate with sequence Step 9

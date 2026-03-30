# AGV Capstone — Claude Session Context

Read this file at the start of every session to understand the full project state.

---

## Working With Yaman — Guidelines for Claude

**Who is Yaman:**
- Capstone student building a real AGV medicine dispensing robot
- Understands hardware well (wiring, motors, encoders) but less familiar with ROS2/Nav2 internals
- Not always familiar with ROS2 conventions (e.g. didn't know robot front = +x axis)
- Uses screenshots of RViz frequently — always check ~/Pictures/Screenshots/ when he says "look at screenshot"
- Communicates informally and quickly — read carefully, he sometimes skips words
- Gets frustrated (rightfully) when Claude suggests unsafe actions like moving robot forward when it's already close to a wall — always check context before suggesting motion

**How to work with Yaman:**
- **Answer questions FIRST before taking action** — he often asks a question before saying "ok go"
- **Always check physical safety** — robot is real hardware, walls are real. If unsure of space, ask or check odom/AMCL first
- **Keep explanations short** — he's hands-on, doesn't need theory lectures
- **When he says "push"** — run: `cd ~/pi/yaman_capstone && git add . && git commit -m "..." && git push`
- **When he says "look at screenshot"** — `ls -lt ~/Pictures/Screenshots/ | head -3` then read the latest
- **RViz runs on VM** (~/pi/yaman_capstone), **Nav2/ROS2 runs on RPi** (ssh -i ~/.ssh/id_rpi yaman@192.168.137.50)
- **Installed params ≠ source params** — nav2_params.yaml changes must be made to BOTH:
  - Source: `~/pi/yaman_capstone/robot_ws/src/robot_controller/config/nav2_params.yaml`
  - Installed: `~/pi/yaman_capstone/robot_ws/install/robot_controller/share/robot_controller/config/nav2_params.yaml`
  - Nav2 launch uses the INSTALLED file. Source changes don't apply until `colcon build` is run.
- **Kill command before any launch:**
  ```bash
  kill -9 $(ps aux | grep -E 'serial_bridge|imu_node|sllidar|robot_state|ros2|component_container' | grep -v grep | awk '{print $2}') 2>/dev/null
  ```
- **Launch Nav2 on RPi:**
  ```bash
  bash -c 'source /opt/ros/jazzy/setup.bash && source /home/yaman/yaman_capstone/robot_ws/install/setup.bash && nohup ros2 launch robot_controller nav2.launch.py > /tmp/nav2.log 2>&1 &'
  ```
- **Check logs:** `tail -30 /tmp/nav2.log` or `grep -E 'ERROR|WARN|navigating|collision' /tmp/nav2.log | tail -20`

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

### ✅ NAV2 WORKING (Session 8, 2026-03-31)
- Map saved: `~/agv_map.pgm` + `~/agv_map.yaml`
- HOME position on map: **(-2.03, -0.51)** — saved as AMCL initial_pose
- Robot drove **1m forward** successfully ✅
- **No 2D Pose Estimate needed at startup** if robot is at HOME
- **Always bring robot to HOME before shutdown** so AMCL auto-localizes next session

### Nav2 Current Config (key non-default settings)
| Parameter | Value | Location |
|---|---|---|
| `inflation_radius` | 0.30m | local + global costmap |
| `xy_goal_tolerance` | 0.05m | controller_server |
| `use_collision_detection` | false | RPP controller |
| `allow_reversing` | true | RPP controller |
| `use_rotate_to_heading` | false | RPP controller |
| `FootprintApproach.enabled` | false | collision_monitor |
| local costmap plugins | static_layer + inflation_layer | local_costmap |
| robot footprint | `[[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]` | both costmaps |
| AMCL initial_pose | x=-2.03, y=-0.51, yaw=0 | amcl |

### ⚠️ Known Issues
- **USB cable:** Arduino USB cable must be seated firmly — loose connection causes data dropout
- **Power supply:** Pi 5 power warning still present — consider official RPi5 PSU (5.1V/5A with USB PD)
- **PID tuning:** Kp=500, Ki=50, Kd=2 — robot overshoots and creeps slowly after reaching goal. Needs tuning next session.
- **Robot drifts slightly sideways** during forward motion (~6cm over 1m) — likely wheel calibration / wheelbase measurement issue

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

### Session 7 (2026-03-30) — Nav2 AMCL Investigation

#### Root cause found: AMCL orientation ambiguity in symmetric room
- **Symptom:** 2D Goal Pose makes robot go backward (away from goal)
- **Root cause:** AMCL localizes robot 180° wrong in a rectangular room — scan matches the map equally well at 0° and 180°, so AMCL converges to the wrong pose
- **Lidar confirmed correctly mounted** — rpy=0,0,0 in URDF is correct. Verified: moving robot backward → red scan dots cluster at BACK of robot model ✓
- **URDF rpy change (0 0 3.14159) was reverted** — it was wrong, lidar is physically correct
- **AMCL at HOME:** When robot starts at HOME, AMCL localizes at approximately (1.56, 0, yaw≈0) — not (0,0), meaning current map origin ≠ HOME

#### Fix plan for next session
- **Remap the room:** Start SLAM with robot at HOME, odom reset to 0. After mapping, HOME = map origin (0,0,0). AMCL's hardcoded initial_pose (0,0, yaw=0) will then match HOME exactly.
- **Save map:** `ros2 run nav2_map_server map_saver_cli -f ~/agv_map` (overwrite old map)
- **Test:** Launch nav2, robot at HOME, set 1m goal → should go forward correctly

#### Save map command (run while slam.launch.py is active)
```bash
ros2 run nav2_map_server map_saver_cli -f /home/yaman/agv_map
```

---

### Session 8 (2026-03-31) — Nav2 Forward Motion Working ✅

#### Map remapped and Nav2 tuned
- **New map saved** — room remapped with slam.launch.py, saved to ~/agv_map
- **Map origin adjusted** — `agv_map.yaml` origin changed to `[-2.886, -1.467, 0]` so HOME = (-2.03, -0.51) in map frame
- **AMCL initial_pose** set to `(-2.03, -0.51, yaw=0)` in nav2_params.yaml — no 2D Pose Estimate needed on startup IF robot starts at HOME
- **Rule:** Always return robot to HOME before shutdown. If robot is at HOME on startup, AMCL auto-localizes correctly.

#### Nav2 params changes (nav2_params.yaml)
| Parameter | Old | New | Reason |
|---|---|---|---|
| `inflation_radius` | 0.55 | 0.30 | Room too small, walls blocked entire path |
| `use_collision_detection` (RPP) | true | false | Disabled — wires on floor triggered false collision |
| `FootprintApproach.enabled` | true | false | Disabled — was zeroing velocity from floor clutter |
| `use_rotate_to_heading` | true | false | Allow reversing without spin |
| `allow_reversing` | false | true | Robot must reverse back to HOME |
| `xy_goal_tolerance` | 0.25 | 0.05 | Was too large — goal instantly "reached" without moving |
| local costmap `plugins` | voxel_layer + inflation | static_layer + inflation | Removed live obstacle layer (floor wires), added static map |
| footprint | `[[0.25,0.30],...]` | `[[0.30,0.25],...]` | Fixed: front/back=60cm(x), sides=50cm(y) |

#### Verified
- Robot drove **1m forward straight** from HOME ✅ (slight 6cm y-drift, expected — PID/wheel calibration)
- Robot **slows and creeps** after reaching goal — PID needs tuning (Kp=500, Ki=50, Kd=2)
- `allow_reversing: true` is set — backward test NOT yet done (battery low)

---

## Priority for Next Session

1. ~~**FIX TELEOP DIRECTION BUG**~~ — **DONE** (Session 5) ✅
2. ~~**Confirm map visible in RViz**~~ — **DONE** (Session 4) ✅
3. ~~**VERIFY TELEOP**~~ — **DONE** (Session 5) ✅
4. ~~**Mission node + sequence integration**~~ — **DONE** (Session 6, 2026-03-30) ✅
5. ~~**SLAM mapping**~~ — **DONE** (Session 8, 2026-03-31) ✅
6. ~~**REMAP THE ROOM**~~ — **DONE** (Session 8) ✅ — map saved, origin adjusted so HOME = (-2.03, -0.51) in map frame
7. ~~**Test Nav2 forward motion**~~ — **DONE** (Session 8) ✅ — robot drove 1m forward successfully
8. **🔴 PID tuning** — robot creeps slowly after reaching goal (overshoots). Kp=500, Ki=50, Kd=2 in yamancode_sketch.ino
9. **🔴 Test Nav2 RETURN (backward)** — `allow_reversing: true` is set, test sending goal back to HOME after forward move
10. **🔴 Measure target_distance** — use teleop + odom to find HOME→SHELF exact distance
11. **🔴 Test full mock mission** — `ros2 launch robot_controller mission.launch.py target_distance:=X.X`
12. **Web dashboard** — sequence steps + camera image (lightweight, runs on RPi)
13. **Camera setup** — USB camera, integrate with sequence Step 9

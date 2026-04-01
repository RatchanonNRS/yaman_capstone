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
  kill -9 $(ps aux | grep -E 'serial_bridge|imu_node|sllidar|robot_state|ros2|component_container' | grep -v grep | awk '{print $2}') 2>/dev/null; rm -rf /dev/shm/fastrtps_* 2>/dev/null
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
            │   ├── mission_node.py      ← Shuttle mission: GOING→SEQUENCING→RETURNING
            │   └── round_trip_safe.py   ← Odom-based round trip with inline scan safety ✅ WORKING
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
- HOME position on map: **(-2.167, -0.769)** — saved as AMCL initial_pose (updated Session 14, 2026-04-01)
- SHELF position on map: **(0.366, -0.464)** (measured Session 14, 2026-04-01)
- Robot drove **1m forward** successfully ✅
- **No 2D Pose Estimate needed at startup** if robot is at HOME
- **Always bring robot to HOME before shutdown** so AMCL auto-localizes next session

### Session 16 (2026-04-01) — Brake reduced, sequence heartbeat added

#### What was done
- **sequence_hardware.h updated** — `seqKeepAlive()` now fires `SEQ:HB` every 3s during blocking sequence (previously heartbeat was dead for ~30-60s while sequence ran)
- **EE dwell added** — `seqDelay(1000)` after EE down pick, before retract — matches `arduinonoraspi.txt` (1s vacuum dwell to seat pack)
- **Brake reduced** — `BRAKE_PWM` 80→20, `BRAKE_DURATION_MS` 500→200ms to prevent Pi brownout during teleop
- **Brake still problematic for teleop** — even at 20 PWM the brake behavior is noticeable. Root cause: `teleop_twist_keyboard` sends V:0,0 on key release, which triggers the brake transition. Fix needed: investigate disabling brake for teleop or reducing further.
- **Robot NOT at HOME** — Pi shut down mid-session during teleop. Robot manually repositioned by Yaman next session.

---

### ✅ MISSION NODE WORKING (Session 10, 2026-03-31)
- **target_distance = 2.70m** (updated Session 15, 2026-04-01 — 2.50m was 20cm short of shelf)
- **Return leg:** uses direct cmd_vel (NOT Nav2) — NavFn planner always turns, doesn't reverse. Nav2 still used for GOING leg (legitimate for report).
- **Slowdown added to mission_node.py:** slows to 0.05 m/s at 0.20m before target (both GOING and RETURNING)
- **Mission ran successfully this session:** reached shelf (x=2.73m odom), started SEQ:START ✅
- **y-drift during straight mission: only -0.0036m** — almost perfectly straight ✅

### Session 15 (2026-04-01) — SEQ:START FIXED ✅, target_distance updated to 2.70m

#### What was done
- **SEQ:START root cause found and fixed** — wrong sketch (motor_test_sketch) was loaded on Arduino. yamancode_sketch was never actually running in previous sessions. Uploaded correct sketch → SEQ:START works immediately.
- **Full sequence tested end-to-end:** SEQ:STEP:1 → STEP:3 → ... → SEQ:RETRY:1 → SEQ:DONE ✅
- **SEQ:HB heartbeat added** to yamancode loop() — sends `SEQ:HB` every 3s. Confirms bidirectional serial is live. Appears in nav2.log as `Sequence: SEQ:HB`. Keep for ongoing health monitoring.
- **SEQ:DBUG line removed** from parseCommand (was spamming V: debug at 10Hz).
- **target_distance updated to 2.70m** — 2.50m was 20cm short of shelf. Confirmed with round trip: SHELF at x=2.723m, HOME return at x=0.026m ✅
- **mock_serial_test sketch created** at `~/yaman_capstone/mock_serial_test/` — proved bidirectional serial concept works (O: stream + SEQ:START/DONE). Keep for future testing.
- **Robot is at HOME** ✅

---

### Session 13 (2026-04-01) — round_trip_safe.py WORKING ✅, Lidar ghost root cause found

#### What was done
- **round_trip_safe.py fully working** — robot drove 2.506m HOME→SHELF, waited 2s, returned 2.623m home ✅
- **Lidar ghost root cause identified** — the "ghost" in front scan is the robot's own ARM/HORIZONTAL RAIL at 0.71m@scan 0°
- **Robot corner pillars identified** — appear at ±39° and ±141° at ~0.33m (4 pillars of AGV frame)
- **Lidar correctly oriented confirmed** — scan 0° = physical front of robot ✅

#### Lidar scan anatomy (full 360°, robot at HOME)
| Direction | Range | What it is |
|---|---|---|
| Scan 0° (front) | 0.71m | **Robot's own ARM/rail structure** — always present, filters needed |
| Scan ±39°, ±141° | ~0.33m | **Robot's own corner pillars** (matches footprint geometry) |
| Scan -90° (right) | 0.85m | Right room wall |
| Scan +90° (left) | 1.33m | Left room wall |
| Scan 180° (rear) | 2.88m | Back room wall |

#### round_trip_safe.py — how it works
- File: `robot_controller/robot_controller/round_trip_safe.py` (also at `/tmp/round_trip_safe.py` on RPi)
- **Annular cone:** ±12° INNER exclusion (robot arm zone) to ±20° OUTER — skips arm, checks sides
- **MIN_DETECT_DIST = 0.50m** (filters pillars at 0.33m)
- **MIN_POINTS = 100** (effectively disabled for now — arm gives ~32pts in annular zone; set to 50 to enable real detection)
- **Dynamic range:** `min(1.5m, remaining - 0.3m)` — shrinks as robot approaches shelf (avoids shelf wall false trigger)
- **Speeds:** NORMAL=0.15 m/s, SLOW=0.05 m/s, slowdown at last 0.20m
- **Logs:** `/tmp/rts.log` on RPi

#### To enable real obstacle detection next session:
Change `MIN_POINTS = 100` → `MIN_POINTS = 50` in round_trip_safe.py. Arm gives ~32 pts, a real person gives >32 pts.

#### Robot position at end of session
- **Robot is at HOME** ✅ — odom was 0,0 at start, robot returned successfully

---

### Session 14 (2026-04-01) — HOME/SHELF positions remeasured, SEQ:START deep debug

#### What was done
- **HOME remeasured** — 2D Pose Estimate at HOME → AMCL (-2.167, -0.769). Updated both source + installed nav2_params.yaml
- **SHELF measured** — AMCL (0.366, -0.464) on map. Saved to memory.
- **Robot driven HOME→SHELF (2.504m) and SHELF→HOME (2.502m)** using inline go_shelf/go_back scripts ✅
- **SEQ:START deep debug started** — see Known Issues for full detail

#### SEQ:START debug findings (Session 14)
- Standalone Python test (new serial connection): `parseCommand` NEVER called for ANY command (V: or SEQ:START). Arduino resets on port open, but even after 3s boot, no bytes received by Arduino.
- Via serial_bridge: serial_bridge logs "Sequence command → Arduino: SEQ:START" ✓ but Arduino never replies
- Added `Serial.println("SEQ:DBUG:" + cmd)` at top of parseCommand — never appears in serial_bridge log
- serial_bridge only holds the port (confirmed via lsof) — no conflict
- Em-dash `—` removed from SEQ:STEP:1 string (was potentially corrupting ASCII decode)
- **Root cause still unknown** — next step: add SEQ:HB heartbeat to Arduino to confirm serial_bridge reads SEQ: lines from Arduino at all

#### Robot position at end of session
- Robot returned to HOME in Session 15 ✅

---

### Session 12 (2026-03-31) — Safety node built, teleop fixed, many issues found

#### What was done
- **safety_node.py built** — watches /scan during GOING/RETURNING, publishes /emergency_stop (Bool)
- **serial_bridge.py updated** — subscribes /emergency_stop, drops cmd_vel and sends V:0,0 when True
- **Kill command updated** — now includes `rm -rf /dev/shm/fastrtps_*` to prevent shared memory port clog
- **FootprintApproach disabled** — was fighting teleop on /cmd_vel (4 publishers, collision_monitor output = /cmd_vel)
- **round_trip_safe.py** at `/tmp/round_trip_safe.py` on RPi — round trip with safety built in, dynamic max_range

#### Problems found this session

**1. Teleop unreliable after restarts**
- Root cause 1: collision_monitor outputs to /cmd_vel. When it sees obstacles (room walls), publishes V:0,0 constantly, overriding teleop. Fixed: FootprintApproach.enabled = False
- Root cause 2: multiple Nav2 restarts clog shared memory ports (fastrtps_port7000+). Fixed: add `rm -rf /dev/shm/fastrtps_*` to kill command

**2. Odom resets on every Nav2/bringup restart**
- Odom resets to 0,0 regardless of physical robot position
- go_shelf.py / return_home.py measure distance FROM odom 0,0 — if robot is not physically at HOME when Nav2 starts, scripts travel wrong distance and hit walls
- **Rule: ALWAYS teleop robot to HOME physically before restarting Nav2**

**3. Safety node (separate) architecture was problematic**
- Too many nodes competing on /cmd_vel (collision_monitor, safety_node, teleop, scripts)
- Better design: integrate scan check INSIDE the movement script (round_trip_safe.py does this)
- Safety node files still exist in codebase but round_trip_safe.py is the tested approach

**4. Safety triggers on shelf wall**
- At ~1m from home, shelf wall (1.5m ahead) enters detection zone → false stop
- Fix: dynamic max_range = min(1.5m, remaining_distance - 0.3m). Implemented in round_trip_safe.py but NOT yet fully tested

**5. SmacPlanner2D failed on RPi**
- Tried to fix Nav2 "turns instead of reversing" — SmacPlanner2D too heavy, robot never moved
- Reverted to NavFn. Nav2 2D Goal Pose still turns — accepted limitation for RViz manual testing
- Actual mission legs use direct cmd_vel (not Nav2) — works correctly

**6. SEQ:START bug still not fixed**
- No progress this session

#### ⚠️ CRITICAL RULE ADDED THIS SESSION
**Before ANY movement script: robot MUST be physically at HOME with odom at 0,0**
Check odom with: `tail -3 /tmp/nav2.log | grep Odom` — must show x≈0, y≈0 before running go_shelf or mission

#### Robot position at end of session
- Robot is being returned to HOME by Yaman via teleop
- Nav2 running, odom at 0,0

### Session 11 (2026-03-31) — RViz fixed, GOING leg confirmed, SEQ:START bug found

#### What worked
- **return_home script ran successfully:** traveled 2.504m, robot physically at HOME ✅
- **RViz fully working:** Map(/map), LaserScan(/scan), RobotModel, TF, AMCL particles all visible ✅
  - agv.rviz already had all topics set correctly — no file changes needed
  - Used **2D Pose Estimate** to align red scan dots with black map walls ✅
  - AMCL converged to ~(-2.0, -0.6) ≈ HOME ✅
- **GOING leg confirmed again:** robot drove 2.50m from HOME to SHELF ✅

#### SEQ:START bug — NOT FIXED, investigate next session
- mission_node reached shelf, sent SEQ:START via /sequence/command ✅
- serial_bridge received and forwarded SEQ:START to Arduino ✅ (logged: "Sequence command → Arduino: SEQ:START")
- **Arduino sent NO response** — no SEQ:STEP, SEQ:DONE, or SEQ:FAIL messages on serial
- Arduino sketch DOES have SEQ:START handler (yamancode_sketch.ino line 256) — should reply with SEQ:STEP:1
- Root cause unknown — Arduino may have missed the command due to serial buffer/timing issue, or mock sequence state machine got stuck
- **Next session:** debug by manually sending `SEQ:START\n` to Arduino via serial monitor or `echo "SEQ:START" > /dev/ttyUSBArduinoMega` and watch for response

#### **Robot currently at SHELF** — must return to HOME at start of next session before anything else

### Nav2 Current Config (key non-default settings)
| Parameter | Value | Location |
|---|---|---|
| `inflation_radius` | 0.30m | local + global costmap |
| `xy_goal_tolerance` | **0.03m** | controller_server |
| `use_collision_detection` | false | RPP controller |
| `allow_reversing` | true | RPP controller |
| `use_rotate_to_heading` | false | RPP controller |
| `FootprintApproach.enabled` | false | collision_monitor |
| local costmap plugins | static_layer + inflation_layer | local_costmap |
| robot footprint | `[[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]` | both costmaps |
| AMCL initial_pose | x=-2.167, y=-0.769, yaw=0 | amcl |

### Arduino Firmware Current State (yamancode_sketch.ino)
| Parameter | Value |
|---|---|
| `PID_KP` | 300 |
| `PID_KI` | 20 |
| `PID_KD` | 8 |
| `PID_INTEGRAL_MAX` | 40 |
| `VEL_DEADBAND` | 0.03 m/s |
| `CMD_TIMEOUT_MS` (watchdog) | 200ms |
| `BRAKE_PWM` | 80 |
| `BRAKE_DURATION_MS` | 500ms |
| Brake type | Timer-based — fires on transition from moving→stop, uses old target direction |
| `BRAKE_PWM` | 20 (reduced from 80 — was causing Pi brownout on teleop) |
| `BRAKE_DURATION_MS` | 200ms (reduced from 500) |
| `SEQ:HB` heartbeat | Every 3s — fires from main loop (idle/driving) AND from seqKeepAlive() during sequence |

**Brake behavior:** When target goes to 0 (k pressed or watchdog), brake fires at 80 PWM for 500ms. Floor friction = 0.011 m/s². Brings robot from ~0.10 m/s to ~0.022 m/s, then 2-3cm coast. **Not yet confirmed working** — to be tested next session.

### ⚠️ Known Issues
- **USB cable:** Arduino USB cable must be seated firmly — loose connection causes data dropout
- **Power supply:** Pi 5 power warning still present — consider official RPi5 PSU (5.1V/5A with USB PD)
- **Robot's arm in scan plane:** Horizontal rail/arm appears at 0.71m@0° in lidar scan — use annular cone filter (±12° exclusion) in any scan-based safety code. Proper fix: `laser_filters` angular bounds filter.
- ~~**SEQ:START no response**~~ — **FIXED Session 15** ✅ Root cause: motor_test_sketch was loaded on Arduino instead of yamancode_sketch. Correct sketch now uploaded. SEQ:DBUG debug line removed. SEQ:HB heartbeat added permanently.
- **Robot drifts slightly sideways** during forward motion (~6cm over 2.5m) — acceptable for now
- **round_trip_safe.py scan disabled:** MIN_POINTS=100 means no real obstacle detection yet — set to 50 to enable
- **Brake still bad for teleop** — even at BRAKE_PWM=20/200ms, brake fires on key release via V:0,0. Options: (a) lower further to 5/100ms, (b) disable brake entirely for teleop by adding a BRAKE:OFF serial command, (c) remove brake and rely on floor friction + slowdown zone for mission precision

---

## Run Commands

```bash
# RPi — round trip safe (odom-based, with inline scan safety)
# Robot MUST be at HOME with odom=0,0 before running
source /opt/ros/jazzy/setup.bash && source ~/yaman_capstone/robot_ws/install/setup.bash
python3 ~/yaman_capstone/robot_ws/src/robot_controller/robot_controller/round_trip_safe.py
# Logs: tail -f /tmp/rts.log

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

### Session 9 (2026-03-31) — PID Brownout Bug Fixed ✅

#### Root cause found: active brake + high Kp caused Pi shutdown

**What happened:** During PID tuning (previous unsaved session), firmware was uploaded with `Kp=500` and an active braking block. The robot overshot the goal at high speed (~0.3 m/s). When the watchdog fired (target→0), the active brake applied an immediate full reverse pulse (`-vel × 200`, up to PWM=-100). This sudden motor direction reversal caused a large current spike → Pi 5 voltage dropped below threshold → shutdown. On Pi restart, the same brake fired again from robot coasting → second shutdown. Uploading empty sketch to Mega stopped the motors and allowed Pi to stay on.

#### Fixes applied to `yamancode_sketch.ino`

| What | Before | After |
|------|--------|-------|
| `PID_KP` | 500 | 200 |
| Active brake cap | 100 (caused brownout) | 70 |
| Active brake multiplier | 200 | 150 |
| Comment: DIR pins | D52/D53 (wrong) | D22/D23 (correct) |

**Brake tuning history this session:**
- First fix: removed brake entirely (pwm=0) → robot coasted 4-5 seconds, too slow
- Second try: cap=40, multiplier=80 → still too slow to stop
- Third try: cap=70, multiplier=150 → still too slow
- cap=100, multiplier=200 → still slow — robot arrives at low speed (low Kp), so proportional brake force is also tiny (e.g. 0.05 m/s × 200 = 10 PWM, not enough)
- Watchdog timeout: 500ms → 200ms
- **Root cause of slow stop:** proportional brake is weak when robot is slow. Fix: use FIXED-force brake (constant PWM regardless of speed) + raise Kp so robot actually reaches commanded speed
- fixed 80 PWM based on encoder vel direction → still same (encoder vel near 0 at low speed, brake never triggers)
- **Root cause confirmed:** brake condition `vel > VEL_DEADBAND` never fires because at low speed encoder reads ~0
- Timer-based brake (300ms, 150 PWM) → still no effect. Bug: teleop sends V:0,0 at 10Hz, each call re-invokes startBrake() which re-reads target (already 0) and resets brake_dir to 0, cancelling the brake every 100ms
- **Real fix:** only call startBrake() on TRANSITION from moving to stop (check old target was non-zero). This fixed the cancellation bug.
- Timer brake confirmed WORKING from log. Tuning history:
  - 150 PWM, 300ms → overshot -0.27m backward (too strong)
  - 80 PWM, 150ms → decelerates -0.103→-0.033 m/s in 1s, then coasts ~5cm (floor friction 0.011 m/s²)
  - 80 PWM, 250ms → same coast issue, 0.033 m/s remaining = 5cm
  - 80 PWM, 500ms → brings vel to ~0.022 m/s → 2-3cm coast ✓. Also xy_goal_tolerance 0.05→0.03

**Why Kp=300:** Stronger than original 150, robot responds better. Active brake replaced with timer-based brake — safe from brownout. Kp=500 was the original brownout cause.

**Floor friction note:** Floor deceleration measured at 0.011 m/s². Very low. With 80 PWM brake:
- Brake deceleration = 0.248 m/s² (measured from log)
- 500ms brings robot from 0.10 m/s to ~0.022 m/s → coast = 2-3cm ✓
- At full teleop speed (0.22 m/s) stopping distance is ~10cm — physics limit without stronger brake

**Also changed:** `xy_goal_tolerance` 0.05 → 0.03m (both source and installed nav2_params.yaml)

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
8. ~~**PID brownout crash**~~ — **FIXED** (Session 9) ✅ — Kp 500→200, removed active brake
9. ~~**Measure target_distance**~~ — **DONE** (Session 10) ✅ — target_distance = **2.50m**
10. ~~**Nav2 return decision**~~ — **DECIDED** (Session 10) ✅ — use cmd_vel odom-based return in mission_node. Nav2 for GOING only.
11. ~~**Slowdown zone**~~ — **DONE** (Session 10) ✅ — slows to 0.05 m/s at 0.20m before target (both directions)
12. ~~**Return robot to HOME**~~ — **DONE** (Session 11) ✅ — return_home script ran, traveled 2.504m
13. ~~**Fix RViz for presentation**~~ — **DONE** (Session 11) ✅ — agv.rviz already correct, used 2D Pose Estimate to align scan with map
14. ~~**Return robot to HOME**~~ — done (Session 13) ✅
15. ~~**Test round_trip_safe.py**~~ — **DONE** (Session 13) ✅ — drove 2.506m to shelf, returned 2.623m home
16. ~~**Return robot to HOME**~~ — done ✅
17. ~~**Debug SEQ:START**~~ — **FIXED Session 15** ✅ (wrong sketch was loaded)
18. **🔴 Fix teleop brake** — brake fires on V:0,0 from key release. Options: lower to 5/100ms, add BRAKE:OFF command, or remove entirely (floor friction sufficient with slowdown zone).
19. **🔴 Enable real obstacle detection** — set `MIN_POINTS = 50` in round_trip_safe.py (was 100=disabled). Block path mid-run, confirm robot stops and resumes.
19. **🔴 Integrate safety + sequence into mission_node** — merge round_trip_safe logic + SEQ:START/DONE handling into mission_node.py for full autonomous mission
20. **🔴 Save round_trip_safe.py to repo** — currently only at /tmp on RPi, lost on reboot
21. **Web dashboard** — sequence steps + camera image (lightweight, runs on RPi)
22. **Camera setup** — USB camera, integrate with sequence Step 9

### Return Robot to HOME (inline script — use when robot is stuck at SHELF)
```bash
ssh -i ~/.ssh/id_rpi yaman@192.168.137.50
source /opt/ros/jazzy/setup.bash && source ~/yaman_capstone/robot_ws/install/setup.bash
# Then in Python:
python3 - <<'EOF'
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ReturnHome(Node):
    def __init__(self):
        super().__init__('return_home')
        self.target = 2.50
        self.start_x = self.start_y = self.cur_x = self.cur_y = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.1, self.loop)
        self.done = False
    def odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.start_x is None: self.start_x, self.start_y = x, y
        self.cur_x, self.cur_y = x, y
    def loop(self):
        if self.done or self.start_x is None: return
        traveled = math.sqrt((self.cur_x-self.start_x)**2+(self.cur_y-self.start_y)**2)
        remaining = self.target - traveled
        if traveled >= self.target:
            self.pub.publish(Twist())
            self.get_logger().info('HOME reached')
            self.done = True; raise SystemExit
        t = Twist()
        t.linear.x = -0.05 if remaining <= 0.20 else -0.15
        self.pub.publish(t)
rclpy.init(); node = ReturnHome()
try: rclpy.spin(node)
except SystemExit: pass
node.destroy_node(); rclpy.shutdown()
EOF
```
12. **Web dashboard** — sequence steps + camera image (lightweight, runs on RPi)
13. **Camera setup** — USB camera, integrate with sequence Step 9

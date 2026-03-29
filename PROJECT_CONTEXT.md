AGV Medicine Dispensing Robot — Project Context
================================================

SYSTEM OVERVIEW
---------------
AGV navigates HOME ↔ SHELF (always straight line, same path).
HOME position = PHARMACIST position (same physical point).
1 medicine pack picked per mission.

On top of AGV:
- Vertical rail (lifts to shelf height)
- Horizontal rail (extends to reach shelf)
- Gripper (grabs medicine box from shelf)
- SCARA arm with vacuum EE + servo rack
- Camera on top of SCARA (captures image before EE descends)
- Container (box on AGV that holds collected medicine pack)

HARDWARE
--------
- Raspberry Pi 5, Ubuntu 24.04 LTS Server
- Arduino Mega (CH340, /dev/ttyUSBArduinoMega) — controls ALL low-level hardware
- RPLidar C1 (/dev/ttyUSBlidar)
- Cytron MDD20A motor driver
- MPU6050 IMU on RPi I2C
- E6B2-CWZ6C 600P/R encoders
- USB camera (pending setup after SCARA hardware complete)

CONNECTIONS
-----------
Encoder Left:  OutA→D2,  OutB→D3
Encoder Right: OutA→D18, OutB→D19
Motor Left:    PWM→D6,   DIR→D22
Motor Right:   PWM→D7,   DIR→D23
IMU:           SCL→RPi Pin 5, SDA→RPi Pin 3, VCC→3.3V

Network: RPi IP 192.168.137.50, user: yaman
SSH: ssh -i ~/.ssh/id_rpi yaman@192.168.137.50  (alias: rpi)
Mount: sshfs yaman@192.168.137.50:/home/yaman ~/pi  (alias: pimount)

OPERATION SEQUENCE (see robot_sequence.txt for full detail)
-----------------------------------------------------------
1. AGV drives HOME → SHELF
2. Retrieve medicine box (vertical up, horizontal extend, grip, retract, down)
3. Pick medicine pack (SCARA to box, camera image, vacuum open, EE down,
   pressure check [retry up to 3x if fail], retract, SCARA to container, vacuum off)
4. Return medicine box to shelf (vertical up, extend, place, release, retract, down)
5. AGV reverses SHELF → HOME
6. Mission complete

VACUUM RETRY LOGIC
------------------
- Vacuum pump opens BEFORE EE descends
- Pressure sensor checked after EE contact
- Fail → rack up, shift SCARA slightly, retry (max 3 attempts)
- After 3 fails → SEQ:FAIL → AGV stops, operator needed
- Vacuum pump closes AFTER pack placed in container

SERIAL PROTOCOL (115200 baud, same USB port)
--------------------------------------------
RPi → Arduino:  V:<vx>,<wz>\n        (velocity command)
                SEQ:START\n           (trigger sequence)
Arduino → RPi:  O:<x>,<y>,<th>,<vl>,<vr>\n  (odometry 50Hz)
                SEQ:STEP:<n>:<desc>\n  (step update)
                SEQ:RETRY:<n>\n        (vacuum retry)
                SEQ:DONE\n             (sequence complete)
                SEQ:FAIL:<reason>\n    (max retries exceeded)

ROS2 TOPICS
-----------
/cmd_vel           — AGV velocity commands
/odom              — wheel odometry
/imu/data          — IMU data
/scan              — lidar scan
/sequence/command  — RPi → serial_bridge → Arduino (SEQ:START)
/sequence/status   — Arduino → serial_bridge → ROS2 (SEQ:STEP/DONE/FAIL)
/mission/command   — 'go' | 'abort' to mission_node
/mission/status    — mission state updates

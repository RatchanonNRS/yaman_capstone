#!/usr/bin/env python3
"""
mission_node.py — Straight-line shuttle with obstacle avoidance

Full mission sequence:
  1. GOING      : drive forward target_distance metres to SHELF
  2. SEQUENCING : send SEQ:START to Arduino → wait for SEQ:DONE
                  Arduino executes retrieve box → pick medicine → return box
                  Vacuum retry: up to 3 attempts on pressure failure
  3. RETURNING  : drive backward target_distance metres to HOME
  4. IDLE       : mission complete

Obstacle avoidance:
  - Monitors a forward / rear cone from /scan
  - Pauses if obstacle is closer than `obstacle_stop_distance`
  - IGNORES the wall when within `obstacle_stop_distance` of destination

Topics:
  /mission/command  (String in)  : 'go' | 'abort'
  /mission/status   (String out) : state updates + sequence step text
  /sequence/command (String out) : 'SEQ:START' → serial_bridge → Arduino
  /sequence/status  (String in)  : SEQ:STEP/DONE/FAIL ← serial_bridge ← Arduino

Trigger:
  ros2 topic pub /mission/command std_msgs/msg/String "data: 'go'" --once
  ros2 topic pub /mission/command std_msgs/msg/String "data: 'abort'" --once

Parameters (set at launch or via ros2 param set):
  target_distance        (float, default 2.0)   metres to travel
  forward_speed          (float, default 0.15)  m/s
  obstacle_stop_distance (float, default 0.45)  metres from lidar centre
  check_angle_deg        (float, default 30.0)  half-width of scan cone
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('target_distance',        2.0)
        self.declare_parameter('forward_speed',          0.15)
        self.declare_parameter('slow_speed',             0.05)
        self.declare_parameter('slowdown_distance',      0.20)
        self.declare_parameter('obstacle_stop_distance', 0.45)
        self.declare_parameter('check_angle_deg',        30.0)

        self.target_dist    = self.get_parameter('target_distance').value
        self.speed          = self.get_parameter('forward_speed').value
        self.slow_speed     = self.get_parameter('slow_speed').value
        self.slowdown_dist  = self.get_parameter('slowdown_distance').value
        self.stop_dist      = self.get_parameter('obstacle_stop_distance').value
        self.check_half_rad = math.radians(self.get_parameter('check_angle_deg').value)

        # Odometry
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0

        # Scan
        self.front_clear = True   # True = no obstacle within stop_dist ahead
        self.rear_clear  = True

        # State machine
        self.state = 'IDLE'   # IDLE | GOING | SEQUENCING | RETURNING | FAILED

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',          10)
        self.status_pub = self.create_publisher(String, '/mission/status',   10)
        self.seq_pub    = self.create_publisher(String, '/sequence/command', 10)

        self.create_subscription(Odometry,  '/odom',            self._odom_cb,    10)
        self.create_subscription(LaserScan, '/scan',            self._scan_cb,    10)
        self.create_subscription(String,    '/mission/command', self._command_cb, 10)
        self.create_subscription(String,    '/sequence/status', self._seq_cb,     10)

        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Mission node ready — target={self.target_dist}m  speed={self.speed}m/s  '
            f'stop_dist={self.stop_dist}m\n'
            f'  ros2 topic pub /mission/command std_msgs/msg/String "data: \'go\'" --once'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def _scan_cb(self, msg):
        self.front_clear = self._sector_clear(msg, center=0.0)
        self.rear_clear  = self._sector_clear(msg, center=math.pi)

    def _command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'go' and self.state == 'IDLE':
            self._reset_start()
            self.state = 'GOING'
            self.get_logger().info(f'GO → driving {self.target_dist} m forward')
        elif cmd == 'abort':
            self._stop()
            self.state = 'IDLE'
            self.get_logger().info('Mission aborted')

    def _seq_cb(self, msg):
        status = msg.data.strip()
        if self.state != 'SEQUENCING':
            return
        self._publish_status(status)
        if status == 'SEQ:DONE':
            self.get_logger().info('Sequence done → returning home')
            self._reset_start()
            self.state = 'RETURNING'
        elif status.startswith('SEQ:FAIL'):
            self.get_logger().error(f'Sequence FAILED: {status} — operator needed')
            self._stop()
            self.state = 'FAILED'

    # ── Control loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        if self.state in ('IDLE', 'SEQUENCING', 'FAILED'):
            return

        traveled  = self._traveled()
        remaining = self.target_dist - traveled

        # When remaining distance ≤ stop_dist the destination wall IS within
        # sensor range — treat it as a normal arrival, not an obstacle
        near_wall = remaining <= self.stop_dist

        twist = Twist()

        if self.state == 'GOING':
            if traveled >= self.target_dist:
                self._stop()
                self.state = 'SEQUENCING'
                seq_msg = String()
                seq_msg.data = 'SEQ:START'
                self.seq_pub.publish(seq_msg)
                self.get_logger().info('Shelf reached → starting sequence')
                return

            if not self.front_clear and not near_wall:
                self._stop()
                self._publish_status('OBSTACLE_AHEAD — waiting')
                return

            twist.linear.x = self.slow_speed if remaining <= self.slowdown_dist else self.speed

        elif self.state == 'RETURNING':
            if traveled >= self.target_dist:
                self._stop()
                self.state = 'IDLE'
                self.get_logger().info('Home reached — mission complete')
                self._publish_status('DONE')
                return

            if not self.rear_clear and not near_wall:
                self._stop()
                self._publish_status('OBSTACLE_BEHIND — waiting')
                return

            twist.linear.x = -self.slow_speed if remaining <= self.slowdown_dist else -self.speed

        self.cmd_pub.publish(twist)
        self._publish_status(f'{self.state}  {traveled:.2f}/{self.target_dist:.2f} m')

    # ── Helpers ──────────────────────────────────────────────────────────────────

    def _sector_clear(self, msg: LaserScan, center: float) -> bool:
        """Return True if no scan point within stop_dist in the sector."""
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            diff  = abs(math.atan2(math.sin(angle - center), math.cos(angle - center)))
            if diff <= self.check_half_rad and r < self.stop_dist:
                return False
        return True

    def _traveled(self) -> float:
        dx = self.cur_x - self.start_x
        dy = self.cur_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def _reset_start(self):
        self.start_x = self.cur_x
        self.start_y = self.cur_y

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

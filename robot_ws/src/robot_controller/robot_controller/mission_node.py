#!/usr/bin/env python3
"""
mission_node.py — Straight-line shuttle, no obstacle avoidance

  1. GOING      : drive forward target_distance metres to SHELF
  2. SEQUENCING : send SEQ:START → wait for SEQ:DONE
  3. RETURNING  : drive backward target_distance metres to HOME
  4. IDLE       : mission complete

No scan obstacle avoidance — robot drives straight regardless.
All sensors (/scan, /odom, /tf, AMCL) still publish for RViz.

Topics:
  /mission/command  (String in)  : 'go' | 'abort'
  /mission/status   (String out) : state updates
  /sequence/command (String out) : 'SEQ:START'
  /sequence/status  (String in)  : SEQ:STEP/DONE/FAIL

Trigger:
  ros2 topic pub /mission/command std_msgs/msg/String "data: 'go'" --once
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('target_distance',   2.70)
        self.declare_parameter('forward_speed',     0.15)
        self.declare_parameter('slow_speed',        0.05)
        self.declare_parameter('slowdown_distance', 0.20)

        self.target_dist   = self.get_parameter('target_distance').value
        self.speed         = self.get_parameter('forward_speed').value
        self.slow_speed    = self.get_parameter('slow_speed').value
        self.slowdown_dist = self.get_parameter('slowdown_distance').value

        self.cur_x = self.cur_y = 0.0
        self.start_x = self.start_y = 0.0
        self.state = 'IDLE'

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',          10)
        self.status_pub = self.create_publisher(String, '/mission/status',   10)
        self.seq_pub    = self.create_publisher(String, '/sequence/command', 10)

        self.create_subscription(Odometry, '/odom',            self._odom_cb,    10)
        self.create_subscription(String,   '/mission/command', self._command_cb, 10)
        self.create_subscription(String,   '/sequence/status', self._seq_cb,     10)

        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Mission node ready — target={self.target_dist}m  '
            f'speed={self.speed}m/s  slowdown at {self.slowdown_dist}m\n'
            f'  ros2 topic pub /mission/command std_msgs/msg/String "data: \'go\'" --once'
        )

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def _command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'go' and self.state == 'IDLE':
            self._reset_start()
            self.state = 'GOING'
            self.get_logger().info(f'GO → driving {self.target_dist}m forward')
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
            self.get_logger().error(f'Sequence FAILED: {status}')
            self._stop()
            self.state = 'FAILED'

    def _loop(self):
        if self.state in ('IDLE', 'SEQUENCING', 'FAILED'):
            return

        traveled  = self._traveled()
        remaining = self.target_dist - traveled

        if self.state == 'GOING':
            if traveled >= self.target_dist:
                self._stop()
                self.state = 'SEQUENCING'
                seq_msg = String()
                seq_msg.data = 'SEQ:START'
                self.seq_pub.publish(seq_msg)
                self.get_logger().info('Shelf reached → starting sequence')
                return
            t = Twist()
            t.linear.x = self.slow_speed if remaining <= self.slowdown_dist else self.speed
            self.cmd_pub.publish(t)

        elif self.state == 'RETURNING':
            if traveled >= self.target_dist:
                self._stop()
                self.state = 'IDLE'
                self.get_logger().info('Home reached — mission complete')
                self._publish_status('DONE')
                return
            t = Twist()
            t.linear.x = -self.slow_speed if remaining <= self.slowdown_dist else -self.speed
            self.cmd_pub.publish(t)

        self._publish_status(f'{self.state}  {traveled:.2f}/{self.target_dist:.2f}m')

    def _traveled(self):
        dx = self.cur_x - self.start_x
        dy = self.cur_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def _reset_start(self):
        self.start_x = self.cur_x
        self.start_y = self.cur_y

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text):
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

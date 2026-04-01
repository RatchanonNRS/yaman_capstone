"""
safety_node.py — Lidar-based people detection safety watchdog

Watches /scan during GOING and RETURNING mission legs.
Publishes /emergency_stop (Bool=True) when a person is detected in the
robot's path, which serial_bridge uses to immediately stop the motors.

Detection zone:
  - Front arc  (±30°) when GOING
  - Rear arc   (±30°) when RETURNING
  - Range: 0.35m (skip robot wires) → 1.5m (detect people)
  - Disabled within 0.5m of target (last 0.5m of each leg) to avoid
    triggering on the shelf wall or home position itself
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class SafetyNode(Node):

    MIN_RANGE     = 0.35   # m — ignore robot's own wires
    MAX_RANGE     = 1.5    # m — detect people at this distance
    ARC_DEG       = 30.0   # half-angle of detection cone (degrees)
    TARGET_DIST   = 2.50   # m — total travel distance per leg
    NEAR_GOAL     = 0.50   # m — disable check this far before target
    RAMP_UP       = 0.30   # m — disable check for first 0.3m (near start position)

    def __init__(self):
        super().__init__('safety_node')

        self.state     = 'IDLE'   # mirrors /mission/status
        self.start_x   = None
        self.start_y   = None
        self.cur_x     = 0.0
        self.cur_y     = 0.0
        self.e_stop    = False

        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        self.create_subscription(LaserScan, '/scan',           self._scan_cb,    10)
        self.create_subscription(String,    '/mission/status', self._status_cb,  10)
        self.create_subscription(Odometry,  '/odom',           self._odom_cb,    10)

        self.get_logger().info('Safety node ready')

    # ── Mission state tracking ───────────────────────────────────────────────
    def _status_cb(self, msg: String):
        new_state = msg.data.split(':')[0].strip()  # e.g. "GOING", "RETURNING"
        if new_state != self.state:
            self.state = new_state
            self.start_x = None   # reset origin for each leg
            self.get_logger().info(f'Safety state: {self.state}')
            if self.state not in ('GOING', 'RETURNING'):
                self._publish(False)

    # ── Odometry tracking ───────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x, self.start_y = x, y
        self.cur_x, self.cur_y = x, y

    # ── Scan processing ─────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        if self.state not in ('GOING', 'RETURNING'):
            return

        # Disable near start (first 0.3m) and near goal (last 0.5m)
        if self.start_x is not None:
            traveled = math.sqrt(
                (self.cur_x - self.start_x) ** 2 +
                (self.cur_y - self.start_y) ** 2
            )
            if traveled < self.RAMP_UP or traveled > (self.TARGET_DIST - self.NEAR_GOAL):
                if self.e_stop:
                    self.get_logger().info('Safety disabled (ramp-up/near-goal zone)')
                    self._publish(False)
                return

        # Choose center angle: 0 = front (GOING), π = rear (RETURNING)
        center = 0.0 if self.state == 'GOING' else math.pi
        arc    = math.radians(self.ARC_DEG)

        obstacle = False
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < self.MIN_RANGE or r > self.MAX_RANGE:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            diff  = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(diff) <= arc:
                obstacle = True
                break

        if obstacle != self.e_stop:
            self.get_logger().warn(
                'OBSTACLE — STOPPING' if obstacle else 'Path clear — resuming'
            )
        self._publish(obstacle)

    def _publish(self, state: bool):
        self.e_stop = state
        msg = Bool()
        msg.data = state
        self.estop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()

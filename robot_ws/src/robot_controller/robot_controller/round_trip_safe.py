#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

TARGET_DISTANCE  = 2.50
SPEED_NORMAL     = 0.15
SPEED_SLOW       = 0.05
SLOWDOWN_DIST    = 0.20
MAX_RANGE        = 1.5
MIN_DETECT_DIST  = 0.50   # ignore very close readings (robot body pillars at 0.33m)
SCAN_HALF_DEG    = 20     # outer cone boundary
ARM_EXCL_DEG     = 12      # exclude ±7° center (arm at 0.71m@0° — robot's own structure)
MIN_POINTS       = 100      # require N points to confirm real obstacle

class RoundTripSafe(Node):
    def __init__(self):
        super().__init__('round_trip_safe')
        self.pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.start_x = self.start_y = None
        self.cur_x   = self.cur_y   = 0.0
        self.scan    = None
        self.state   = 'GOING'
        self.blocked = False
        self.shelf_arrival_time = None
        self.tick    = 0

        self.get_logger().info('=== RoundTripSafe starting ===')
        self.get_logger().info(f'    cone: ±{ARM_EXCL_DEG}° to ±{SCAN_HALF_DEG}° (annular — excludes arm at 0°)')
        self.get_logger().info(f'    range: {MIN_DETECT_DIST}–{MAX_RANGE}m  min_points: {MIN_POINTS}')
        self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x, self.start_y = x, y
            self.get_logger().info(f'[ODOM] Start locked: ({x:.4f}, {y:.4f})')
        self.cur_x, self.cur_y = x, y

    def scan_cb(self, msg):
        was_none = self.scan is None
        self.scan = msg
        if was_none:
            self.get_logger().info(f'[SCAN] First scan — {len(msg.ranges)} pts')

    def traveled(self):
        if self.start_x is None:
            return 0.0
        return math.sqrt((self.cur_x - self.start_x)**2 + (self.cur_y - self.start_y)**2)

    def obstacle_in(self, direction, max_range):
        """
        Annular cone: checks ±ARM_EXCL_DEG to ±SCAN_HALF_DEG around direction.
        Excludes center (robot arm/structure) but detects real obstacles at the sides.
        A person at 1m centered at 0° is still ±12° wide — well within ±7° to ±20°.
        """
        if self.scan is None:
            return False, 0, None, None
        center   = 0.0 if direction == 'front' else math.pi
        outer    = math.radians(SCAN_HALF_DEG)
        inner    = math.radians(ARM_EXCL_DEG)
        count    = 0
        closest_r, closest_a = float('inf'), None

        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < MIN_DETECT_DIST or r > max_range:
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            diff  = abs(math.atan2(math.sin(angle - center), math.cos(angle - center)))
            if inner < diff <= outer:   # annular zone: exclude center, keep sides
                count += 1
                if r < closest_r:
                    closest_r, closest_a = r, angle

        if closest_a is None:
            return False, 0, None, None
        blocked = count >= MIN_POINTS
        return blocked, count, closest_r, math.degrees(closest_a)

    def fmt(self, r, a):
        return 'none' if r is None else f'{r:.2f}m@{a:.1f}°'

    def loop(self):
        self.tick += 1
        verbose = (self.tick % 10 == 0)

        if self.start_x is None or self.scan is None:
            if verbose:
                self.get_logger().info('[WAIT] No odom/scan yet...')
            return
        if self.state == 'DONE':
            return

        dist      = self.traveled()
        remaining = TARGET_DISTANCE - dist

        if self.state == 'GOING':
            dyn_range = min(MAX_RANGE, max(0.3, remaining - 0.3))
            blocked, count, closest_r, closest_a = self.obstacle_in('front', dyn_range)

            if verbose:
                self.get_logger().info(
                    f'[GOING] dist={dist:.3f}m  remaining={remaining:.3f}m  '
                    f'dyn_range={dyn_range:.2f}m  points={count}  '
                    f'closest={self.fmt(closest_r, closest_a)}  '
                    f'speed={"SLOW" if remaining <= SLOWDOWN_DIST else "NORMAL"}'
                )

            if blocked:
                if not self.blocked:
                    self.get_logger().warn(
                        f'[GOING] ⛔ OBSTACLE {count}pts closest={self.fmt(closest_r, closest_a)} — STOPPED at {dist:.3f}m'
                    )
                    self.blocked = True
                self.pub.publish(Twist())
                return
            if self.blocked:
                self.get_logger().info(f'[GOING] ✅ Clear — resuming from {dist:.3f}m')
                self.blocked = False

            if dist >= TARGET_DISTANCE:
                self.pub.publish(Twist())
                self.get_logger().info(f'[GOING] ✅ AT SHELF — {dist:.3f}m — waiting 2s')
                self.state = 'AT_SHELF'
                return

            t = Twist()
            t.linear.x = SPEED_SLOW if remaining <= SLOWDOWN_DIST else SPEED_NORMAL
            self.pub.publish(t)

        elif self.state == 'AT_SHELF':
            if self.shelf_arrival_time is None:
                self.shelf_arrival_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.shelf_arrival_time).nanoseconds / 1e9
            if verbose:
                self.get_logger().info(f'[AT_SHELF] {elapsed:.1f}s / 2.0s')
            if elapsed >= 2.0:
                self.get_logger().info('[AT_SHELF] ✅ Starting RETURN')
                self.state   = 'RETURNING'
                self.blocked = False

        elif self.state == 'RETURNING':
            dist_from_home = dist
            dyn_range = min(MAX_RANGE, max(0.3, dist_from_home - 0.3))
            blocked, count, closest_r, closest_a = self.obstacle_in('rear', dyn_range)

            if verbose:
                self.get_logger().info(
                    f'[RETURNING] dist_from_home={dist_from_home:.3f}m  '
                    f'dyn_range={dyn_range:.2f}m  points={count}  '
                    f'closest={self.fmt(closest_r, closest_a)}  '
                    f'speed={"SLOW" if dist_from_home <= SLOWDOWN_DIST else "NORMAL"}'
                )

            if blocked:
                if not self.blocked:
                    self.get_logger().warn(
                        f'[RETURNING] ⛔ OBSTACLE {count}pts closest={self.fmt(closest_r, closest_a)} '
                        f'— STOPPED {dist_from_home:.3f}m from HOME'
                    )
                    self.blocked = True
                self.pub.publish(Twist())
                return
            if self.blocked:
                self.get_logger().info(f'[RETURNING] ✅ Clear — resuming {dist_from_home:.3f}m from HOME')
                self.blocked = False

            if dist_from_home <= 0.05:
                self.pub.publish(Twist())
                self.get_logger().info('✅✅✅ HOME reached — complete ✅✅✅')
                self.state = 'DONE'
                raise SystemExit

            t = Twist()
            t.linear.x = -SPEED_SLOW if dist_from_home <= SLOWDOWN_DIST else -SPEED_NORMAL
            self.pub.publish(t)


def main():
    rclpy.init()
    node = RoundTripSafe()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

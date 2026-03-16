"""
serial_bridge.py — ROS2 node that bridges Arduino Mega ↔ ROS2

  Arduino → RPi  (parsed here):
    "O:<x>,<y>,<th>,<vl>,<vr>\\n"                          → nav_msgs/Odometry + odom→base_footprint TF
    "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<qw>,<qx>,<qy>,<qz>\\n" → sensor_msgs/Imu  (BNO055 fusion)
       ax/ay/az  = linear accel m/s²  (gravity removed by BNO055)
       gx/gy/gz  = angular velocity rad/s
       qw/qx/qy/qz = absolute orientation quaternion from BNO055

  RPi → Arduino  (sent here):
    /cmd_vel (Twist) → "V:<vx>,<wz>\\n"
    watchdog: sends "V:0,0\\n" if no /cmd_vel for 500 ms

ROS2 parameters (set in launch file or command line):
  serial_port   (string, default /dev/ttyUSBArduinoMega)
  baud_rate     (int,    default 115200)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import serial

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSBArduinoMega')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # ── Serial port ─────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {port}: {e}')
            raise SystemExit(1)

        # ── Publishers ──────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry,  '/odom', 10)
        self.imu_pub  = self.create_publisher(Imu,       '/imu/data', qos)
        self.tf_br    = TransformBroadcaster(self)

        # ── Subscriber ──────────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._last_cmd_time = self.get_clock().now()

        # ── Watchdog timer (10 Hz) ───────────────────────────────────────────
        self.create_timer(0.1, self._watchdog_cb)

        # ── Background reader thread ─────────────────────────────────────────
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self._read_thread.start()

    # ── /cmd_vel callback ────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z
        line = f'V:{vx:.4f},{wz:.4f}\n'
        try:
            self.ser.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
        self._last_cmd_time = self.get_clock().now()

    # ── Watchdog — stop robot if /cmd_vel goes silent ────────────────────────
    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > 0.5:
            try:
                self.ser.write(b'V:0.0000,0.0000\n')
            except serial.SerialException:
                pass

    # ── Serial read loop (runs in background thread) ─────────────────────────
    def _read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                continue

            if line.startswith('O:'):
                self._handle_odom(line[2:])
            elif line.startswith('I:'):
                self._handle_imu(line[2:])
            elif line.startswith('ERR:'):
                self.get_logger().warn(f'Arduino: {line}')

    # ── Parse odometry line and publish ─────────────────────────────────────
    def _handle_odom(self, payload: str):
        try:
            x, y, th, vl, vr = [float(v) for v in payload.split(',')]
        except ValueError:
            return

        now = self.get_clock().now().to_msg()

        # Linear and angular velocity from wheel velocities
        # v = (vr + vl) / 2,  w = (vr - vl) / wheelbase
        wheelbase = 0.445
        v_linear  = (vr + vl) / 2.0
        v_angular = (vr - vl) / wheelbase

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (rotation around Z)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(th / 2.0)
        odom.pose.pose.orientation.w = math.cos(th / 2.0)

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        # Pose covariance diagonal (x, y, z, roll, pitch, yaw)
        odom.pose.covariance[0]  = 0.001   # x
        odom.pose.covariance[7]  = 0.001   # y
        odom.pose.covariance[14] = 1e6     # z  (2D robot, unknown)
        odom.pose.covariance[21] = 1e6     # roll
        odom.pose.covariance[28] = 1e6     # pitch
        odom.pose.covariance[35] = 0.01    # yaw

        # Twist covariance
        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(th / 2.0)
        tf.transform.rotation.w = math.cos(th / 2.0)
        self.tf_br.sendTransform(tf)

    # ── Parse IMU line and publish ────────────────────────────────────────────
    def _handle_imu(self, payload: str):
        # Format: ax,ay,az,gx,gy,gz,qw,qx,qy,qz  (10 fields from BNO055)
        try:
            ax, ay, az, gx, gy, gz, qw, qx, qy, qz = [
                float(v) for v in payload.split(',')]
        except ValueError:
            return

        imu = Imu()
        imu.header.stamp    = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        # Linear acceleration (gravity already removed by BNO055 onboard fusion)
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Angular velocity
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        # Absolute orientation quaternion from BNO055 fusion
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz

        # BNO055 fused orientation — small covariance (reliable)
        imu.orientation_covariance[0] = 0.002
        imu.orientation_covariance[4] = 0.002
        imu.orientation_covariance[8] = 0.002

        imu.linear_acceleration_covariance[0] = 0.01
        imu.linear_acceleration_covariance[4] = 0.01
        imu.linear_acceleration_covariance[8] = 0.01

        imu.angular_velocity_covariance[0] = 0.001
        imu.angular_velocity_covariance[4] = 0.001
        imu.angular_velocity_covariance[8] = 0.001

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.write(b'V:0.0000,0.0000\n')
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
serial_bridge.py — ROS2 node that bridges Arduino Mega ↔ ROS2

  Arduino → RPi  (parsed here):
    "O:<x>,<y>,<th>,<vl>,<vr>\\n"  → nav_msgs/Odometry + odom→base_footprint TF
    "SEQ:STEP:<n>:<desc>\\n"        → /sequence/status (String)
    "SEQ:RETRY:<n>\\n"              → /sequence/status (String)
    "SEQ:DONE\\n"                   → /sequence/status (String)
    "SEQ:FAIL:<reason>\\n"          → /sequence/status (String)

  RPi → Arduino  (sent here):
    /cmd_vel (Twist)        → "V:<vx>,<wz>\\n"
    /sequence/command (String) → forwarded as-is (e.g. "SEQ:START\\n")
    watchdog: sends "V:0,0\\n" if no /cmd_vel for 500 ms

Note: IMU is now read directly from RPi I2C by imu_node.py

ROS2 parameters (set in launch file or command line):
  serial_port   (string, default /dev/ttyUSBArduinoMega)
  baud_rate     (int,    default 115200)
"""

import math
import queue
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSBArduinoMega')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # ── Serial port (timeout=1 for blocking readline) ────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {port}: {e}')
            raise SystemExit(1)

        # ── Command queue: ROS callbacks → serial thread ─────────────────────
        self._cmd_queue    = queue.Queue()
        self._emergency    = False  # set True by safety_node

        # ── Publishers ──────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.seq_pub  = self.create_publisher(String, '/sequence/status', 10)
        self.tf_br    = TransformBroadcaster(self)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.seq_cmd_sub = self.create_subscription(
            String, '/sequence/command', self._seq_command_cb, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self._estop_cb, 10)
        self._last_cmd_time = time.monotonic()

        # ── Watchdog timer (10 Hz) ───────────────────────────────────────────
        self.create_timer(0.1, self._watchdog_cb)

        # ── Serial thread: ALL serial I/O runs here ──────────────────────────
        self._stop_serial = False
        self._serial_thread = threading.Thread(
            target=self._serial_loop, daemon=True)
        self._serial_thread.start()

        self._odom_count = 0
        self.get_logger().info('Serial bridge ready')

    # ── /emergency_stop callback — bypass queue and stop motors immediately ──
    def _estop_cb(self, msg: Bool):
        if msg.data and not self._emergency:
            self.get_logger().warn('EMERGENCY STOP — motors halted')
        self._emergency = msg.data
        if msg.data:
            self._cmd_queue.put(b'V:0.0000,0.0000\n')

    # ── /cmd_vel callback — puts command in queue ────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        if self._emergency:
            return  # drop velocity commands while emergency stop is active
        vx = msg.linear.x
        wz = msg.angular.z
        self._cmd_queue.put(f'V:{vx:.4f},{wz:.4f}\n'.encode())
        self._last_cmd_time = time.monotonic()

    # ── /sequence/command callback — forwards raw string to Arduino ──────────
    def _seq_command_cb(self, msg: String):
        cmd = msg.data.strip()
        self._cmd_queue.put(f'{cmd}\n'.encode())
        self.get_logger().info(f'Sequence command → Arduino: {cmd}')

    # ── Watchdog — sends stop if /cmd_vel goes silent ────────────────────────
    def _watchdog_cb(self):
        if time.monotonic() - self._last_cmd_time > 0.5:
            self._cmd_queue.put(b'V:0.0000,0.0000\n')

    # ── Serial loop — ONLY thread that touches self.ser ─────────────────────
    def _serial_loop(self):
        self.get_logger().info('Serial thread started')
        buf = b''
        while not self._stop_serial:
            # 1. Drain the command queue (writes)
            while True:
                try:
                    cmd = self._cmd_queue.get_nowait()
                    self.ser.write(cmd)
                except queue.Empty:
                    break
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')

            # 2. Read available bytes (blocking up to 1s via readline)
            try:
                raw = self.ser.readline()
            except (serial.SerialException, TypeError, OSError) as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)
                continue

            if not raw:
                continue

            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('O:'):
                self._handle_odom(line[2:])
                self._odom_count += 1
                if self._odom_count % 50 == 0:
                    self.get_logger().info(f'Odom published x{self._odom_count}: {line}')
            elif line.startswith('SEQ:'):
                self._handle_seq(line)
            elif line.startswith('ERR:') or line.startswith('INFO:'):
                self.get_logger().warn(f'Arduino: {line}')

    # ── Parse SEQ: line and publish ──────────────────────────────────────────
    def _handle_seq(self, line: str):
        msg = String()
        msg.data = line
        self.seq_pub.publish(msg)
        self.get_logger().info(f'Sequence: {line}')

    # ── Parse odometry line and publish ─────────────────────────────────────
    def _handle_odom(self, payload: str):
        try:
            x, y, th, vl, vr = [float(v) for v in payload.split(',')]
        except ValueError:
            return

        now = self.get_clock().now().to_msg()

        wheelbase = 0.445
        v_linear  = (vr + vl) / 2.0
        v_angular = (vr - vl) / wheelbase

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(th / 2.0)
        odom.pose.pose.orientation.w = math.cos(th / 2.0)

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        odom.pose.covariance[0]  = 0.001
        odom.pose.covariance[7]  = 0.001
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.01

        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

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

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_serial = True
        try:
            node.ser.write(b'V:0.0000,0.0000\n')
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

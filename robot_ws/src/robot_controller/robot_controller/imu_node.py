"""
imu_node.py — ROS2 node that reads MPU6050 directly via I2C on RPi
               and publishes sensor_msgs/Imu on /imu/data at 50 Hz

MPU6050 clone specifics:
  I2C address : 0x68
  WHO_AM_I    : 0x70 (non-standard — skip testConnection, wake directly)
  Accel range : ±2 g    → 16384 LSB/g    → m/s² = raw / 16384 * 9.81
  Gyro  range : ±250°/s → 131 LSB/(°/s)  → rad/s = raw / 131 * π/180
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

import smbus2

# MPU6050 registers
_PWR_MGMT_1   = 0x6B
_ACCEL_XOUT_H = 0x3B   # 6 bytes: ax_h ax_l ay_h ay_l az_h az_l
_GYRO_XOUT_H  = 0x43   # 6 bytes: gx_h gx_l gy_h gy_l gz_h gz_l

_ACCEL_SCALE  = 16384.0          # LSB/g  (±2 g default)
_GYRO_SCALE   = 131.0            # LSB/(°/s) (±250°/s default)
_G            = 9.80665          # m/s²
_DEG2RAD      = math.pi / 180.0


def _s16(high: int, low: int) -> int:
    """Combine two bytes into a signed 16-bit integer."""
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus',     1)
        self.declare_parameter('i2c_address', 0x68)

        bus_num   = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self._addr = self.get_parameter('i2c_address').get_parameter_value().integer_value

        try:
            self._bus = smbus2.SMBus(bus_num)
        except Exception as e:
            self.get_logger().fatal(f'Cannot open I2C bus {bus_num}: {e}')
            raise SystemExit(1)

        # Wake the MPU6050 (boots in sleep mode — write 0 to PWR_MGMT_1)
        try:
            self._bus.write_byte_data(self._addr, _PWR_MGMT_1, 0x00)
        except Exception as e:
            self.get_logger().fatal(
                f'Cannot communicate with MPU6050 at 0x{self._addr:02X}: {e}')
            raise SystemExit(1)

        self.get_logger().info(
            f'MPU6050 initialised on I2C bus {bus_num} address 0x{self._addr:02X}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Imu, '/imu/data', qos)

        self.create_timer(0.02, self._read_and_publish)   # 50 Hz

    def _read_and_publish(self):
        try:
            raw_a = self._bus.read_i2c_block_data(self._addr, _ACCEL_XOUT_H, 6)
            raw_g = self._bus.read_i2c_block_data(self._addr, _GYRO_XOUT_H,  6)
        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return

        ax = _s16(raw_a[0], raw_a[1]) / _ACCEL_SCALE * _G
        ay = _s16(raw_a[2], raw_a[3]) / _ACCEL_SCALE * _G
        az = _s16(raw_a[4], raw_a[5]) / _ACCEL_SCALE * _G

        gx = _s16(raw_g[0], raw_g[1]) / _GYRO_SCALE * _DEG2RAD
        gy = _s16(raw_g[2], raw_g[3]) / _GYRO_SCALE * _DEG2RAD
        gz = _s16(raw_g[4], raw_g[5]) / _GYRO_SCALE * _DEG2RAD

        imu = Imu()
        imu.header.stamp    = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.orientation_covariance[0] = -1.0   # no orientation estimate

        imu.linear_acceleration_covariance[0] = 0.01
        imu.linear_acceleration_covariance[4] = 0.01
        imu.linear_acceleration_covariance[8] = 0.01

        imu.angular_velocity_covariance[0] = 0.001
        imu.angular_velocity_covariance[4] = 0.001
        imu.angular_velocity_covariance[8] = 0.001

        self._pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._bus.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import smbus2
import time
import math

# Odometry covariance diagonals — reflects open-loop uncertainty
POSE_COVARIANCE = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e9, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e9, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e9, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
]
TWIST_COVARIANCE = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e9, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e9, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e9, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e9, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
]

# Angular threshold: when Nav2 sends combined linear+angular,
# turn takes priority if angular exceeds this value (rad/s)
ANGULAR_PRIORITY_THRESHOLD = 0.3


class WalterHardwareBridge(Node):
    def __init__(self):
        super().__init__('walter_hardware_bridge')

        # --- HARDWARE SETUP ---
        self.bus = smbus2.SMBus(1)
        self.motor_addr = 0x55
        self.imu_addr = 0x6A
        self.offset_y = 0.0
        self.init_imu()

        # --- ROS 2 COMMS ---
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.last_time = self.get_clock().now()

        self.create_timer(0.02, self.hardware_loop)  # 50 Hz
        self.get_logger().info("🟢 Walter Hardware Bridge Ready (Motors + IMU + Odometry)")

    # ==========================================
    # IMU
    # ==========================================
    def read8(self, addr, reg):
        try:
            return self.bus.read_byte_data(addr, reg)
        except:
            return 0

    def write8(self, addr, reg, val):
        try:
            self.bus.write_byte_data(addr, reg, val & 0xFF)
        except:
            pass

    def read16s(self, addr, lo, hi):
        l, h = self.read8(addr, lo), self.read8(addr, hi)
        v = (h << 8) | l
        return v - 65536 if v & 32768 else v

    def init_imu(self):
        self.get_logger().info("Calibrating Gyro... Please keep Walter still.")
        self.write8(self.imu_addr, 0x10, 0x06)
        self.write8(self.imu_addr, 0x11, 0x06)
        time.sleep(0.1)
        total = sum(self.read16s(self.imu_addr, 0x24, 0x25) * 0.00875 for _ in range(100))
        self.offset_y = total / 100.0
        self.get_logger().info(f"Gyro Calibrated! Offset: {self.offset_y:.4f}")

    # ==========================================
    # MOTORS
    # ==========================================
    def send_motor_cmd(self, cmd, spd=0, dur=0):
        try:
            if cmd == 26:
                self.bus.write_byte(self.motor_addr, cmd)
            else:
                self.bus.write_i2c_block_data(
                    self.motor_addr, cmd,
                    [spd & 0xFF, (spd >> 8) & 0xFF, dur & 0xFF, (dur >> 8) & 0xFF]
                )
        except Exception as e:
            self.get_logger().warning(f"Motor Error: {e}")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        self.vx = linear

        if linear == 0.0 and angular == 0.0:
            self.send_motor_cmd(26)
            return

        base_speed = min(150, max(0, int(abs(linear) * 300)))
        turn_speed = min(150, max(0, int(abs(angular) * 100)))

        # Pure rotation or Nav2 curve command with strong angular component
        if abs(linear) < 0.01 or abs(angular) > ANGULAR_PRIORITY_THRESHOLD:
            self.send_motor_cmd(23 if angular > 0 else 22, turn_speed, 0)
        else:
            self.send_motor_cmd(20, base_speed, 0)

    # ==========================================
    # MAIN LOOP — IMU + ODOMETRY
    # ==========================================
    def hardware_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Read gyro
        raw_dps = self.read16s(self.imu_addr, 0x24, 0x25) * 0.00875
        gyro_rads = math.radians(raw_dps - self.offset_y)

        # Dead reckoning
        self.th += gyro_rads * dt
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = POSE_COVARIANCE
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = gyro_rads
        odom.twist.covariance = TWIST_COVARIANCE
        self.odom_pub.publish(odom)

        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.angular_velocity.y = gyro_rads
        imu_msg.angular_velocity_covariance[8] = 0.05
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WalterHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_motor_cmd(26)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

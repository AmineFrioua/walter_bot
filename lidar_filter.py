import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import SetParametersResult
import math

BLIND_SPOTS_DEG = [101.0, 160.0, 200.0, 259.0]
MARGIN_DEG = 5.0

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # forward_arc_deg: total frontal arc to pass through (centred on 0°).
        # 360 = full scan. 180 = forward hemisphere only.
        # Can be changed at runtime from the Admin UI — no restart needed.
        self.declare_parameter('forward_arc_deg', 360.0)
        self._update_arc(self.get_parameter('forward_arc_deg').value)
        self.add_on_set_parameters_callback(self._on_param_change)

        # Pre-compute blind spot ranges in radians
        margin_rad = math.radians(MARGIN_DEG)
        self.blind_ranges = [
            (math.radians(deg) - margin_rad, math.radians(deg) + margin_rad)
            for deg in BLIND_SPOTS_DEG
        ]

        # Use sensor_data QoS (BEST_EFFORT) to match:
        #   • sllidar_ros2 publisher  → publishes /scan      with SensorDataQoS (BEST_EFFORT)
        #   • slam_toolbox subscriber → subscribes /scan_filtered with sensor_data QoS (BEST_EFFORT)
        # Using the default RELIABLE QoS causes a QoS incompatibility and zero data flows.
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data)

        self._recv_count = 0
        self._last_log_ns = 0

        spots = ', '.join([f'{d}°' for d in BLIND_SPOTS_DEG])
        self.get_logger().info(
            f'LiDAR filter ready — {self._arc_label()}, blind spots: {spots} (±{MARGIN_DEG}°)'
        )
        self.get_logger().info(
            'QoS: BEST_EFFORT (sensor_data) on /scan subscriber and /scan_filtered publisher'
        )
        self.get_logger().info(
            'Waiting for first /scan message…'
        )

    def _update_arc(self, deg):
        self._fwd_arc_deg = float(deg)
        self._fwd_half = math.radians(deg / 2.0) if deg < 360.0 else None

    def _arc_label(self):
        return f'{self._fwd_arc_deg:.0f}° arc' if self._fwd_half else 'full 360°'

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'forward_arc_deg':
                self._update_arc(p.value)
                self.get_logger().info(f'Forward arc -> {self._arc_label()}')
        return SetParametersResult(successful=True)

    def scan_cb(self, msg):
        self._recv_count += 1

        # Log first scan so the terminal confirms data is flowing
        if self._recv_count == 1:
            self.get_logger().info(
                f'✅ First /scan received ({len(msg.ranges)} rays, '
                f'range {msg.range_min:.2f}–{msg.range_max:.2f} m) — forwarding to /scan_filtered'
            )

        # Heartbeat every 10 s so you can see the filter is alive
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns > 10_000_000_000:
            self._last_log_ns = now_ns
            self.get_logger().info(
                f'lidar_filter heartbeat — scans received: {self._recv_count}'
            )

        ranges = list(msg.ranges)
        angle  = msg.angle_min

        for i in range(len(ranges)):
            # Forward-arc gate
            if self._fwd_half is not None:
                signed = angle % (2 * math.pi)
                if signed > math.pi:
                    signed -= 2 * math.pi
                if abs(signed) > self._fwd_half:
                    ranges[i] = float('inf')
                    angle += msg.angle_increment
                    continue

            # Blind-spot gate
            norm = angle % (2 * math.pi)
            for lo, hi in self.blind_ranges:
                if lo <= norm <= hi:
                    ranges[i] = float('inf')
                    break

            angle += msg.angle_increment

        msg.ranges = ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

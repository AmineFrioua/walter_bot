import rclpy
from rclpy.node import Node
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

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        spots = ', '.join([f'{d}°' for d in BLIND_SPOTS_DEG])
        self.get_logger().info(
            f'LiDAR filter active — {self._arc_label()}, blind spots: {spots} (±{MARGIN_DEG}°)'
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

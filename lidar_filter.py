import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

BLIND_SPOTS_DEG = [101.0, 160.0, 200.0, 259.0]
MARGIN_DEG = 5.0

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # Optional forward-only arc.  Set via:
        #   ros2 run walter_bot lidar_filter --ros-args -p forward_arc_deg:=180.0
        # Default 360 = full scan (backward compatible).
        self.declare_parameter('forward_arc_deg', 360.0)
        fwd_arc = self.get_parameter('forward_arc_deg').value
        self._fwd_half = math.radians(fwd_arc / 2.0) if fwd_arc < 360.0 else None

        # Pre-compute blind spot ranges in radians
        margin_rad = math.radians(MARGIN_DEG)
        self.blind_ranges = [
            (math.radians(deg) - margin_rad, math.radians(deg) + margin_rad)
            for deg in BLIND_SPOTS_DEG
        ]

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        spots = ', '.join([f'{d}°' for d in BLIND_SPOTS_DEG])
        fwd_label = f'{fwd_arc:.0f}° arc' if self._fwd_half else 'full 360°'
        self.get_logger().info(
            f'LiDAR filter active — {fwd_label}, blind spots: {spots} (±{MARGIN_DEG}°)'
        )

    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        angle  = msg.angle_min

        for i in range(len(ranges)):
            # Forward-arc gate: blank beams outside the configured half-angle
            if self._fwd_half is not None:
                signed = angle % (2 * math.pi)
                if signed > math.pi:
                    signed -= 2 * math.pi
                if abs(signed) > self._fwd_half:
                    ranges[i] = float('inf')
                    angle += msg.angle_increment
                    continue

            # Blind-spot gate: blank mount-obstruction angles
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

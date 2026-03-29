import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        # Listen to the raw scanner
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_cb, 10)
        # Publish the clean data to the standard topic
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info("🕶️ LiDAR Blinders Active: Filtering pillars at 85°-135° and 225°-275°.")

    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        
        for i in range(len(ranges)):
            angle_rad = msg.angle_min + (i * msg.angle_increment)
            angle_deg = (math.degrees(angle_rad) + 360) % 360

            # LEFT BEAM: 85° to 135°
            # RIGHT BEAM: 225° to 275°
            if (105 < angle_deg < 145) or (215 < angle_deg < 255):
                ranges[i] = float('inf') # Erase the laser beam

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

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
        
        # Configuration for the blind spots
        self.margin = 5.0  # Erase 5 degrees on either side of the pillar
        self.get_logger().info(f"🕶️ LiDAR Blinders Active: Filtering pillars at 101°, 160°, 200°, and 259° (±{self.margin}°).")

    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        
        for i in range(len(ranges)):
            angle_rad = msg.angle_min + (i * msg.angle_increment)
            angle_deg = (math.degrees(angle_rad) + 360) % 360

            # LEFT PILLARS
            pillar_1_left = 101.0
            pillar_2_left = 160.0
            
            # RIGHT PILLARS (Symmetrical)
            pillar_1_right = 360.0 - 101.0  # 259.0
            pillar_2_right = 360.0 - 160.0  # 200.0

            # Check if the current laser beam hits any of the 4 pillars
            if (abs(angle_deg - pillar_1_left) < self.margin) or \
               (abs(angle_deg - pillar_2_left) < self.margin) or \
               (abs(angle_deg - pillar_1_right) < self.margin) or \
               (abs(angle_deg - pillar_2_right) < self.margin):
                
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
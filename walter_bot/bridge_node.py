import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .communication.uart_comm import UARTCommunication
from .config import UART_PORT, UART_BAUDRATE

class WalterBridge(Node):
    def __init__(self):
        super().__init__('walter_bridge')
        
        # Initialize your existing UART logic
        self.uart = UARTCommunication(port=UART_PORT, baudrate=UART_BAUDRATE)
        try:
            self.uart.connect()
            self.get_logger().info("UART Connected to ESP32")
        except Exception as e:
            self.get_logger().error(f"Failed to connect UART: {e}")

        # Subscribe to standard ROS velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("Walter Bridge Node Started. Listening for cmd_vel...")

    def cmd_vel_callback(self, msg):
        # Translate ROS Twist (m/s) to your ESP32 Motor Commands (0-255)
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Basic tank drive math (you can tune this later)
        left_speed = int((linear - angular) * 255)
        right_speed = int((linear + angular) * 255)
        
        # Cap speeds at 255 and -255
        left_speed = max(min(left_speed, 255), -255)
        right_speed = max(min(right_speed, 255), -255)

        # Send command via your existing UART class
        command = f"MOTOR:{left_speed},{right_speed}\n"
        self.uart.send(command)
        self.get_logger().debug(f'Sent to ESP32: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = WalterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.uart.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

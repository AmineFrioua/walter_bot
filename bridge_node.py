import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # --- SERIAL CONFIGURATION ---
        # Update this if your ESP32 is on a different port (e.g., /dev/ttyACM0)
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"🟢 Connected to Walter's Motors on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"🔴 Failed to connect to serial port: {e}")
            self.ser = None

        # --- ROS 2 SUBSCRIPTION ---
        # Listen to the '/cmd_vel' topic for driving commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return

        # Get forward/backward (linear.x) and left/right (angular.z) speeds
        linear = msg.linear.x
        angular = msg.angular.z

        # Create the string to send to the ESP32. 
        # Example output: "V,0.50,0.00\n"
        command = f"V,{linear:.2f},{angular:.2f}\n"

        try:
            self.ser.write(command.encode('utf-8'))
            # self.get_logger().info(f"Sent: {command.strip()}") # Uncomment to debug
        except Exception as e:
            self.get_logger().error(f"Error sending to motors: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge_node = MotorBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge_node.ser:
            # Stop motors before shutting down
            bridge_node.ser.write("V,0.00,0.00\n".encode('utf-8'))
            bridge_node.ser.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

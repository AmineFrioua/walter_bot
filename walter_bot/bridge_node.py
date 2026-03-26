import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import time

# Matching your ESP32 commands from slave_infra.ino
CMD_FWD = 20
CMD_REV = 21
CMD_RIGHT = 22
CMD_LEFT = 23
CMD_STOP = 26
I2C_ADDR = 0x55

class WalterBridge(Node):
    def __init__(self):
        super().__init__('walter_bridge')
        
        # Initialize I2C Bus 1
        self.bus = SMBus(1)
        self.get_logger().info(f"I2C Connected to ESP32 at address {hex(I2C_ADDR)}")

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("Walter Bridge Node Started. Listening for cmd_vel...")

    def send_i2c_cmd(self, cmd, speed=0, duration=0):
        try:
            if cmd == CMD_STOP:
                self.bus.write_byte(I2C_ADDR, cmd)
            else:
                # Pack speed and duration into bytes (Little Endian)
                data = [speed & 0xFF, speed >> 8, duration & 0xFF, duration >> 8]
                self.bus.write_i2c_block_data(I2C_ADDR, cmd, data)
        except Exception as e:
            self.get_logger().error(f"I2C Write Failed: {e}")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Convert speed to 0-255 PWM (0.5 m/s roughly equals 255 PWM)
        speed_pwm = int(min(max(abs(v) / 0.5, 0.0), 1.0) * 255)
        turn_pwm = int(min(max(abs(omega) / 2.0, 0.0), 1.0) * 255)

        # Map ROS Twist to your discrete ESP32 states
        if v > 0.0:
            self.send_i2c_cmd(CMD_FWD, speed_pwm, 0)
            self.get_logger().debug(f'Moving FORWARD: {speed_pwm}')
        elif v < 0.0:
            self.send_i2c_cmd(CMD_REV, speed_pwm, 0)
            self.get_logger().debug(f'Moving REVERSE: {speed_pwm}')
        elif omega > 0.0:
            self.send_i2c_cmd(CMD_LEFT, turn_pwm, 0)
            self.get_logger().debug(f'Turning LEFT: {turn_pwm}')
        elif omega < 0.0:
            self.send_i2c_cmd(CMD_RIGHT, turn_pwm, 0)
            self.get_logger().debug(f'Turning RIGHT: {turn_pwm}')
        else:
            self.send_i2c_cmd(CMD_STOP)
            self.get_logger().debug('STOPPING')

def main(args=None):
    rclpy.init(args=args)
    node = WalterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_i2c_cmd(CMD_STOP) # Safety stop on exit
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

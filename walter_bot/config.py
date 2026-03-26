"""
Configuration for Raspberry Pi Robot Controller
"""

# I2C Configuration
I2C_BUS = 1
LSM6DSV_ADDRESS = 0x6A
BQ25820_ADDRESS = 0x6B

# UART Communication
UART_PORT = '/dev/ttyAMA0'
UART_BAUDRATE = 115200

# Sensor Configuration
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
LIDAR_MAX_RANGE = 12.0
LIDAR_TIMEOUT = 5.0
LIDAR_GRAPH_ENABLED = True

# Motor Limits
MAX_MOTOR_SPEED = 255
MIN_MOTOR_SPEED = 0

# Safety Parameters
OBSTACLE_DISTANCE_THRESHOLD = 0.5  # meters
MAX_ROTATION_RATE = 50             # degrees/second

# Display Configuration
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 480
REFRESH_RATE = 30  # Hz

# Monitoring Configuration
SAMPLE_INTERVAL = 1.0
LOG_INTERVAL = 900.0
LOG_FILE = "robot_log.txt"

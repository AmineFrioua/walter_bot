#!/bin/bash

# This automatically kills all background processes when you press Ctrl+C
trap "echo '🔴 Shutting down Walter...'; kill 0" EXIT

echo "🟢 Loading ROS 2..."
source /opt/ros/humble/setup.bash

echo "🧠 Starting Hardware Brain (Odometry & Motors)..."
python3 bridge_node.py &

echo "👀 Starting RPLidar..."
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser_frame -p angle_compensate:=true -r scan:=scan_raw &

echo "🦴 Building Skeleton..."
ros2 run tf2_ros static_transform_publisher 0 0 0.4 0 0 0 base_link laser_frame > /dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link imu_link > /dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint > /dev/null 2>&1 &

echo "🕶️ Equipping LiDAR Blinders..."
python3 lidar_filter.py &

echo "🗺️ Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py &

echo "📡 Starting Foxglove Bridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

echo "✅ Walter is fully online. Press Ctrl+C to stop."
wait

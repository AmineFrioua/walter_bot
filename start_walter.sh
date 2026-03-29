#!/bin/bash

# --- GRACEFUL SHUTDOWN ---
cleanup() {
    echo ""
    echo "🔴 Shutting down Walter cleanly..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "💤 Walter is asleep."
    exit
}
trap cleanup SIGINT SIGTERM

# --- LOAD ROS 2 ---
echo "🟢 Loading ROS 2..."
source /opt/ros/humble/setup.bash

# --- 1. START THE EYES FIRST ---
echo "👀 Starting RPLidar..."
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser_frame -p angle_compensate:=true &

echo "⏳ Waiting 8 seconds for the LiDAR motor to reach max speed..."
sleep 8

# --- 2. HARDWARE, SKELETON & FILTER ---
echo "🧠 Starting Hardware Brain (Odometry & Motors)..."
python3 /ros2_ws/bridge_node.py &

echo "🦴 Building Skeleton..."
ros2 run robot_state_publisher robot_state_publisher /ros2_ws/urdf/walter.urdf &

echo "🕶️ Equipping LiDAR Blinders..."
python3 /ros2_ws/lidar_filter.py &

# --- 3. THE HEAVY ALGORITHMS ---
echo "🗺️ Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/slam_params.yaml > /dev/null 2>&1 &

echo "📡 Starting Foxglove Bridge..."
ros2 run rosbridge_server rosbridge_websocket > /dev/null 2>&1 &
ros2 run rosapi rosapi_node > /dev/null 2>&1 &

echo "✅ Walter is fully online. Press Ctrl+C to stop."
wait

#!/bin/bash

# --- GRACEFUL SHUTDOWN ---
cleanup() {
    echo ""
    echo "🔴 Shutting down Walter's Brain cleanly..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "💤 Brain is asleep."
    exit
}
trap cleanup SIGINT SIGTERM

# --- LOAD ROS 2 ---
echo "🟢 Loading ROS 2..."
source /opt/ros/humble/setup.bash

# --- 1. HARDWARE, SKELETON & FILTER ---
echo "🧠 Starting Hardware Brain (Odometry & Motors)..."
python3 /ros2_ws/bridge_node.py &

echo "🦴 Building Skeleton..."
ros2 run robot_state_publisher robot_state_publisher /ros2_ws/urdf/walter.urdf &

echo "🕶️ Equipping LiDAR Blinders..."
python3 /ros2_ws/lidar_filter.py &

# Brief pause to let the Python scripts settle
sleep 2

# --- 2. THE HEAVY ALGORITHMS ---
echo "🗺️ Starting SLAM Toolbox..."
# NOTE: Ensure your slam_params_file path matches where you saved it!
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/slam_params.yaml > /dev/null 2>&1 &

echo "📡 Starting Foxglove Bridge..."
ros2 run rosbridge_server rosbridge_websocket > /dev/null 2>&1 &
ros2 run rosapi rosapi_node > /dev/null 2>&1 &

echo "✅ Walter's Brain is fully online. Press Ctrl+C to stop."
wait
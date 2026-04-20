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
source /ros2_ws/install/setup.bash

# --- STAGE 1: HARDWARE ---
echo "📡 Starting LiDAR..."
ros2 run sllidar_ros2 sllidar_node \
  --ros-args \
  -p serial_port:=/dev/ttyAMA0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser_frame \
  -p angle_compensate:=true \
  -p scan_mode:=Standard &

echo "🧠 Starting Hardware Brain (Odometry & Motors)..."
python3 /ros2_ws/bridge_node.py &

echo "🦴 Building Skeleton..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat /ros2_ws/urdf/walter.urdf)" &

echo "🕶️ Equipping LiDAR Blinders..."
python3 /ros2_ws/lidar_filter.py &

echo "⏳ Waiting for hardware to settle..."
sleep 3

# --- STAGE 2: MAPPING & BRIDGE ---
echo "🗺️ Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/ros2_ws/slam_params.yaml > /dev/null 2>&1 &

echo "📺 Starting Foxglove Bridge..."
ros2 run rosbridge_server rosbridge_websocket > /dev/null 2>&1 &
ros2 run rosapi rosapi_node > /dev/null 2>&1 &

echo "⏳ Waiting for SLAM to settle..."
sleep 5

# --- STAGE 3: NAVIGATION ---
echo "🧭 Starting Nav2..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False \
  autostart:=True \
  params_file:=/ros2_ws/config/nav2_params.yaml > /dev/null 2>&1 &

echo "✅ Walter is fully online. Press Ctrl+C to stop."
wait

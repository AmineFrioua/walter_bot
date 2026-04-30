#!/bin/bash
# Usage:
#   bash start_brain.sh             → mapping mode (default)
#   bash start_brain.sh navigate    → navigation mode (needs a saved map)
#   bash start_brain.sh navigate my_map  → navigate with specific map name

MODE="${1:-mapping}"
MAP_NAME="${2:-room_map}"
MAP_FILE="/ros2_ws/maps/${MAP_NAME}"

# --- GRACEFUL SHUTDOWN ---
cleanup() {
    echo ""
    echo "🔴 Shutting down Walter cleanly..."
    if [ "$MODE" = "mapping" ]; then
        echo "💾 Auto-saving map before exit..."
        bash /ros2_ws/save_map.sh "${MAP_NAME}_shutdown" 2>/dev/null || true
    fi
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "💤 Walter is asleep."
    exit
}
trap cleanup SIGINT SIGTERM

# --- LOAD ROS 2 ---
echo "🟢 Loading ROS 2 (mode: $MODE)..."
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# --- STAGE 1: HARDWARE (always) ---
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

# --- STAGE 2: rosbridge (always) ---
echo "📺 Starting rosbridge..."
ros2 run rosbridge_server rosbridge_websocket > /dev/null 2>&1 &
ros2 run rosapi rosapi_node > /dev/null 2>&1 &

# --- STAGE 3: mode-specific ---
if [ "$MODE" = "mapping" ]; then
    echo "🗺️ Starting SLAM Toolbox (mapping)..."
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:=/ros2_ws/slam_params.yaml > /dev/null 2>&1 &

    echo "⏳ Waiting for SLAM to settle..."
    sleep 5

    # Auto-save map every 2 minutes in the background
    echo "💾 Auto-save enabled (every 2 min → maps/${MAP_NAME})"
    (while true; do
        sleep 120
        bash /ros2_ws/save_map.sh "$MAP_NAME" > /dev/null 2>&1 \
            && echo "💾 [$(date +%H:%M:%S)] Map auto-saved → maps/${MAP_NAME}" \
            || echo "⚠️  Map auto-save failed"
    done) &

    echo "✅ Walter is mapping. Drive around, then press Save Map in the UI."

elif [ "$MODE" = "navigate" ]; then
    if [ ! -f "${MAP_FILE}.yaml" ]; then
        echo "❌ Map not found: ${MAP_FILE}.yaml"
        echo "   Run in mapping mode first, save the map, then restart with: navigate ${MAP_NAME}"
        exit 1
    fi
    echo "🗺️ Starting Map Server (${MAP_NAME})..."
    ros2 run nav2_map_server map_server \
      --ros-args -p yaml_filename:="${MAP_FILE}.yaml" > /dev/null 2>&1 &

    echo "📍 Starting AMCL (localisation)..."
    ros2 run nav2_amcl amcl \
      --ros-args --params-file /ros2_ws/config/nav2_params.yaml > /dev/null 2>&1 &

    sleep 4

    echo "🧭 Starting Nav2..."
    ros2 launch nav2_bringup navigation_launch.py \
      use_sim_time:=False \
      autostart:=True \
      params_file:=/ros2_ws/config/nav2_params.yaml > /dev/null 2>&1 &

    echo "✅ Walter is navigating with map: ${MAP_NAME}"
fi

echo "Press Ctrl+C to stop."
wait

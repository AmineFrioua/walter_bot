#!/bin/bash
# Usage:
#   bash start_brain.sh              → mapping mode (default)
#   bash start_brain.sh navigate     → production mode (AMCL + Nav2, no SLAM)
#   bash start_brain.sh navigate my_map

MODE="${1:-mapping}"
MAP_NAME="${2:-room_map}"
MAP_FILE="/ros2_ws/maps/${MAP_NAME}"

# --- GRACEFUL SHUTDOWN ---
_SHUTDOWN=0
cleanup() {
    # Re-entrancy guard — ignore repeated Ctrl+C presses during cleanup
    [ "$_SHUTDOWN" -eq 1 ] && return
    _SHUTDOWN=1
    trap '' SIGINT SIGTERM   # block further signals while we clean up

    echo ""
    echo "🔴 Shutting down Walter cleanly..."

    # In mapping mode: attempt map save NOW, while SLAM Toolbox is still
    # starting to shut down (it received the same SIGINT but takes ~2 s to
    # fully exit, giving us a brief window to call the save service).
    if [ "$MODE" = "mapping" ]; then
        echo "💾 Saving map before exit..."
        bash /ros2_ws/save_map.sh "${MAP_NAME}" 2>/dev/null \
            && echo "✅ Map saved." \
            || echo "⚠️  Map save skipped (SLAM already stopped)."
    fi

    # Kill all background jobs (auto-save loop, nodes, rosbridge…)
    kill $(jobs -p) 2>/dev/null
    wait 2>/dev/null
    echo "💤 Walter is asleep."
    exit 0
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

# --- STAGE 2: bridges (always) ---
#
# Two bridges run side-by-side:
#   • rosbridge_websocket  (port 9090)  → used by the web UI (map.html, drive.html,
#     logs.html, etc.).  Web UI needs /odom, /cmd_vel, /imu/data_raw, /scan,
#     /scan_filtered and /map, so we don't whitelist here.
#   • foxglove_bridge      (port 8765)  → used by Foxglove Studio.  Whitelisted
#     to ONLY the four user-facing topics so the panel list stays clean:
#         /scan, /scan_filtered, /map, and /robot (= /tf + /tf_static +
#         /robot_description — the three topics Foxglove needs to render the
#         robot model).
#
# Logs go to /tmp/<name>.log so you can tail them when something is broken:
#     docker exec -it walter_dev tail -f /tmp/slam.log
#     docker exec -it walter_dev tail -f /tmp/rosbridge.log
#     docker exec -it walter_dev tail -f /tmp/foxglove_bridge.log
echo "📺 Starting rosbridge   (port 9090, web UI)..."
ros2 run rosbridge_server rosbridge_websocket > /tmp/rosbridge.log 2>&1 &
ros2 run rosapi rosapi_node > /tmp/rosapi.log 2>&1 &

echo "🦊 Starting foxglove_bridge (port 8765, Foxglove Studio — whitelisted)..."
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args \
  -p port:=8765 \
  -p topic_whitelist:="['/scan','/scan_filtered','/map','/tf','/tf_static','/robot_description']" \
  -p send_buffer_limit:=10000000 \
  > /tmp/foxglove_bridge.log 2>&1 &

# ─────────────────────────────────────────────────────────────────────────────
# MAPPING MODE — SLAM Toolbox only, no Nav2
# Heavy during use, but you only do this once to build the map.
# ─────────────────────────────────────────────────────────────────────────────
if [ "$MODE" = "mapping" ]; then
    echo "🗺️  Starting SLAM Toolbox (mapping mode — no Nav2)..."
    # Log to /tmp/slam.log instead of /dev/null so you can debug "map not
    # accumulating" issues with:  docker exec walter_dev tail -f /tmp/slam.log
    # Common failures visible in the log:
    #   • "[WARN] No transform from [base_link] to [map]"  → TF tree broken
    #   • "[WARN] Failed to compute odom pose"             → /odom not flowing
    #   • repeated "Got scan" with no "Adding scan to graph" → minimum_travel_*
    #     too high (robot didn't move enough between scans)
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:=/ros2_ws/slam_params.yaml > /tmp/slam.log 2>&1 &

    echo "⏳ Waiting for SLAM to settle..."
    sleep 5

    # Auto-save map every 2 minutes
    echo "💾 Auto-save active (every 2 min → maps/${MAP_NAME})"
    (while true; do
        sleep 120
        bash /ros2_ws/save_map.sh "$MAP_NAME" > /dev/null 2>&1 \
            && echo "💾 [$(date +%H:%M:%S)] Auto-saved → maps/${MAP_NAME}" \
            || echo "⚠️  Auto-save failed"
    done) &

    echo ""
    echo "✅ Mapping. Drive around, then Save Map in the UI (or Ctrl+C)."
    echo "   When done: restart with  ./run_walter.sh  (auto-detects saved map)"

# ─────────────────────────────────────────────────────────────────────────────
# NAVIGATE MODE — AMCL + Nav2, no SLAM
# This is the production/daily mode. Much lighter than mapping.
# Requires a saved map in /ros2_ws/maps/<name>.yaml
# ─────────────────────────────────────────────────────────────────────────────
elif [ "$MODE" = "navigate" ]; then
    if [ ! -f "${MAP_FILE}.yaml" ]; then
        echo "❌ Map not found: ${MAP_FILE}.yaml"
        echo "   Run  ./run_walter.sh mapping  first, save the map, then restart."
        exit 1
    fi

    echo "🗺️  Starting Nav2 with saved map: ${MAP_NAME}"
    echo "    (map_server + AMCL + planner + controller — no SLAM)"

    # nav2_bringup's bringup_launch includes map_server, AMCL, and full Nav2 stack.
    # Logs go to /tmp/nav2.log for debugging.
    ros2 launch nav2_bringup bringup_launch.py \
      use_sim_time:=False \
      autostart:=True \
      map:="${MAP_FILE}.yaml" \
      params_file:=/ros2_ws/config/nav2_params.yaml > /tmp/nav2.log 2>&1 &

    echo "⏳ Waiting for Nav2 to initialise..."
    sleep 8

    echo ""
    echo "✅ Walter is live. AMCL localising on saved map."
    echo "   Use Delivery mode in the UI to send navigation goals."
fi

echo "Press Ctrl+C to stop."
wait

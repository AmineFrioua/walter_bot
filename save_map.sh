#!/bin/bash
# Save current SLAM map to maps/<name>.pgm + .yaml
# Run inside Docker while SLAM Toolbox is active.
#
# Strategy (two approaches, first one that works wins):
#
#  1. SLAM Toolbox /slam_toolbox/save_map service  ← preferred
#     Asks SLAM Toolbox to write the map itself — no QoS negotiation,
#     no lifecycle node, no subscription timeout.  Always works as long
#     as the SLAM node is alive.
#
#  2. nav2_map_server map_saver_cli  ← fallback
#     Subscribes to /map and writes the files.  Needs use_transient_local_qos
#     because SLAM Toolbox publishes /map with TRANSIENT_LOCAL durability.
#     Can still time-out if the map topic hasn't published recently.

MAP_DIR="/ros2_ws/maps"
MAP_NAME="${1:-room_map}"
OUT="$MAP_DIR/$MAP_NAME"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

mkdir -p "$MAP_DIR"

echo "💾 Saving map as '$MAP_NAME'..."

# ── Method 1: SLAM Toolbox service ────────────────────────────────────────────
echo "   Trying SLAM Toolbox save_map service..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$OUT'}}" \
    2>/dev/null

# Check if the .pgm file was actually written (service call exit code is
# unreliable — it returns 0 even on failure, so we check the file instead)
if [ -f "${OUT}.pgm" ]; then
    echo "✅ Map saved via SLAM Toolbox: ${OUT}.pgm + .yaml"
    exit 0
fi

# ── Method 2: map_saver_cli (fallback) ────────────────────────────────────────
echo "   SLAM Toolbox service didn't produce a file — trying map_saver_cli..."
ros2 run nav2_map_server map_saver_cli \
    -f "$OUT" \
    --ros-args \
    -p save_map_timeout:=15.0 \
    -p use_transient_local_qos:=true

if [ -f "${OUT}.pgm" ]; then
    echo "✅ Map saved via map_saver_cli: ${OUT}.pgm + .yaml"
    exit 0
fi

echo "❌ Map save failed — is SLAM Toolbox running?"
echo "   Check with: ros2 node list | grep slam"
exit 1

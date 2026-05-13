#!/bin/bash
# Save current SLAM map via the SLAM Toolbox service.
# Run inside Docker while SLAM Toolbox is active.
#
# Uses /slam_toolbox/save_map — SLAM Toolbox writes the pgm/yaml itself.
# No QoS negotiation, no lifecycle node, no subscription timeout.

MAP_DIR="/ros2_ws/maps"
MAP_NAME="${1:-room_map}"
OUT="$MAP_DIR/$MAP_NAME"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

mkdir -p "$MAP_DIR"

echo "💾 Saving map as '$MAP_NAME'..."

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$OUT'}}" 2>/dev/null

# Service exit code is unreliable — check the file itself
if [ -f "${OUT}.pgm" ]; then
    echo "✅ Map saved: ${OUT}.pgm + ${OUT}.yaml"
    exit 0
fi

echo "❌ Map save failed — is SLAM Toolbox running?"
echo "   Check: ros2 node list | grep slam"
exit 1

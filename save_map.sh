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

# Quick service-availability check (3 s) — fail fast if SLAM isn't running
# rather than hanging for 30 s waiting for DDS to time out.
if ! timeout 3 ros2 service list 2>/dev/null | grep -q "/slam_toolbox/save_map"; then
    echo "❌ Map save failed — /slam_toolbox/save_map not found."
    echo "   Is SLAM Toolbox running?  Check: ros2 node list | grep slam"
    exit 1
fi

timeout 8 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$OUT'}}" 2>/dev/null

# Service exit code is unreliable — check the file itself
if [ -f "${OUT}.pgm" ]; then
    echo "✅ Map saved: ${OUT}.pgm + ${OUT}.yaml"
    exit 0
fi

echo "❌ Map save failed — SLAM Toolbox did not write the file."
echo "   Check: ros2 node list | grep slam"
exit 1

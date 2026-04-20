#!/bin/bash
# Save current SLAM map and switch to localization mode.
# Run inside Docker while SLAM is active.

MAP_DIR="/ros2_ws/maps"
MAP_NAME="${1:-room_map}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

mkdir -p "$MAP_DIR"

echo "💾 Saving map as '$MAP_NAME'..."
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p save_map_timeout:=10.0

if [ $? -eq 0 ]; then
    echo "✅ Map saved: $MAP_DIR/$MAP_NAME.pgm + .yaml"
    echo ""
    echo "To switch to localization mode, restart with:"
    echo "  start_brain.sh localization $MAP_NAME"
else
    echo "❌ Map save failed — is SLAM running?"
    exit 1
fi

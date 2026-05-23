#!/bin/bash
# diagnose_map.sh — diagnose "the map isn't accumulating" problems
# ───────────────────────────────────────────────────────────────────────────
# Run INSIDE the walter_dev Docker container:
#   docker exec -it walter_dev bash /ros2_ws/diagnose_map.sh
#
# Walks through every link in the chain:
#   /scan → lidar_filter → /scan_filtered → SLAM Toolbox → /map
# and prints clear ✅ / ❌ for each step.

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null

PASS="✅"
FAIL="❌"
WARN="⚠️ "

echo "════════════════════════════════════════════════════════════"
echo "  WALTER MAP PIPELINE DIAGNOSTIC"
echo "  $(date)"
echo "════════════════════════════════════════════════════════════"

# ── 1. List nodes ─────────────────────────────────────────────────────────
echo ""
echo "── 1. ROS nodes running ────────────────────────────────────"
NODES=$(timeout 3 ros2 node list 2>/dev/null)
echo "$NODES" | sed 's/^/    /'

need_node() {
    if echo "$NODES" | grep -q "$1"; then
        echo "  $PASS $1 is running"
    else
        echo "  $FAIL $1 is NOT running"
    fi
}
echo ""
need_node "/sllidar_node"
need_node "/lidar_filter"
need_node "/walter_hardware_bridge"
need_node "/slam_toolbox"

# ── 2. /scan ──────────────────────────────────────────────────────────────
echo ""
echo "── 2. /scan (raw LiDAR) ────────────────────────────────────"
SCAN_HZ=$(timeout 3 ros2 topic hz /scan 2>&1 | grep -oP 'rate:\s+\K[0-9.]+' | head -1)
if [ -n "$SCAN_HZ" ]; then
    echo "  $PASS /scan publishing at ${SCAN_HZ} Hz"
else
    echo "  $FAIL /scan not publishing — check sllidar_node + /dev/ttyAMA0"
fi

# ── 3. /scan_filtered ─────────────────────────────────────────────────────
echo ""
echo "── 3. /scan_filtered (after lidar_filter) ──────────────────"
SF_HZ=$(timeout 3 ros2 topic hz /scan_filtered 2>&1 | grep -oP 'rate:\s+\K[0-9.]+' | head -1)
if [ -n "$SF_HZ" ]; then
    echo "  $PASS /scan_filtered publishing at ${SF_HZ} Hz"
else
    echo "  $FAIL /scan_filtered not publishing — lidar_filter is dead or /scan missing"
fi

# ── 4. TF tree ────────────────────────────────────────────────────────────
echo ""
echo "── 4. TF tree (required by SLAM to ground scans in the map) "
TF_TARGETS=("base_link" "laser_frame" "odom")
for f in "${TF_TARGETS[@]}"; do
    if timeout 3 ros2 run tf2_ros tf2_echo map "$f" 2>&1 | grep -q "Translation"; then
        echo "  $PASS map → $f  available"
    else
        # Try odom→f as a fallback (during mapping startup, map→base_link is the link
        # SLAM creates; before SLAM publishes it, only odom→base_link exists)
        if timeout 3 ros2 run tf2_ros tf2_echo odom "$f" 2>&1 | grep -q "Translation"; then
            echo "  $WARN map → $f  not yet available, but odom → $f works  (SLAM hasn't published map transform yet)"
        else
            echo "  $FAIL map → $f  AND odom → $f  both unavailable — TF tree broken"
        fi
    fi
done

# ── 5. /map ───────────────────────────────────────────────────────────────
echo ""
echo "── 5. /map topic ──────────────────────────────────────────"
if timeout 3 ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    echo "  $PASS /map topic exists"
    # Try to grab one message (with transient_local QoS)
    MAP_INFO=$(timeout 4 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /map nav_msgs/msg/OccupancyGrid 2>/dev/null \
               | grep -E "width|height|resolution" | head -3 | tr '\n' ' ')
    if [ -n "$MAP_INFO" ]; then
        echo "  $PASS Latest /map :  ${MAP_INFO}"
    else
        echo "  $FAIL /map exists but no message received in 4s — SLAM not publishing"
    fi
else
    echo "  $FAIL /map topic does NOT exist — SLAM Toolbox isn't publishing it"
    echo "       (are you in MAPPING mode? Navigate mode also has /map but from map_server.)"
fi

# ── 6. Bridges ────────────────────────────────────────────────────────────
echo ""
echo "── 6. Bridges (where the UI / Foxglove read from) ─────────"
if pgrep -f rosbridge_websocket > /dev/null; then
    echo "  $PASS rosbridge_websocket alive on :9090   (log /tmp/rosbridge.log)"
else
    echo "  $FAIL rosbridge_websocket NOT running"
fi
if pgrep -f foxglove_bridge > /dev/null; then
    echo "  $PASS foxglove_bridge alive on :8765      (log /tmp/foxglove_bridge.log)"
else
    echo "  $WARN foxglove_bridge NOT running        (only matters if you use Foxglove Studio)"
fi

# ── 7. SLAM log tail ──────────────────────────────────────────────────────
echo ""
echo "── 7. Last 6 lines of SLAM log ─────────────────────────────"
if [ -f /tmp/slam.log ]; then
    tail -6 /tmp/slam.log | sed 's/^/    /'
else
    echo "    /tmp/slam.log not found — SLAM never started this session"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  If /scan_filtered is publishing AND /map exists AND TF is"
echo "  OK but you still don't see the map grow in Foxglove/UI:"
echo "  → drive the robot at least minimum_travel_distance (0.3 m)"
echo "    so SLAM accepts a new scan into the graph."
echo "  → /map updates only every map_update_interval = 8 s."
echo "  → in Foxglove use the 'Map' panel, topic /map."
echo "  → in the web UI map.html: open browser dev-tools console;"
echo "    look for 'mapMsgTotal' increasing.  If it stays at 0,"
echo "    rosbridge is dropping the message — restart it."
echo "════════════════════════════════════════════════════════════"

#!/bin/bash
# Usage:
#   ./run_walter.sh                    → auto: navigate if map exists, else mapping
#   ./run_walter.sh mapping            → force mapping mode (build new map)
#   ./run_walter.sh navigate           → force navigate mode (prod)
#   ./run_walter.sh navigate my_map    → navigate with a specific saved map

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-auto}"
MAP_NAME="${2:-room_map}"
MAP_FILE="$SCRIPT_DIR/maps/${MAP_NAME}.yaml"

# Auto-detect mode: navigate if map file already exists, otherwise map
if [ "$MODE" = "auto" ]; then
  if [ -f "$MAP_FILE" ]; then
    MODE="navigate"
    echo "🗺️  Found saved map: maps/${MAP_NAME}.yaml → starting in navigate mode"
  else
    MODE="mapping"
    echo "🔍 No saved map found → starting in mapping mode"
    echo "   Drive around, save the map, then restart to auto-navigate."
  fi
fi

cleanup() {
  echo ""
  echo "🔴 Shutting down Walter..."
  kill "$WEB_PID" 2>/dev/null
  docker rm -f walter_dev 2>/dev/null
  exit
}
trap cleanup SIGINT SIGTERM

echo "⚡ Powering LiDAR (GPIO 17)..."
sudo pinctrl set 17 op dh

echo "🌐 Starting web UI on http://0.0.0.0:5000 ..."
python3 "$SCRIPT_DIR/web_server.py" &
WEB_PID=$!

echo "⏳ Waiting for LiDAR to power up..."
sleep 3

echo "🐳 Starting Walter in Docker (mode: $MODE)..."
docker rm -f walter_dev 2>/dev/null && echo "  (removed stale container)" || true
docker run -it --rm \
  --name walter_dev \
  --privileged \
  --network host \
  -v "$SCRIPT_DIR":/ros2_ws \
  walter_dev \
  bash /ros2_ws/start_brain.sh "$MODE" "$MAP_NAME"

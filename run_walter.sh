#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

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

echo "🐳 Starting Walter in Docker..."
docker rm -f walter_dev 2>/dev/null && echo "  (removed stale container)" || true
docker run -it --rm \
  --name walter_dev \
  --privileged \
  --network host \
  -v "$SCRIPT_DIR":/ros2_ws \
  walter_dev \
  bash /ros2_ws/start_brain.sh

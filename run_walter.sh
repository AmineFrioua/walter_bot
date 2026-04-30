#!/bin/bash

echo "⚡ Powering LiDAR (GPIO 17)..."
sudo pinctrl set 17 op dh

echo "⏳ Waiting for LiDAR to power up..."
sleep 3

echo "🐳 Starting Walter in Docker..."
docker run -it --rm \
  --name walter_dev \
  --privileged \
  --network host \
  -v $(pwd):/ros2_ws \
  walter_dev \
  bash /ros2_ws/start_brain.sh

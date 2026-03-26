#!/bin/bash
docker run -it --rm \
    --name walter_dev \
    --net=host \
    --privileged \
    -v /dev:/dev \
    -v $(pwd):/ros2_ws/src/walter_bot \
    walter_ros

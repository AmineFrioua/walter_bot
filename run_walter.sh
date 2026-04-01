#!/bin/bash
docker run -it --rm \
  --name walter_dev \
  --privileged \
  --network host \
  -v $(pwd):/ros2_ws \
  walter_dev

FROM ros:humble-ros-base

# 1. Install tools and IMMEDIATELY clear the cache to save GBs of space
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    python3-smbus \
    i2c-tools \
    ros-humble-rplidar-ros \
    ros-humble-slam-toolbox \
    ros-humble-rosbridge-suite \
    ros-humble-teleop-twist-keyboard \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 2. Install python dependencies
RUN pip3 install pyserial

WORKDIR /ros2_ws
COPY . .

CMD ["bash"]

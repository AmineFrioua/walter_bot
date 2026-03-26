FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    python3-smbus \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# Install your actual Python requirements
RUN pip3 install numpy rplidar-roboticia pyserial smbus2 spidev RPi.GPIO

WORKDIR /ros2_ws

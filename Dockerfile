# Use the official ROS 2 Humble base
FROM ros:humble-ros-base

# 1. Install System Tools & Navigation Stack
# Combining into one RUN command to keep the image size small
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    python3-pip \
    python3-serial \
    python3-smbus \
    i2c-tools \
    python3-spidev \
    ros-humble-rplidar-ros \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rosbridge-suite \
    ros-humble-teleop-twist-keyboard \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Python Dependencies
# smbus2 is often more stable for Python 3 I2C
RUN pip3 install --no-cache-dir \
    pyserial \
    rich \
    psutil \
    smbus2 \
    flask  \
    rich psutil

# 3. Setup Workspace
WORKDIR /ros2_ws

# Copy all project files into the container
COPY . .

# Source ROS 2 automatically for every new terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

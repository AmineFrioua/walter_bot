# 🤖 Project Walter: Autonomous Service Robot

Walter is a ROS 2 Humble-powered service robot designed for autonomous mapping, navigation, and weight-sensitive delivery. It utilizes a **staggered-boot architecture** to overcome Raspberry Pi 4 USB power limitations and a **Dual-Processor HAL** (Hardware Abstraction Layer) using an ESP32.

---

##  System Architecture

### **High-Level Overview**
This graph shows the physical and logical flow from the user down to the motors.

```mermaid
graph TD
    User((User/Operator)) -->|Select Goal| UI[Web UI / Foxglove]
    UI -->|Action Goal| Nav2[Nav2 Navigation Stack]
    
    subgraph RPi4 [Raspberry Pi 4 - Dockerized ROS 2]
        Nav2 -->|Path Planning| SLAM[SLAM Toolbox]
        SLAM -->|Map/Pose| Nav2
        Filter[LiDAR Filter Node] -->|Clean Scan| SLAM
        Filter -->|Clean Scan| Nav2
        Bridge[Hardware Bridge Node] -->|Odometry/IMU| Nav2
    end

    Lidar[RPLidar A1] -->|Raw Scan| Filter
    Nav2 -->|cmd_vel| Bridge
    Bridge -->|Serial| ESP32[ESP32 Slave]

    subgraph Hardware [Physical Robot]
        ESP32 -->|PWM| Motors[Drive Motors]
        LoadCell[Load Cell/SPI] -->|Weight Data| ESP32
        Cliff[Cliff Sensors] -->|Safety Stop| ESP32
        IMU[MPU6050] -->|Heading| ESP32
    end


### ** Low Level ROS 2 Node **

graph LR
    subgraph Sensing
        N1[rplidar_node] -->|/scan_raw| N2[lidar_filter.py]
    end

    subgraph Perception
        N2 -->|/scan| N4[SLAM Toolbox]
        N2 -->|/scan| N5[Nav2 Stack]
    end

    subgraph Control
        N3[bridge_node.py] -->|/odom| N4
        N3 -->|/weight| M[Mission Manager]
        N5 -->|/cmd_vel| N3
    end

    subgraph World
        N4 -->|/map| N5
        N4 -->|/tf| N5
    end


## Setup and installation 

### Docker Setup 

`docker build -t walter_dev .`

```
docker run -it --rm \
  --name walter_dev \
  --privileged \
  --network host \
  -v $(pwd):/ros2_ws \
  walter_dev
```

## Lunch Sequence 

### Terminal 1 

```
docker exec -it walter_dev bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser_frame -p angle_compensate:=true
```

### Terminal 2 

```
docker exec -it walter_dev bash
./start_brain.sh
```

### Terminal 3 

```
docker exec -it walter_dev bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False autostart:=True params_file:=/ros2_ws/config/nav2_params.yaml
```

### Terminal 4

```
docker exec -it walter_dev bash
python3 roomba_mapper.py
```

## Key design 

No Reverse: Walter is configured in nav2_params.yaml with min_vel_x: 0.0. It will only move forward or turn in place.

Staggered Boot: To prevent the 80008002 error, the LiDAR is given a "head start" before SLAM Toolbox spikes the CPU.

Safety First: Cliff detection is handled on the ESP32 level; if a cliff is detected, the ESP32 will override ROS commands and stop the motors instantly.

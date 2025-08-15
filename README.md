# GPU‑SLAM Bot — Starter Project (ROS2 + C++ + CUDA + Gazebo)

A minimal, end‑to‑end template to simulate a robot in Gazebo and build a real‑time, GPU‑accelerated 2D SLAM pipeline. It subscribes to sensor_msgs/LaserScan, integrates scans into a log‑odds occupancy grid on the GPU, and publishes nav_msgs/OccupancyGrid. Gazebo SDF world is set with a flat ground plane and a sun.

## Scope: mapping only (no loop closure). Easy to extend into FastSLAM (particles on GPU) or ICP/scan‑matching later.

### 1) System Architecture

- Gazebo Sim → publishes /scan (LiDAR) and /tf

- gpu_slam_node (ROS2 C++ + CUDA)

Subscribe: /scan (sensor_msgs/msg/LaserScan)

(Optional) Subscribe: /tf for robot pose (or use odom)

GPU kernels:

Ray integration kernel: cast each beam, mark freespace and endpoint occupied (atomicAdd log‑odds)

Publish: /map (nav_msgs/msg/OccupancyGrid)

- RViz2 → visualize /map

### 2) Directory Layout

```
ws/
 ├─ src/
 │   └─ gpu_slam_bot/
 │       ├─ CMakeLists.txt
 │       ├─ package.xml
 │       ├─ include/gpu_slam_bot/gpu_grid.hpp
 │       ├─ src/gpu_grid.cu
 │       ├─ src/gpu_slam_node.cpp
 │       ├─ launch/sim.launch.py
 │       └─ worlds/empty_lidar.world
 └─ ...
```

### 3) Build Prereqs

- ROS2 Humble (or newer)

- CUDA 11+ (works on most modern NVIDIA GPUs)

- CMake ≥ 3.18 (for first‑class CUDA language)

- Gazebo (Fortress/Garden) or classic Gazebo; adjust launch accordingly



### 4) To Run:

```bash

cd ws

source /opt/ros/humble/setup.bash #Replace humble with your version

colcon build --symlink-install #Build the workspace with CUDA support

```

If CUDA is not found, you might need to set: 

```bash

colcon build --symlink-install --cmake-args -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc

```

After a successful build:

```bash

source install/setup.bash

```

Run the launch file: 

```bash

cd ws

ros2 launch gpu_slam_bot sim.launch.py

```


Extra checks:

```bash

nvcc --version #Verify CUDA

ros2 pkg list | grep gpu_slam_bot #Verify package is found:


```
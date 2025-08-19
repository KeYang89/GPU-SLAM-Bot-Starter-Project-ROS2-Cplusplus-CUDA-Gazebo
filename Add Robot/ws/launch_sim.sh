#!/bin/bash

# Stop immediately if any command fails
set -e

pkill -9 gzserver || true
pkill -9 gzclient || true


# Regenerate URDF from XACRO
echo "Generating URDF from XACRO..."
ros2 run xacro xacro src/gpu_slam_bot/urdf/gpu_slam_bot.urdf.xacro -o src/gpu_slam_bot/urdf/gpu_slam_bot.urdf

# Clean previous builds
echo "Cleaning previous build, install, and log directories..."
rm -rf build install log

# Build workspace
echo "Building workspace..."
colcon build --symlink-install

# Source the setup file
echo "Sourcing workspace..."
source install/setup.bash

# Launch the simulation
echo "Launching simulation..."
ros2 launch gpu_slam_bot sim.launch.py


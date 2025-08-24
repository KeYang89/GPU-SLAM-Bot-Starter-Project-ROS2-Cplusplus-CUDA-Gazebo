#!/bin/bash
set -e

pkill -9 gzserver || true
pkill -9 gzclient || true

echo "Generating world file..."
python3 src/gpu_slam_bot/scripts/generate_maze_world.py

echo "Generating URDF from XACRO..."
xacro src/gpu_slam_bot/urdf/gpu_slam_bot.urdf.xacro -o src/gpu_slam_bot/urdf/gpu_slam_bot.urdf

echo "Cleaning previous build..."
rm -rf build install log

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace..."
source install/setup.bash

echo "Launching simulation..."
ros2 launch gpu_slam_bot sim.launch.py world:=src/gpu_slam_bot/worlds/maze_generated.world

#!/bin/bash
set -e

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
for cmd in colcon ros2 python3; do
    if ! command_exists "$cmd"; then
        echo "Error: $cmd is not installed. Please install it (e.g., 'sudo apt install ros-foxy-ros-base python3-colcon-common-extensions')."
        exit 1
    fi
done

# Check for required ROS packages
for pkg in gazebo_ros gazebo_ros2_control robot_state_publisher xacro; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "Error: ROS package $pkg is not installed. Install with 'sudo apt install ros-foxy-$pkg'."
        exit 1
    fi
done

# Kill any running Gazebo
if pgrep -x "gzserver" > /dev/null || pgrep -x "gzclient" > /dev/null; then
    echo "Stopping running Gazebo processes..."
    pkill -TERM gzserver || true
    pkill -TERM gzclient || true
    sleep 1
fi

echo "Cleaning previous build..."
rm -rf build install log

# Source ROS 2
echo "Sourcing ROS 2 Foxy environment..."
source /opt/ros/foxy/setup.bash

echo "Generating world file..."
if [ -f "src/gpu_slam_bot/scripts/generate_maze_world.py" ]; then
    python3 src/gpu_slam_bot/scripts/generate_maze_world.py
    if [ ! -f "src/gpu_slam_bot/worlds/maze_generated.world" ]; then
        echo "Error: Failed to generate maze_generated.world"
        exit 1
    fi
else
    echo "Error: generate_maze_world.py not found in src/gpu_slam_bot/scripts/"
    exit 1
fi

echo "Building workspace..."
colcon build --symlink-install --packages-select gpu_slam_bot
echo "Sourcing workspace..."
source install/setup.bash

echo "Generating URDF from XACRO..."
if [ -f "src/gpu_slam_bot/urdf/gpu_slam_bot.urdf.xacro" ]; then
    ros2 run xacro xacro "src/gpu_slam_bot/urdf/gpu_slam_bot.urdf.xacro" -o "src/gpu_slam_bot/urdf/gpu_slam_bot.urdf"
    if [ ! -f "src/gpu_slam_bot/urdf/gpu_slam_bot.urdf" ]; then
        echo "Error: Failed to generate gpu_slam_bot.urdf"
        exit 1
    fi
else
    echo "Error: gpu_slam_bot.urdf.xacro not found in src/gpu_slam_bot/urdf/"
    exit 1
fi

echo "Launching simulation..."
ros2 launch gpu_slam_bot sim.launch.py world:="src/gpu_slam_bot/worlds/maze_generated.world"

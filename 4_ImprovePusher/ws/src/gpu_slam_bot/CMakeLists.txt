cmake_minimum_required(VERSION 3.5)
project(gpu_slam_bot LANGUAGES CXX CUDA)

# Set C++ and CUDA standard to 14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# ------------------------
# Build CUDA library
# ------------------------
add_library(gpu_grid
  src/gpu_grid.cu
)
set_target_properties(gpu_grid PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

# ------------------------
# Build GPU SLAM node executable
# ------------------------
add_executable(gpu_slam_node
  src/gpu_slam_node.cpp
)
ament_target_dependencies(gpu_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
  trajectory_msgs
)
target_link_libraries(gpu_slam_node gpu_grid)

# ------------------------
# Build obstacle mover executable
# ------------------------
add_executable(move_obstacle
  src/move_obstacle.cpp
)
ament_target_dependencies(move_obstacle
  rclcpp
  gazebo_msgs
)

# ------------------------
# Build auto_move node executable
# ------------------------
add_executable(auto_move
  src/auto_move.cpp
)
ament_target_dependencies(auto_move
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(auto_move gpu_grid)

# ------------------------
# Build auto_lifter node executable
# ------------------------
add_executable(auto_lifter
  src/auto_lifter.cpp
)
ament_target_dependencies(auto_lifter
  rclcpp
  sensor_msgs
  std_msgs
)

# ------------------------
# Install executables
# ------------------------
install(TARGETS
  gpu_slam_node
  move_obstacle
  auto_move
  auto_lifter
  gpu_grid
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# Install resources
# ------------------------
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# ------------------------
# Build autonomous gripper controller executable
# ------------------------
add_executable(autonomous_gripper_controller
  src/autonomous_gripper_controller.cpp
)

ament_target_dependencies(autonomous_gripper_controller
  rclcpp
  trajectory_msgs
  sensor_msgs
  gazebo_msgs
  nav_msgs
  geometry_msgs
)

# Update your install section to include the new executable:
install(TARGETS
  gpu_slam_node
  move_obstacle
  auto_move
  auto_lifter
  autonomous_gripper_controller  # Add this line
  gpu_grid
  DESTINATION lib/${PROJECT_NAME}
)
# ------------------------
# Export package
# ------------------------
ament_package()
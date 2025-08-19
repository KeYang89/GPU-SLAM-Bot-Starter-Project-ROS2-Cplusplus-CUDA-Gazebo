cmake_minimum_required(VERSION 3.5)
project(gpu_slam_bot LANGUAGES CXX CUDA)

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
)
target_link_libraries(gpu_slam_node gpu_grid)

# Build obstacle mover executable
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
# Install executables
# ------------------------
install(TARGETS
  gpu_slam_node
  move_obstacle
  auto_move              # <- add auto_move here
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
# Export package
# ------------------------
ament_package()

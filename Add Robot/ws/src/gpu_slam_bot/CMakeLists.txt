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

# Include directories
include_directories(
  include
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Build CUDA library
add_library(gpu_grid
  src/gpu_grid.cu
)
set_target_properties(gpu_grid PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

# Build node executable
add_executable(gpu_slam_node
  src/gpu_slam_node.cpp
)

# Link ROS2 dependencies
ament_target_dependencies(gpu_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link CUDA library to node
target_link_libraries(gpu_slam_node gpu_grid)

# Install targets
install(TARGETS
  gpu_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install worlds
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes
# install(DIRECTORY meshes/
  # DESTINATION share/${PROJECT_NAME}/meshes
# )
# Export package
ament_package()

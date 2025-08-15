cmake_minimum_required(VERSION 3.18)
project(gpu_slam_bot LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# CUDA settings
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_library(gpu_grid STATIC src/gpu_grid.cu)
set_target_properties(gpu_grid PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(gpu_grid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(gpu_slam_node src/gpu_slam_node.cpp)

target_link_libraries(gpu_slam_node
  PRIVATE gpu_grid)

target_include_directories(gpu_slam_node PRIVATE include)

ament_target_dependencies(gpu_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS gpu_slam_node gpu_grid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
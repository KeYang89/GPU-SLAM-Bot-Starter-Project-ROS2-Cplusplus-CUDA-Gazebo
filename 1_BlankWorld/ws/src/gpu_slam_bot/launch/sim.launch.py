from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gpu_slam_bot')

    world_path = os.path.join(pkg_share, 'worlds', 'empty_lidar.world')

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Launch GPU SLAM Node
        Node(
            package='gpu_slam_bot',
            executable='gpu_slam_node',
            name='gpu_slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gpu_slam')

    # Path to the world file
    world_path = os.path.join(pkg_share, 'worlds', 'empty_lidar.world')

    return LaunchDescription([
        # Start Gazebo with the empty_lidar.world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),

        # Launch the GPU SLAM Node
        Node(
            package='gpu_slam',
            executable='gpu_slam_node',
            name='gpu_slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])

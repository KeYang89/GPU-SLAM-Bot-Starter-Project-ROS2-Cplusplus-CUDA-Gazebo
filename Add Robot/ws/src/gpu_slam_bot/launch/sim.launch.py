from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gpu_slam_bot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'gpu_slam_bot.urdf.xacro')

    # Path to gazebo_ros package launch file
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch')

    return LaunchDescription([
        # Launch Gazebo with ROS 2 integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
            ),
            launch_arguments={'verbose': 'true'}.items()
        ),

        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_file]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'gpu_slam_bot',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.05'
            ],
            output='screen'
        ),

        # Start GPU SLAM node
        Node(
            package='gpu_slam_bot',
            executable='gpu_slam_node',
            name='gpu_slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

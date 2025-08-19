from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import random

def generate_launch_description():
    pkg_share = get_package_share_directory('gpu_slam_bot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'gpu_slam_bot.urdf.xacro')

    # Convert Xacro to URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    ld = LaunchDescription()

    # Launch Gazebo
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            launch_arguments={'verbose': 'true'}.items()
        )
    )

    # Publish robot state
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        )
    )

    # Spawn the robot
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'gpu_slam_bot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.05'
            ],
            output='screen'
        )
    )

    # Start GPU SLAM node
    ld.add_action(
        Node(
            package='gpu_slam_bot',
            executable='gpu_slam_node',
            name='gpu_slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    )

    # Start auto_move C++ node
    ld.add_action(
        Node(
            package='gpu_slam_bot',
            executable='auto_move',
            name='auto_move',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    )

    # Spawn obstacles **after a short delay**
    for i in range(1, 6):
        x = round(random.uniform(0.5, 2.0), 2)
        y = round(random.uniform(0.5, 2.0), 2)
        moving = random.choice([True, False])
        entity_name = f'obstacle_{i}'

        # TimerAction to delay obstacle spawn
        ld.add_action(
            TimerAction(
                period=2.0 + i*0.5,  # stagger spawn times
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=[
                            '-entity', entity_name,
                            '-file', os.path.join(pkg_share, 'models', 'box.sdf'),
                            '-x', str(x),
                            '-y', str(y),
                            '-z', '0.1'
                        ],
                        output='screen'
                    )
                ]
            )
        )

        # If obstacle should move
        if moving:
            ld.add_action(
                TimerAction(
                    period=3.0 + i*0.5,  # delay before moving
                    actions=[
                        Node(
                            package='gpu_slam_bot',
                            executable='move_obstacle',  # your C++ move_obstacle node
                            name=f'move_{entity_name}',
                            output='screen',
                            parameters=[{'entity_name': entity_name}]
                        )
                    ]
                )
            )

    return ld

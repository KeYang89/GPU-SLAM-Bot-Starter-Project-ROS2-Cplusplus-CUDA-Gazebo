from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import random
import math
import tempfile
import re

def generate_launch_description():
    pkg_share = get_package_share_directory('gpu_slam_bot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'gpu_slam_bot.urdf.xacro')
    maze_world_file = os.path.join(pkg_share, 'worlds', 'maze.world')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    ld = LaunchDescription()

    # Launch Gazebo with maze world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            launch_arguments={'verbose': 'true', 'world': maze_world_file}.items()
        )
    )

    # Robot state publisher
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
            arguments=['-entity', 'gpu_slam_bot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.05'],
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

    # Start auto_move node
    ld.add_action(
        Node(
            package='gpu_slam_bot',
            executable='auto_move',
            name='auto_move',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    )

    # Morandi color palette
    colors = {
        'dusty_red':    ('0.76 0.56 0.56 1', '0.76 0.56 0.56 1'),
        'sage_green':   ('0.64 0.69 0.62 1', '0.64 0.69 0.62 1'),
        'muted_blue':   ('0.60 0.69 0.77 1', '0.60 0.69 0.77 1'),
        'beige':        ('0.78 0.74 0.66 1', '0.78 0.74 0.66 1'),
        'lavender':     ('0.72 0.68 0.75 1', '0.72 0.68 0.75 1'),
        'gray_olive':   ('0.60 0.60 0.54 1', '0.60 0.60 0.54 1')
    }

    # Larger spawn range inside maze
    spawn_range_x = (-3.5, 3.5)
    spawn_range_y = (-3.5, 3.5)

    positions = []

    def generate_position(existing, min_dist=0.4):
        while True:
            x = round(random.uniform(*spawn_range_x), 2)
            y = round(random.uniform(*spawn_range_y), 2)
            if all(math.hypot(x-ex, y-ey) >= min_dist for ex, ey in existing):
                existing.append((x, y))
                return x, y

    original_sdf = os.path.join(pkg_share, 'models', 'box.sdf')

    def spawn_colored_box(entity_name, color_name, x, y, z, period):
        ambient, diffuse = colors[color_name]
        with open(original_sdf, 'r') as f:
            sdf_content = f.read()

        # Replace any existing ambient/diffuse inside <material> tags
        sdf_content = re.sub(r'<ambient>.*?</ambient>', f'<ambient>{ambient}</ambient>', sdf_content, flags=re.DOTALL)
        sdf_content = re.sub(r'<diffuse>.*?</diffuse>', f'<diffuse>{diffuse}</diffuse>', sdf_content, flags=re.DOTALL)

        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.sdf')
        temp_file.write(sdf_content.encode('utf-8'))
        temp_file.close()

        return TimerAction(
            period=period,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', entity_name,
                               '-file', temp_file.name,
                               '-x', str(x), '-y', str(y), '-z', str(z)],
                    output='screen'
                )
            ]
        )

    # Spawn 5-10 boxes with random Morandi colors
    num_boxes = random.randint(10, 20)
    for i in range(num_boxes):
        x, y = generate_position(positions)
        color_name = random.choice(list(colors.keys()))
        ld.add_action(spawn_colored_box(f'{color_name}_box_{i+1}', color_name, x, y, 0.1, 2.0 + i*0.5))

    return ld

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

    # Start autonomous gripper controller
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gpu_slam_bot',
                    executable='autonomous_gripper_controller',
                    name='autonomous_gripper_controller',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'min_cycle_time': 2.0},
                        {'max_cycle_time': 3.5},
                        {'gripper_open_angle': 0.4},
                        {'gripper_closed_angle': 0.0},
                        {'gripper_push_angle': -0.3},
                        {'lifter_down': 0.0},
                        {'lifter_up': 0.15},
                        {'lifter_push_height': 0.08},
                        {'wall_detection_distance': 0.3},
                        {'obstacle_push_distance': 0.2}
                    ]
                )
            ]
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

    # Box types with adjusted mass and friction for easier pushing/picking
    box_types = {
        'standard': {'size': '0.2 0.2 0.2', 'mass': 0.4, 'friction': 0.4},
        'small': {'size': '0.15 0.15 0.15', 'mass': 0.2, 'friction': 0.45},
        'flat': {'size': '0.25 0.25 0.1', 'mass': 0.3, 'friction': 0.5},
        'tall': {'size': '0.15 0.15 0.3', 'mass': 0.5, 'friction': 0.35}
    }

    # Define safe spawn area within maze walls
    safe_spawn_range_x = (-9, 9)
    safe_spawn_range_y = (-9, 9)
    wall_buffer = 0.3

    positions = []
    stack_positions = []

    def is_within_maze_bounds(x, y):
        """Check if the position is within the maze boundaries"""
        return (safe_spawn_range_x[0] + wall_buffer <= x <= safe_spawn_range_x[1] - wall_buffer and
                safe_spawn_range_y[0] + wall_buffer <= y <= safe_spawn_range_y[1] - wall_buffer)

    def generate_position(existing, min_dist=0.4):
        """Generate a position within maze boundaries and away from other boxes"""
        while True:
            x = round(random.uniform(*safe_spawn_range_x), 2)
            y = round(random.uniform(*safe_spawn_range_y), 2)
            if (is_within_maze_bounds(x, y) and
                all(math.hypot(x - ex, y - ey) >= min_dist for ex, ey in existing)):
                existing.append((x, y))
                return x, y

    def generate_stack_position(existing, min_dist=0.8):
        """Generate positions for intentional stacks with more spacing within maze boundaries"""
        while True:
            x = round(random.uniform(*safe_spawn_range_x), 2)
            y = round(random.uniform(*safe_spawn_range_y), 2)
            if (is_within_maze_bounds(x, y) and
                all(math.hypot(x - ex, y - ey) >= min_dist for ex, ey in existing)):
                existing.append((x, y))
                return x, y

    def create_stackable_sdf(box_type, color_name):
        """Create SDF content for stackable boxes with proper physics"""
        ambient, diffuse = colors[color_name]
        size = box_types[box_type]['size']
        mass = box_types[box_type]['mass']
        friction = box_types[box_type]['friction']
        
        sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="stackable_box">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>
    
    <link name="box_link">
      <pose>0 0 0 0 0 0</pose>
      
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <collision name="box_collision">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction}</mu>
              <mu2>{friction}</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>500000</kp>
              <kd>50</kd>
              <max_vel>1.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      
      <visual name="box_visual">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return sdf_content

    def spawn_stackable_box(entity_name, box_type, color_name, x, y, z, period):
        """Spawn a stackable box with the given properties"""
        sdf_content = create_stackable_sdf(box_type, color_name)
        
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

    # Spawn individual scattered boxes
    num_scattered_boxes = random.randint(15, 25)
    for i in range(num_scattered_boxes):
        x, y = generate_position(positions)
        color_name = random.choice(list(colors.keys()))
        box_type = random.choice(list(box_types.keys()))
        ld.add_action(spawn_stackable_box(
            f'{color_name}_{box_type}_box_{i+1}', 
            box_type, color_name, x, y, 0.1, 0.1
        ))


    # Create some intentional stacks (2-4 boxes high)
    num_stacks = random.randint(3, 6)
    box_counter = num_scattered_boxes + 1
    
    for stack_i in range(num_stacks):
        stack_x, stack_y = generate_stack_position(stack_positions)
        stack_height = random.randint(2, 4)
        
        # Create stack from bottom to top
        current_z = 0.05  # Start slightly above ground
        
        for level in range(stack_height):
            color_name = random.choice(list(colors.keys()))
            box_type = random.choice(['standard', 'small', 'flat'])  # Better for stacking
            
            # Add slight random offset for more realistic stacking
            offset_x = random.uniform(-0.02, 0.02)
            offset_y = random.uniform(-0.02, 0.02)
            
            ld.add_action(spawn_stackable_box(
                f'stack_{stack_i}_{color_name}_{box_type}_box_{box_counter}',
                box_type, color_name, 
                stack_x + offset_x, stack_y + offset_y, current_z,
                3.0 + stack_i*1.0 + level*0.2  # Staggered timing for proper stacking
            ))
            
            # Update height for next level based on box type
            if box_type == 'flat':
                current_z += 0.1
            elif box_type == 'small':
                current_z += 0.15
            else:  # standard
                current_z += 0.2
                
            box_counter += 1

    return ld
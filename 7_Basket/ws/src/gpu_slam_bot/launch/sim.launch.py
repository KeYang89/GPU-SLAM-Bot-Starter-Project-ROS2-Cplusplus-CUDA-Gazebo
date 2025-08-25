import os
import random
import math
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('gpu_slam_bot')

    # Declare launch argument for world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'maze.world'),
        description='Path to the Gazebo world file'
    )

    # Process Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'gpu_slam_bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Gazebo launch directory
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # Launch description
    ld = LaunchDescription()

    # Add world argument
    ld.add_action(world_arg)

    # Launch Gazebo with specified world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            launch_arguments={'verbose': 'true', 'world': LaunchConfiguration('world')}.items()
        )
    )

    # Robot state publisher
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description['robot_description']}]
        )
    )

    # Spawn the robot
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'gpu_slam_bot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen'
        )
    )

    # Controller manager for ros2_control
    ld.add_action(
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, os.path.join(pkg_share, 'config', 'controllers.yaml')],
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

    # Start auto_lifter node
    ld.add_action(
        Node(
            package='gpu_slam_bot',
            executable='auto_lifter',
            name='auto_lifter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    )

    # Start box detector with better parameters for larger boxes
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gpu_slam_bot',
                    executable='box_detector',
                    name='box_detector',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'min_detection_range': 0.05},
                        {'max_detection_range': 0.4},  # Increased range
                        {'min_cluster_size': 6},  # Increased for larger boxes
                        {'max_angular_gap': 0.08},  # Smaller gap for better clustering
                        {'min_box_angular_width': 0.03},  # Minimum angular width
                        {'max_box_angular_width': 0.25}   # Maximum angular width
                    ]
                )
            ]
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
                        {'max_cycle_time': 4.0},  # Longer cycle for stability
                        {'gripper_open_angle': 0.5},
                        {'gripper_closed_angle': -0.1},
                        {'gripper_push_angle': -0.3},
                        {'lifter_down': 0.1},
                        {'lifter_up': 0.5},  # Higher lift
                        {'lifter_push_height': 0.15},
                        {'wall_detection_distance': 0.3},
                        {'obstacle_push_distance': 0.2}  # Closer approach
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

    # Box types with larger sizes
    box_types = {
        'standard': {'size': '0.1 0.1 0.1', 'mass': 0.05, 'friction': 0.8},    
        'small': {'size': '0.06 0.06 0.06', 'mass': 0.01, 'friction': 0.7},   
        'flat': {'size': '0.07 0.1 0.08', 'mass': 0.04, 'friction': 0.75},    
        'tall': {'size': '0.07 0.07 0.12', 'mass': 0.05, 'friction': 0.7}     
    }

    # Define safe spawn area within maze walls
    safe_spawn_range_x = (-9, 9)
    safe_spawn_range_y = (-9, 9)
    wall_buffer = 0.5  # Buffer from walls

    positions = []
    stack_positions = []
    
    # Keep track of generated box names for the gripper controller
    generated_box_names = []

    def is_within_maze_bounds(x, y):
        """Check if the position is within the maze boundaries"""
        return (safe_spawn_range_x[0] + wall_buffer <= x <= safe_spawn_range_x[1] - wall_buffer and
                safe_spawn_range_y[0] + wall_buffer <= y <= safe_spawn_range_y[1] - wall_buffer)

    def generate_position(existing, min_dist=0.5):
        """Generate a position within maze boundaries and away from other boxes"""
        while True:
            x = round(random.uniform(*safe_spawn_range_x), 2)
            y = round(random.uniform(*safe_spawn_range_y), 2)
            if (is_within_maze_bounds(x, y) and
                all(math.hypot(x - ex, y - ey) >= min_dist for ex, ey in existing)):
                existing.append((x, y))
                return x, y

    def generate_stack_position(existing, min_dist=1.0):
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
        
        size_parts = size.split()
        sx, sy, sz = float(size_parts[0]), float(size_parts[1]), float(size_parts[2])
        
        ixx = (mass / 12.0) * (sy*sy + sz*sz)
        iyy = (mass / 12.0) * (sx*sx + sz*sz)
        izz = (mass / 12.0) * (sx*sx + sy*sy)
        
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
          <ixx>{ixx:.6f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{iyy:.6f}</iyy>
          <iyz>0</iyz>
          <izz>{izz:.6f}</izz>
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

        # Store the box name for potential teleportation
        generated_box_names.append(entity_name)

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
    num_scattered_boxes = random.randint(12, 18)  # Reasonable number for testing
    for i in range(num_scattered_boxes):
        x, y = generate_position(positions)
        color_name = random.choice(list(colors.keys()))
        box_type = random.choice(list(box_types.keys()))
        
        # Create consistent naming pattern
        entity_name = f'{color_name}_{box_type}_box_{i+1}'
        
        ld.add_action(spawn_stackable_box(
            entity_name, 
            box_type, color_name, x, y, 0.06, 0.1 + i * 0.05  # Staggered spawn times
        ))

    # Create intentional stacks
    num_stacks = random.randint(2, 4)  # Fewer stacks for stability
    box_counter = num_scattered_boxes + 1
        
    for stack_i in range(num_stacks):
        stack_x, stack_y = generate_stack_position(stack_positions)
        stack_height = random.randint(2, 3)  # Conservative stack height
            
        current_z = 0.06  # Start height for stacks
            
        for level in range(stack_height):
            color_name = random.choice(list(colors.keys()))
            box_type = random.choice(['standard', 'small', 'flat'])
                
            offset_x = random.uniform(-0.005, 0.005)  # Small offsets for stability
            offset_y = random.uniform(-0.005, 0.005)
            
            entity_name = f'stack_{stack_i}_{color_name}_{box_type}_box_{box_counter}'
                
            ld.add_action(spawn_stackable_box(
                entity_name,
                box_type, color_name, 
                stack_x + offset_x, stack_y + offset_y, current_z,
                4.0 + stack_i * 1.0 + level * 0.3  # More spacing between stack spawns
            ))
                
            # Update z position for next level
            if box_type == 'flat':
                current_z += 0.08  
            elif box_type == 'small':
                current_z += 0.08  
            else:  # standard
                current_z += 0.10  
                    
            box_counter += 1

    # Write box names to a file for the gripper controller to reference
    box_names_file = os.path.join('/tmp', 'spawned_box_names.txt')
    try:
        with open(box_names_file, 'w') as f:
            for name in generated_box_names:
                f.write(name + '\n')
    except Exception as e:
        print(f"Warning: Could not write box names file: {e}")

    print(f"Generated {len(generated_box_names)} boxes for simulation")
    return ld
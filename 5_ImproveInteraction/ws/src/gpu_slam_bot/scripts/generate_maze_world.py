#!/usr/bin/env python3
import random
import os

WORLD_FILE = "src/gpu_slam_bot/worlds/maze_generated.world"

# Maze boundaries
min_x, max_x = -5.9, 5.9
min_y, max_y = -5.9, 5.9

# Avoid spawning inside walls (x_min, x_max, y_min, y_max)
excluded_zones = [
    (-5.9, -1, 1, 5.9),  # example for a wall area
    (1, 5.9, -5.9, -1)
]

colors = [
    (0.76, 0.56, 0.56),
    (0.64, 0.69, 0.62),
    (0.60, 0.69, 0.77),
    (0.78, 0.74, 0.66),
    (0.72, 0.68, 0.75),
    (0.60, 0.60, 0.54)
]

def random_box_pose():
    while True:
        x = round(random.uniform(min_x, max_x), 2)
        y = round(random.uniform(min_y, max_y), 2)
        z = 0.05
        # Check against excluded zones
        inside_zone = any(x_min <= x <= x_max and y_min <= y <= y_max for x_min, x_max, y_min, y_max in excluded_zones)
        if not inside_zone:
            return x, y, z

def random_box_color():
    r, g, b = random.choice(colors)
    return f"{r} {g} {b} 1", f"{r} {g} {b} 1"

def generate_boxes(n=100):
    models = []
    for i in range(n):
        x, y, z = random_box_pose()
        ambient, diffuse = random_box_color()
        size = round(random.uniform(0.15, 0.25), 2)
        model = f"""
  <model name="box_{i+1}">
    <static>false</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.2</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse></material>
      </visual>
    </link>
  </model>
"""
        models.append(model)
    return "\n".join(models)

# World template with boxes
world_template = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="small_maze_world">
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Outer walls -->
    <model name="wall_north">
      <static>true</static><pose>0 6 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>12 0.1 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>12 0.1 0.5</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_south">
      <static>true</static><pose>0 -6 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>12 0.1 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>12 0.1 0.5</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_east">
      <static>true</static><pose>6 0 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 12 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 12 0.5</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_west">
      <static>true</static><pose>-6 0 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 12 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 12 0.5</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>

    <!-- Interior walls -->
    <model name="maze_wall_1">
      <static>true</static><pose>-3 2 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>4 0.1 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>4 0.1 0.5</size></box></geometry><material><ambient>0.4 0.4 0.8 1</ambient></material></visual>
      </link>
    </model>

    <model name="maze_wall_2">
      <static>true</static><pose>2 -5 0.25 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>4 0.1 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>4 0.1 0.5</size></box></geometry><material><ambient>0.8 0.4 0.4 1</ambient></material></visual>
      </link>
    </model>

    <model name="maze_wall_3">
      <static>true</static><pose>3 4 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>3 0.1 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>3 0.1 0.5</size></box></geometry><material><ambient>0.4 0.8 0.4 1</ambient></material></visual>
      </link>
    </model>

    <!-- Random boxes -->
{generate_boxes(450)}

  </world>
</sdf>
"""

# Write world to file
os.makedirs(os.path.dirname(WORLD_FILE), exist_ok=True)
with open(WORLD_FILE, "w") as f:
    f.write(world_template)

print(f"World file generated: {WORLD_FILE}")

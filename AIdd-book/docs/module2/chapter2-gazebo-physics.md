---
id: chapter2-gazebo-physics
title: 'Advanced Simulation in Gazebo: Physics and Environments'
sidebar_position: 2
---

## Learning Objectives

-   Understand the structure of the Simulation Description Format (SDF) and its key elements.
-   Model realistic rigid body dynamics, including mass and inertia.
-   Define detailed collision properties and surface friction for realistic interactions.
-   Build a custom Gazebo world with lighting, physics properties, and multiple static and dynamic models.
-   Create a complete ROS 2 package to launch and manage a Gazebo simulation.

## Key Concepts

-   **SDF (Simulation Description Format)**: An XML-based format that describes everything in a simulation, from the robot's physical properties to the environment's lighting. It is the native format for Gazebo.
-   **`<link>`**: A fundamental building block in SDF, representing a single rigid body of the robot with physical properties.
-   **`<inertial>`**: An element within a link that defines its dynamic properties: mass and the inertia tensor.
-   **`<collision>`**: An element that defines the geometry of a link for the purpose of physics calculations. This is what the physics engine "sees."
-   **`<visual>`**: An element that defines the geometry and appearance of a link for rendering. This is what the user sees. It can be simpler or more complex than the collision geometry.
-   **`<joint>`**: An element that connects two links and defines their relative motion (e.g., `revolute` for a hinge, `prismatic` for a slider).
-   **Gazebo Plugins**: Shared libraries that can be loaded at runtime to extend the functionality of Gazebo. There are plugins for sensors, actuators, and ROS 2 communication.

## Deep Theoretical Explanation

### The `<collision>` and `<visual>` Distinction

A crucial concept in simulation is the separation of collision geometry from visual geometry.
-   **`<collision>`**: This geometry should be as simple as possible (e.g., spheres, cylinders, boxes) while still accurately representing the robot's shape for physics interactions. Complex collision meshes are computationally expensive and can slow down the simulation.
-   **`<visual>`**: This geometry can be a highly detailed mesh (e.g., from a CAD model) to provide a photo-realistic appearance without affecting the physics performance.

By keeping them separate, you can have a simulation that both looks good and runs fast.

### Modeling Friction

Friction in Gazebo is defined within the `<surface>` element of a `<collision>` tag. The most common model is the "cone of friction" model, which uses two primary coefficients:
-   **`mu` (μ)**: The coefficient of static friction. It defines the force required to initiate motion between two surfaces.
-   **`mu2` (μ2)**: The second coefficient of friction, used for anisotropic friction (where friction differs by direction).

You can also set parameters like `slip1` and `slip2` to model wheel slippage, which is critical for realistic ground vehicle simulations.

```xml
<collision name="collision">
  <geometry>
    <box size="0.5 0.5 0.1"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Practical Tasks

### 1. Create a Gazebo Package

Create a dedicated ROS 2 package to hold your simulation files.

```bash
cd ~/ros2_ws/src
ros2 pkg create my_gazebo_package --build-type ament_python --dependencies rclpy launch_ros
```

### 2. Create `worlds`, `models`, and `launch` directories

Inside your new package, create directories to store your world files, robot models, and launch files.

```bash
cd my_gazebo_package
mkdir worlds models launch
```

### 3. Create the World and Robot SDF files

Place the `my_world.sdf` and `robot.sdf` files (from the templates below) into the `worlds` and `models` directories, respectively.

### 4. Create the Launch File

Place the `spawn_robot.launch.py` file into the `launch` directory. You will need to modify the `get_package_share_directory` call to use your new package name, `my_gazebo_package`.

### 5. Build and Launch

1.  Build your workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
2.  Launch the simulation:
    ```bash
    ros2 launch my_gazebo_package spawn_robot.launch.py
    ```

## Diagrams

*   **SDF File Structure**: A tree diagram showing the nested structure of an SDF file, starting from the root `<sdf>` tag, branching to `<world>` and `<model>`, and then further down to `<link>`, `<joint>`, `<collision>`, `<visual>`, and `<inertial>`.
*   **Gazebo-ROS 2 Communication**: A diagram illustrating how Gazebo communicates with ROS 2. It shows the `gazebo_ros` plugins (like `libgazebo_ros_init.so` and `libgazebo_ros_factory.so`) acting as a bridge, converting ROS 2 messages into Gazebo API calls and vice-versa.

## Code Templates

### Detailed Robot SDF with Friction

```xml
<!-- examples/module2/chapter2-gazebo-physics/robot.sdf -->
<sdf version="1.6">
  <model name="my_robot">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.083</iyy>
          <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

### World SDF with Custom Physics

```xml
<!-- examples/module2/chapter2-gazebo-physics/my_world.sdf -->
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

### ROS 2 Launch file for Gazebo

```python
# examples/module2/chapter2-gazebo-physics/spawn_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_gazebo_package') # <-- Change to your package name
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    robot_file = os.path.join(pkg_share, 'models', 'robot.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', robot_file],
            output='screen'
        )
    ])
```
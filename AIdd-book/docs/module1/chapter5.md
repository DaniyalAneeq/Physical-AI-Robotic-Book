---
id: chapter5
title: Robot Description and Gazebo Simulation
sidebar_position: 5
---

## Learning Objectives

*   Understand how to describe a robot's physical properties using URDF.
*   Learn how to simulate a robot in the Gazebo 3D simulator.
*   Integrate a URDF model with Gazebo for realistic simulation.

## Key Concepts

*   **URDF (Unified Robot Description Format)**: An XML file format for modeling a robot's kinematics, dynamics, and visual appearance.
*   **Gazebo**: A powerful 3D robotics simulator that allows you to test and validate robot designs and algorithms.
*   **SDF (Simulation Description Format)**: An XML format used by Gazebo to describe everything in a simulation environment, from robots to lighting.
*   **`ros2_control`**: A framework for real-time control of robots, which can also be used in simulation.
*   **`gazebo_ros_pkgs`**: A set of ROS 2 packages for integrating ROS 2 with Gazebo.

## Practical Tasks

1.  **Create a Simple URDF**: Write a URDF file for a simple two-wheeled robot.
2.  **Launch in Gazebo**: Create a ROS 2 launch file to spawn the URDF model in an empty Gazebo world.
3.  **Add Gazebo Plugins**: Extend the URDF with Gazebo-specific tags to add sensors (e.g., a camera) and a differential drive controller.
4.  **Control the Robot**: Publish velocity commands to the differential drive controller to move the robot in Gazebo.

## Diagrams

*   **URDF Structure**: A diagram showing the hierarchical structure of a URDF file (links, joints, visuals, collisions).
*   **Gazebo Simulation Loop**: A diagram illustrating the interaction between ROS 2, Gazebo, and the simulated robot.

## Code Templates

### Simple URDF File (`.urdf`)

```xml
<!-- examples/module1/chapter5-urdf-gazebo/simple_robot.urdf -->
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  <!-- Add more links and joints -->
</robot>
```

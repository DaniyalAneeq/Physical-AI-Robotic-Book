---
id: chapter4-sensor-simulation
title: Advanced Sensor Simulation
sidebar_position: 4
---

## Learning Objectives

-   Configure and simulate a 2D/3D LiDAR sensor in Gazebo and Unity.
-   Configure and simulate an RGB-D (depth) camera.
-   Model an Inertial Measurement Unit (IMU) with noise.

## Key Concepts

-   Sensor Modeling
-   Noise Models
-   Ray Tracing (for LiDAR)
-   Point Clouds
-   ROS 2 Bridge

## Code Examples

### SDF/URDF snippets for adding LiDAR, depth camera, and IMU sensors to a robot model

```xml
<!-- examples/module2/chapter4-sensor-simulation/sensors.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="lidar_sensor" params="parent">
    <joint name="lidar_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="lidar_link"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </joint>

    <link name="lidar_link">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.04"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.04"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <gazebo reference="lidar_link">
      <sensor type="ray" name="lidar">
        <update_rate>10</update_rate>
        <visualize>true</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14</min_angle>
              <max_angle>3.14</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/my_robot</namespace>
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>

</robot>
```

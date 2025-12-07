---
id: chapter5-humanoid-digital-twin
title: Building a Full Humanoid Digital Twin
sidebar_position: 5
---

## Learning Objectives

-   Combine Gazebo and Unity in a multi-simulator workflow.
-   Optimize simulation performance for a complex humanoid robot.
-   Implement a full ROS 2 control loop for the humanoid Digital Twin.

## Key Concepts

-   Multi-simulator Workflow
-   Performance Optimization
-   End-to-End Testing

## Code Examples

### A complete URDF for a simple humanoid robot

```xml
<!-- examples/module2/chapter5-humanoid-digital-twin/humanoid.urdf -->
<robot name="humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

### A ROS 2 package for controlling the humanoid's joints

```python
# examples/module2/chapter5-humanoid-digital-twin/humanoid_control/humanoid_control/joint_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['left_shoulder_joint', 'right_shoulder_joint']
        self.joint_state.position = [0.0, 0.0]

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

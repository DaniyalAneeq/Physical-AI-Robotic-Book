---
id: chapter4
title: 'Parameters and TF2: Configuration and Coordinate Frames'
sidebar_position: 4
---

## Learning Objectives

*   Understand how to use ROS 2 parameters for dynamic, run-time configuration of nodes.
*   Learn to load parameters from YAML files for organized and reusable configurations.
*   Master the TF2 (Transformations) library for managing and transforming data between different coordinate frames.
*   Implement TF2 broadcasters and listeners in Python to build a complete robot coordinate system.
*   Visualize coordinate frames and their relationships using RViz2.

## Key Concepts

*   **Parameters**: Key-value pairs associated with a node that can be modified at runtime without recompiling code. They are essential for tuning and configuring robot behavior. ROS 2 supports various parameter types, including strings, integers, floats, booleans, and arrays.
*   **Parameter Server**: Each ROS 2 node has its own internal parameter server. The `ros2 param` command-line tool communicates with these individual servers to get, set, list, or dump parameters.
*   **TF2**: The official transformation library in ROS 2. It manages the complex web of coordinate frames (e.g., `world`, `base_link`, `camera_link`, `gripper_link`) in a robotic system, allowing you to ask questions like, "What is the position of the apple (in the `camera_link` frame) relative to the robot's hand (in the `gripper_link` frame)?"
*   **`tf2_ros`**: The ROS 2 package that provides the necessary tools to work with TF2, including the transform listener and broadcaster.
*   **Static Transform**: A transform between two coordinate frames that never changes (e.g., the mounting position of a camera on a robot's chassis). These are published once and are assumed to be valid indefinitely.
*   **Dynamic Transform**: A transform that changes over time (e.g., the relationship between a moving robot's base and a fixed world frame). These must be published continuously.
*   **Transform Tree**: The entire collection of coordinate frames and the transforms that connect them, forming a tree-like structure.

## Deep Theoretical Explanation

### How TF2 Works

TF2 operates on a simple but powerful principle: it builds a **tree** of coordinate frames. Each transform defines a parent-child relationship. For example, publishing a transform from `world` (parent) to `base_link` (child) tells TF2 how to get from the world frame to the robot's base frame.

The core components of TF2 are:

*   **`tf2_ros.Buffer`**: The buffer stores all the transforms it receives from broadcasters for a configurable amount of time. It is the central database of coordinate frame information.
*   **`tf2_ros.TransformListener`**: The listener subscribes to the `/tf` and `/tf_static` topics, collects all broadcasted transforms, and populates the buffer.
*   **`tf2_ros.TransformBroadcaster` / `StaticTransformBroadcaster`**: These are used to publish dynamic and static transforms, respectively.

When you request a transform between two arbitrary frames (e.g., `camera_link` to `gripper_link`), the TF2 buffer traverses the transform tree to find a path between them. It then concatenates the transforms along this path to compute the final transformation. This allows for a completely decentralized system where different nodes can be responsible for publishing different parts of the transform tree.

## Practical Tasks

### 1. Using Parameters and YAML Files

1.  **Create a Parameter YAML file**: Create a file named `my_params.yaml` in a `config` directory within your package (`my_first_package/config/my_params.yaml`):
    ```yaml
    parameter_node:
      ros__parameters:
        my_parameter: "world"
        frequency: 2.0
    ```
2.  **Create a Launch File**: Create a launch file (`my_first_package/launch/params_launch.py`) to start the node and load the parameters:
    ```python
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node
    import os

    def generate_launch_description():
        config = os.path.join(
            get_package_share_directory('my_first_package'),
            'config',
            'my_params.yaml'
            )

        return LaunchDescription([
            Node(
                package='my_first_package',
                executable='parameter_node', # Assumes entry point is named 'parameter_node'
                name='parameter_node',
                parameters=[config]
            )
        ])
    ```
3.  **Run the Launch File**:
    ```bash
    ros2 launch my_first_package params_launch.py
    ```
    You will see the node start with the values from your YAML file.

### 2. Implementing a Full TF2 System

1.  **Implement a Dynamic TF2 Broadcaster**: Create a Python node that publishes a continuously changing transform, for example, a robot base moving in a circle.
2.  **Implement a Static TF2 Broadcaster**: Create a node that publishes the fixed transform from the robot's `base_link` to a `camera_link`.
3.  **Implement a TF2 Listener**: Create a node that listens to the transform tree and calculates the transform from the `camera_link` to a dynamic frame.
4.  **Visualize with RViz2**: Use RViz2 to add a TF display and visualize the entire coordinate frame tree in real-time.

## Diagrams

*   **Parameter Server Interaction**: A diagram showing a launch file loading a YAML file. The parameters are passed to a newly created node, which then populates its internal Parameter Server. The `ros2 param` tool is shown interacting with the node's server to inspect and change values at runtime.
*   **TF2 Tree Example**: A diagram illustrating a typical robot transform tree. It would show `world` as the root, with a child `odom`. `odom` would be the parent of `base_link`. `base_link` would have multiple children, such as `camera_link`, `lidar_link`, and `left_wheel_link`, visually representing the robot's physical structure.

## Code Templates

*(These templates provide a starting point. You would integrate them into your `my_first_package`.)*

### Python `rclpy` Parameter Usage Example

```python
# examples/module1/chapter4-params-tf2/parameter_node.py
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('frequency', 1.0)
        self.get_logger().info('ParameterNode started.')

        # Create a timer that uses one of the parameters
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

    def timer_callback(self):
        my_parameter = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'My parameter is: {my_parameter}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python `rclpy` Static TF2 Broadcaster Example

```python
# examples/module1/chapter4-params-tf2/static_tf2_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'base_link'
        static_transformStamped.child_frame_id = 'camera_link'
        static_transformStamped.transform.translation.x = 0.1
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.2
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---
id: chapter2
title: ROS 2 Nodes, Topics, and Messages
sidebar_position: 2
---

## Learning Objectives

*   Deepen understanding of ROS 2 nodes and their lifecycle.
*   Master the publish/subscribe communication pattern using topics and messages.
*   Define custom message types for structured data communication.
*   Create a complete ROS 2 package from scratch, including building and running it.
*   Implement simple ROS 2 publishers and subscribers in Python using `rclpy`.

## Key Concepts

*   **Nodes**: The fundamental processing units in a ROS 2 system. Each node should have a single, well-defined purpose (e.g., controlling a wheel, processing camera images). A complex system is built by composing many simple nodes.
*   **Topics**: The primary communication channel for publish/subscribe messaging. Topics are strongly typed; a topic can only transmit messages of a single, defined message type. This ensures type safety at the communication layer.
*   **Messages**: Structured data containers defined in `.msg` files. They consist of a set of typed fields. ROS 2 provides a vast library of standard messages, but defining custom messages is essential for creating tailored applications.
*   **Packages**: The primary unit of organization for ROS 2 code. A package contains everything related to a specific piece of functionality, including nodes, message definitions, launch files, and configuration files.
*   **`rclpy`**: The Python client library for ROS 2. It provides the high-level functionality for creating nodes, publishers, subscribers, and other ROS 2 entities in Python.
*   **`colcon`**: The standard build tool for ROS 2. It is used to build and install ROS 2 workspaces and packages.

## Deep Theoretical Explanation

### Node Lifecycle

In ROS 2, nodes can exist in a more controlled state machine called the **Lifecycle**. This is particularly important for building robust, fault-tolerant systems. A lifecycle node has the following primary states:

*   **Unconfigured**: The initial state. The node has been created but has not allocated any resources.
*   **Configuring**: A transitional state where the node is allocating resources and setting up its internal state.
*   **Inactive**: The node is configured but not currently active. It is not processing data or responding to requests.
*   **Activating**: A transitional state where the node is preparing to become active.
*   **Active**: The node is fully operational and performing its tasks.
*   **Deactivating**: A transitional state where the node is stopping its active processes.
*   **Cleaning up**: A transitional state where the node is releasing resources.
*   **Finalized**: The terminal state.

This state machine allows a system supervisor to manage the startup, shutdown, and recovery of nodes in a predictable way.

### Quality of Service (QoS)

ROS 2's use of DDS enables fine-grained control over the **Quality of Service (QoS)** of communication. QoS policies determine how messages are handled by the middleware. Some of the most important QoS settings include:

*   **History**:
    *   `KEEP_LAST`: Store up to a specified number of recent messages.
    *   `KEEP_ALL`: Store all messages, up to the resource limits of the system.
*   **Depth**: Used with `KEEP_LAST` to specify how many messages to keep.
*   **Reliability**:
    *   `RELIABLE`: Guarantees delivery. Important for critical data like commands.
    *   `BEST_EFFORT`: Faster, but may drop messages. Suitable for high-frequency sensor data where a single missed message is not critical.
*   **Durability**:
    *   `VOLATILE`: Messages are not persisted.
    *   `TRANSIENT_LOCAL`: Publishers store a configurable number of recent messages, which are delivered to any late-joining subscribers. This is extremely useful for topics that publish configuration or status information.

## Practical Tasks

### 1. Create a New ROS 2 Package

Let's create a package named `my_first_package` to house our custom message and nodes.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_package --dependencies rclpy std_msgs
```

### 2. Create a Custom Message

1.  Create a `msg` directory inside your new package:
    ```bash
    mkdir -p ~/ros2_ws/src/my_first_package/my_first_package/msg
    ```
2.  Create a new file named `MyCustomMessage.msg` in that directory with the following content:
    ```
    # ~/ros2_ws/src/my_first_package/my_first_package/msg/MyCustomMessage.msg
    std_msgs/Header header
    string data
    float32 temperature
    ```
3.  Modify `~/ros2_ws/src/my_first_package/package.xml` to add the dependencies needed to build the message:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
4.  Modify `~/ros2_ws/src/my_first_package/setup.py` to ensure the `msg` directory is found by `colcon`.

### 3. Build the Package

Navigate to the root of your workspace and build:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. Implement the Talker and Listener Nodes

Place the Python code for `my_talker.py` and `my_listener.py` (from the "Code Templates" section below) into the `~/ros2_ws/src/my_first_package/my_first_package/` directory. Remember to update the code to import and use your new `MyCustomMessage` type.

Modify `setup.py` to create entry points for your nodes, so `ros2 run` can find them.

### 5. Run and Verify

1.  Build the workspace again:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
2.  Run the listener:
    ```bash
    ros2 run my_first_package my_listener
    ```
3.  In a new terminal, run the talker:
    ```bash
    ros2 run my_first_package my_talker
    ```
4.  Verify communication with `ros2 topic echo`:
    ```bash
    ros2 topic echo /my_topic
    ```

## Diagrams

*   **Publisher-Subscriber Communication Flow**: This diagram will show two nodes, a "Talker" and a "Listener". An arrow will originate from the Talker, point to a box representing the topic (`/my_topic`), and another arrow will point from the topic to the Listener. This visually represents the unidirectional flow of data.
*   **ROS 2 Package Structure**: A tree diagram showing the standard layout of a ROS 2 Python package, including `package.xml`, `setup.py`, the node source files, and the `msg` directory.

## Code Templates

*Note: You will need to modify these to use `my_first_package.msg.MyCustomMessage` instead of `std_msgs.msg.String` and populate the custom fields.*

### Python `rclpy` Talker Node (Publisher)

```python
# examples/module1/chapter2-nodes-topics/my_talker.py
# (This code would be placed in ~/ros2_ws/src/my_first_package/my_first_package/my_talker.py)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Replace with your custom message

class MyTalker(Node):
    def __init__(self):
        super().__init__('my_talker')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.get_logger().info('MyTalker node started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from MyTalker! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python `rclpy` Listener Node (Subscriber)

```python
# examples/module1/chapter2-nodes-topics/my_listener.py
# (This code would be placed in ~/ros2_ws/src/my_first_package/my_first_package/my_listener.py)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Replace with your custom message

class MyListener(Node):
    def __init__(self):
        super().__init__('my_listener')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.get_logger().info('MyListener node started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

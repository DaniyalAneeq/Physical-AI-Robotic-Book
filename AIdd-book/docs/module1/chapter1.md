---
id: chapter1
title: Introduction to ROS 2
sidebar_position: 1
---

## Learning Objectives

*   Understand the core architecture of ROS 2, including the role of the DDS middleware.
*   Learn how to install ROS 2 on a supported Linux distribution.
*   Create and manage a ROS 2 workspace for developing robot applications.
*   Get hands-on experience with fundamental ROS 2 concepts by running a simple simulation.

## Key Concepts

*   **Nodes**: A node is a process that performs computation. In ROS 2, a single executable can contain one or more nodes. They are the fundamental building blocks of a ROS 2 system, responsible for specific tasks like controlling a motor, reading sensor data, or planning a path.
*   **Topics**: Topics are named buses over which nodes exchange messages. They are the primary mechanism for asynchronous, publish/subscribe communication. A node can publish messages to a topic, and any number of nodes can subscribe to that topic to receive the messages.
*   **Messages**: Messages are the data structures used for communication over topics. ROS 2 provides a rich set of standard message types (e.g., `std_msgs`, `sensor_msgs`, `geometry_msgs`), and you can also define your own custom message types in `.msg` files.
*   **Services**: Services provide a synchronous, request/reply communication pattern. A client node sends a request to a server node and waits for a response. This is useful for tasks that have a clear beginning and end, like "get the robot's current position."
*   **Actions**: Actions are used for long-running, asynchronous tasks that provide feedback. An action client sends a goal to an action server (e.g., "move to a specific location"). The server executes the goal, providing periodic feedback (e.g., "distance to goal"), and eventually returns a result.
*   **ROS Graph**: The ROS Graph is the network of all ROS 2 elements (nodes, topics, services, actions) running in a system at a given time. Understanding the graph is crucial for debugging and introspection.

## Deep Theoretical Explanation

### The ROS 2 Architecture and DDS

ROS 2 represents a significant architectural evolution from ROS 1. One of the most fundamental changes is the adoption of the **Data Distribution Service (DDS)** as its underlying middleware. DDS is an industry standard for real-time, scalable, and reliable data exchange.

By using DDS, ROS 2 gains several key advantages:

*   **Improved Performance**: DDS provides a high-performance, low-latency communication layer.
*   **Enhanced Reliability**: It offers configurable Quality of Service (QoS) policies for data delivery, such as "reliable" (guaranteed delivery) and "best effort" (faster, but with potential for message loss).
*   **Better Scalability**: DDS is designed for large, distributed systems, making ROS 2 suitable for complex multi-robot applications.
*   **Interoperability**: Different DDS implementations from various vendors can interoperate, providing flexibility.

The ROS 2 stack is layered, with the user-facing ROS 2 client libraries (`rclpy` for Python, `rclcpp` for C++) at the top. These libraries provide the familiar ROS concepts (nodes, topics, etc.). Beneath them, the `rmw` (ROS Middleware) API provides an abstraction layer, and the `rmw` implementation for a specific DDS vendor sits below that.

## Practical Tasks

### 1. ROS 2 Installation

We will be using **ROS 2 Humble Hawksbill**, which is the latest long-term support (LTS) release. It is officially supported on Ubuntu 22.04 (Jammy Jellyfish).

1.  **Set Locale**:
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 APT Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Desktop**:
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```
4.  **Source the Setup File**:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### 2. Workspace Creation

A ROS 2 workspace is a directory where you can create, modify, and build your own ROS 2 packages.

1.  **Create a Workspace Directory**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Build the Workspace**:
    ```bash
    colcon build
    ```
    This command will build any packages found in the `src` directory. Since it's empty, it will just create the `build`, `install`, and `log` directories.
3.  **Source the Workspace**:
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### 3. Running `turtlesim`

`turtlesim` is a classic ROS demo that provides a simple and fun way to visualize basic ROS 2 concepts.

1.  **Run the `turtlesim` Node**:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    This will open a window with a turtle in the center.

2.  **Run the `turtle_teleop_key` Node**:
    In a new terminal, run:
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
    You can now use the arrow keys on your keyboard to move the turtle in the `turtlesim` window.

3.  **Introspecting the ROS Graph**:
    Use the `ros2` command-line tool to inspect the running system:
    ```bash
    ros2 node list
    ros2 topic list
    ros2 topic echo /turtle1/cmd_vel
    ```

## Diagrams

*   **ROS 2 Architecture Overview**: This diagram will show the layered architecture of ROS 2, from the user's code at the top, through the `rcl` client libraries, the `rmw` middleware interface, and down to the specific DDS implementation at the bottom. It will also illustrate how different DDS vendors can be used.
*   **ROS Graph Illustration**: This diagram will visually represent the `turtlesim` example, showing the `/turtlesim` and `/teleop_turtle` nodes, and the `/turtle1/cmd_vel` topic connecting them.

## Code Templates

### Basic `rclpy` Publisher Example

This example demonstrates how to create a simple node that publishes a `String` message to a topic every 0.5 seconds.

```python
# examples/module1/chapter1-ros2-basics/simple_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic `rclpy` Subscriber Example

This example shows how to create a node that subscribes to the `topic` and prints the received messages to the console.

```python
# examples/module1/chapter1-ros2-basics/simple_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

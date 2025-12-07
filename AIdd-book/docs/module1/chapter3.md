---
id: chapter3
title: 'ROS 2 Services and Actions: Request/Reply and Long-Running Tasks'
sidebar_position: 3
---

## Learning Objectives

*   Understand the synchronous request/reply communication pattern of ROS 2 Services and when to use it.
*   Define custom service types and implement Service Servers and Clients in Python.
*   Understand the asynchronous, goal-based communication pattern of ROS 2 Actions and its advantages for long-running tasks.
*   Define custom action types and implement Action Servers and Clients in Python.

## Key Concepts

*   **Services**: A strictly synchronous, one-to-one communication pattern. When a client calls a service, it **blocks** (waits) until the server completes the request and returns a single response. This is ideal for quick, atomic operations like querying a sensor value or retrieving a configuration parameter.
*   **Actions**: An asynchronous communication pattern designed for long-running, goal-oriented tasks that may take a significant amount of time to complete. Unlike services, actions are non-blocking. They provide continuous feedback on their progress and are cancellable. This is perfect for tasks like navigation, manipulation, or complex computations.
*   **Service Definition (`.srv` file)**: A file that defines the structure of a service's request and response. The request and response sections are separated by a `---`.
*   **Action Definition (`.action` file)**: A file that defines the structure for an action. It is composed of three parts, separated by `---`:
    1.  **Goal**: The request sent by the client to initiate the action.
    2.  **Result**: The final outcome sent by the server upon completion.
    3.  **Feedback**: Intermediate updates sent by the server while the goal is being executed.

## Deep Theoretical Explanation

### Services vs. Actions: Choosing the Right Tool

| Feature         | Services                               | Actions                                                         |
| --------------- | -------------------------------------- | --------------------------------------------------------------- |
| **Paradigm**    | Synchronous Request/Response           | Asynchronous Goal/Feedback/Result                               |
| **Execution**   | Blocking (Client waits for response)   | Non-blocking (Client can perform other tasks)                   |
| **Feedback**    | None                                   | Continuous feedback during execution                            |
| **Preemption**  | No (Cannot be cancelled)               | Yes (Client can request to cancel the goal)                     |
- **Use Case**    | Quick, atomic queries or commands      | Long-running, stateful tasks (navigation, manipulation)         |

### Anatomy of a ROS 2 Action

An action is more complex than a service and involves several components on both the client and server side.

*   **Action Server**:
    *   **Goal Callback**: A function that is called when a new goal is received from a client. The server can choose to `ACCEPT` or `REJECT` the goal.
    - **Handle Accepted Callback**: This callback is triggered after a goal has been accepted. It's where you typically start the execution of the action, often by spawning a new thread to avoid blocking the main ROS event loop.
    *   **Execute Callback**: The main function where the long-running task is performed. It should periodically check if a cancellation has been requested and publish feedback.
    *   **Cancel Callback**: A function that is called when a client requests to cancel the current goal. The server can `ACCEPT` or `REJECT` the cancellation request.

*   **Action Client**:
    *   **Send Goal**: The client initiates the action by sending a goal message to the server. This is an asynchronous call.
    *   **Goal Response Callback**: This function is called when the server accepts or rejects the goal.
    *   **Feedback Callback**: This function is called whenever the server publishes feedback for the action.
    *   **Result Callback**: This function is called when the action completes and the server sends the final result.

## Practical Tasks

These tasks should be performed within the `my_first_package` you created in the previous chapter.

### 1. Define a Custom Service

1.  Create a `srv` directory: `mkdir -p ~/ros2_ws/src/my_first_package/my_first_package/srv`
2.  Create the file `AddTwoInts.srv` inside the `srv` directory:
    ```
    # ~/ros2_ws/src/my_first_package/my_first_package/srv/AddTwoInts.srv
    int64 a
    int64 b
    ---
    int64 sum
    ```
3.  Update `package.xml` and `setup.py` as you did for custom messages to ensure the service definition is found and built.

### 2. Define a Custom Action

1.  Create an `action` directory: `mkdir -p ~/ros2_ws/src/my_first_package/my_first_package/action`
2.  Create the file `Fibonacci.action` inside the `action` directory:
    ```
    # ~/ros2_ws/src/my_first_package/my_first_package/action/Fibonacci.action
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
3.  Update `package.xml` and `setup.py` to include the new action definition.

### 3. Build and Implement

1.  **Build the definitions**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
2.  **Implement the Server/Client Nodes**: Create the Python files for the service server/client and action server/client within `my_first_package/my_first_package/`. Use the code from the templates below, but be sure to import your newly defined custom types (e.g., `from my_first_package.srv import AddTwoInts`).
3.  **Update `setup.py`**: Add the new nodes to the `entry_points` in your `setup.py` file.
4.  **Build and Run**: Build the workspace again and run your new service and action nodes. Use `ros2 service call` and `ros2 action send_goal` to test them from the command line.

## Diagrams

*   **Service Communication Flow**: A sequence diagram showing the `AddTwoIntsClient` sending a request message to the `AddTwoIntsServer`. The client's timeline shows a "blocked" state until the server processes the request and sends back a response message.
*   **Action State Machine**: A state diagram illustrating the lifecycle of an action goal. It will show the primary states (PENDING, ACTIVE, SUCCEEDED, CANCELED, ABORTED) and the transitions between them, triggered by events like `goal_request`, `accept_goal`, `execute`, `publish_feedback`, `finish`, and `cancel_request`.

## Code Templates

*(Note: These templates use `example_interfaces` for simplicity. You should replace them with your custom-defined `my_first_package` interfaces after creating them.)*

### Python `rclpy` Service Server

```python
# examples/module1/chapter3-services-actions/add_two_ints_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Server started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python `rclpy` Action Server

```python
# examples/module1/chapter3-services-actions/fibonacci_action_server.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeeded()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

*(Client code templates omitted for brevity but would follow a similar pattern of replacing `example_interfaces` with the custom package interfaces.)*

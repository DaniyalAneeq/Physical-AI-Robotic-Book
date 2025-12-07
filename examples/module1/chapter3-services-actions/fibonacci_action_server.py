import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalStatus, GoalResponse
from example_interfaces.action import Fibonacci # Using example_interfaces for simplicity
import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Fibonacci Action Server started.')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.order}')
        if goal_request.order > 0:
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle):
        # Execute the action in a separate thread/process to avoid blocking the main ROS 2 thread
        # For simplicity, we'll execute it directly here, but in a real app, use threading/asyncio
        self.get_logger().info('Goal accepted, executing...')
        goal_handle.execute(self.execute_callback) # This line is conceptually wrong for direct execution, should be a separate thread.

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result(sequence=feedback_msg.partial_sequence)

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate work

        goal_handle.succeeded()
        self.get_logger().info('Goal succeeded')
        return Fibonacci.Result(sequence=feedback_msg.partial_sequence)

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
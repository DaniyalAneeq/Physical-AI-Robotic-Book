import rclpy
from rclpy.node import Node

class BaseNode(Node):
    """
    A reusable base class for ROS 2 nodes in the VLA module.

    This class serves as a foundation for other ROS 2 nodes, providing
    convenience methods for logging. It automatically logs a message upon
    initialization.
    """
    def __init__(self, node_name: str):
        """
        Initializes the BaseNode.

        Args:
            node_name: The name of the node.
        """
        super().__init__(node_name)
        self.get_logger().info(f"Node '{node_name}' initialized.")

    def log_info(self, message: str):
        """Logs an informational message."""
        self.get_logger().info(message)

    def log_warn(self, message: str):
        """Logs a warning message."""
        self.get_logger().warn(message)

    def log_error(self, message: str):
        """Logs an error message."""
        self.get_logger().error(message)

# Example usage (for testing, not part of the class itself)
if __name__ == '__main__':
    rclpy.init()
    minimal_node = BaseNode('minimal_base_node')
    minimal_node.log_info("This is an info message from BaseNode.")
    rclpy.spin_once(minimal_node, timeout_sec=1)
    minimal_node.destroy_node()
    rclpy.shutdown()

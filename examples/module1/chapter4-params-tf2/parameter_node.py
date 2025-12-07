import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'hello')
        self.declare_parameter('frequency', 1.0)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ParameterNode started.')

    def timer_callback(self):
        my_parameter = self.get_parameter('my_parameter').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.get_logger().info(f'My parameter is: {my_parameter}, Frequency: {frequency}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

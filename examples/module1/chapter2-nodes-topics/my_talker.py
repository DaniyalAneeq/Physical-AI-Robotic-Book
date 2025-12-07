import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyTalker(Node):

    def __init__(self):
        super().__init__('my_talker')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 1.0  # seconds
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
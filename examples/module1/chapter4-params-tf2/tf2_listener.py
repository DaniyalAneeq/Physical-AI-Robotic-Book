import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped

class TF2Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('TF2 Listener started.')

    def timer_callback(self):
        # Define a point in the 'base_link' frame
        point_in_base_link = PointStamped()
        point_in_base_link.header.frame_id = 'base_link'
        point_in_base_link.header.stamp = self.get_clock().now().to_msg()
        point_in_base_link.point.x = 0.5
        point_in_base_link.point.y = 0.0
        point_in_base_link.point.z = 0.0

        try:
            # Transform the point to the 'world' frame
            point_in_world = self.tf_buffer.transform(
                point_in_base_link, 'world', timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info(
                f'Point in world frame: x={point_in_world.point.x:.2f}, '
                f'y={point_in_world.point.y:.2f}, z={point_in_world.point.z:.2f}'
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TF2Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

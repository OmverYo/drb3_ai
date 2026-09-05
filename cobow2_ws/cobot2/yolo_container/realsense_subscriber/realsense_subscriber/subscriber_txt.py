import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class RealSenseSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_subscriber')

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

    def rgb_callback(self, msg):
        self.get_logger().info(f"RGB Received: {msg.width}x{msg.height}")

    def depth_callback(self, msg):
        self.get_logger().info(f"Depth Received: {msg.width}x{msg.height}")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
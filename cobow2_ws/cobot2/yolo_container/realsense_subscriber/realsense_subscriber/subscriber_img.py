import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy

from cv_bridge import CvBridge
import cv2

class RealSenseSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            qos_profile
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            qos_profile
        )

        self.get_logger().info('RealSense Image Subscriber started.')
        self.get_logger().info('Waiting for RGB and Depth images...')

    def rgb_callback(self, msg):

        try:
            # ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            cv2.imshow('RGB Image', frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f'RGB Error: {e}'
            )

    def depth_callback(self, msg):

        try:
            # ROS Image -> OpenCV
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

            depth_normalized = cv2.normalize(
                depth,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )

            depth_normalized = depth_normalized.astype('uint8')

            depth_colormap = cv2.applyColorMap(
                depth_normalized,
                cv2.COLORMAP_JET
            )

            cv2.imshow(
                'Depth Image',
                depth_colormap
            )

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f'Depth Error: {e}'
            )

    def destroy_node(self):

        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)

    node = RealSenseSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

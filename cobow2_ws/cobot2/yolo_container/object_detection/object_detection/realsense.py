import time
from collections import deque

import numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = self.depth_frame = self.intrinsics = None
        self.color_frame_stamp = None
        self.colors, self.depths = deque(maxlen=8), deque(maxlen=8)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self.color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self.camera_info_callback, qos_profile_sensor_data)
        self._img_exec = SingleThreadedExecutor()
        self._img_exec.add_node(self)

    def spin_once(self, timeout_sec=0.1):
        self._img_exec.spin_once(timeout_sec=timeout_sec)

    def camera_info_callback(self, msg):
        self.intrinsics = dict(fx=msg.k[0], fy=msg.k[4], ppx=msg.k[2], ppy=msg.k[5])

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_stamp = self.stamp(msg)
        self.colors.append((self.color_frame_stamp, self.color_frame))

    def depth_callback(self, msg):
        encoding = msg.encoding.upper()
        if encoding not in ('16UC1', 'MONO16', '32FC1'):
            self.get_logger().error('Unsupported depth encoding: ' + msg.encoding)
            return
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        scale = 1.0 if encoding == '32FC1' else 0.001
        # Existing position services expect millimetres, for either encoding.
        self.depth_frame = raw.astype(np.float32) * scale * 1000
        self.depths.append((self.stamp(msg), raw, scale))

    def get_color_frame(self):
        return self.color_frame

    def get_depth_frame(self):
        return self.depth_frame

    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    def get_camera_intrinsic(self):
        return self.intrinsics

    def get_synced_rgbd(self, timeout_sec=2.0, max_delta_sec=0.02):
        """Return one fresh aligned color/depth pair.

        Clearing both queues makes the pair newer than this request.  Do not
        compare camera message stamps with the node clock: hardware timestamps,
        simulated time and system time do not always share the same epoch.
        """
        timeout_sec = float(timeout_sec)
        max_delta_sec = float(max_delta_sec)
        if timeout_sec <= 0 or max_delta_sec <= 0:
            raise ValueError('RGB-D timeout and timestamp tolerance must be positive')
        self.colors.clear()
        self.depths.clear()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.02)
            if not self.colors or not self.depths:
                continue

            # Select the closest pair rather than relying on callback order.
            best = None
            for cs, color in self.colors:
                for ds, depth, scale in self.depths:
                    delta = abs(cs - ds)
                    if best is None or delta < best[0]:
                        best = (delta, color, depth, scale)
            if best is not None and best[0] <= max_delta_sec:
                return best[1].copy(), best[2].copy(), best[3]
        return None, None, 0.001

    def destroy_node(self):
        self._img_exec.remove_node(self)
        self._img_exec.shutdown()
        return super().destroy_node()

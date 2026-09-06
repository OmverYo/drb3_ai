import json
import numpy as np
import rclpy
import urllib.error
import urllib.request
import os
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel
from object_detection.aruco import ArucoModel


PACKAGE_NAME = 'object_detection'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.board_api_url = os.getenv(
            'JANGGI_BOARD_API_URL', 'http://127.0.0.1:5000/api/board'
        )
        self.board_corners = self._load_board_corners()
        self.aruco = ArucoModel()
        sync_interval = float(os.getenv('JANGGI_SYNC_INTERVAL', '2.0'))
        self.create_timer(sync_interval, self._sync_board)
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def _load_board_corners(self):
        """Load image points in top-left, top-right, bottom-right, bottom-left order."""
        raw_corners = os.getenv('JANGGI_BOARD_CORNERS')
        if not raw_corners:
            self.get_logger().warn(
                "JANGGI_BOARD_CORNERS is not set; using ArUco markers (IDs 0-3)."
            )
            return None
        try:
            corners = np.asarray(json.loads(raw_corners), dtype=np.float32)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Invalid JANGGI_BOARD_CORNERS: {error}")
            return None
        if corners.shape != (4, 2):
            self.get_logger().error(
                "JANGGI_BOARD_CORNERS must contain four [x, y] points."
            )
            return None
        return corners

    def _detect_aruco_corners(self):
        """Detect the four board corner markers (IDs 0-3) in the current frame."""
        self.img_node.spin_once()
        frame = self.img_node.get_color_frame()
        if frame is None:
            return None
        result = self.aruco.detect(frame)
        if not result['all_required']:
            missing = sorted(
                self.aruco.required_ids - result['detected_ids']
            )
            self.get_logger().warn(
                f"ArUco markers not all visible: missing IDs {missing}"
            )
            return None

        centers = result['centers']
        return np.asarray(
            [[centers[i][0], centers[i][1]] for i in (0, 1, 2, 3)],
            dtype=np.float32,
        )

    def _board_homography(self):
        source = self.board_corners
        target = np.asarray(
            [[0, 0], [8, 0], [8, 9], [0, 9]], dtype=np.float32
        )
        matrix = []
        values = []
        for (x, y), (u, v) in zip(source, target):
            matrix.extend([
                [x, y, 1, 0, 0, 0, -u * x, -u * y],
                [0, 0, 0, x, y, 1, -v * x, -v * y],
            ])
            values.extend([u, v])
        try:
            return np.linalg.solve(
                np.asarray(matrix, dtype=np.float32),
                np.asarray(values, dtype=np.float32),
            )
        except np.linalg.LinAlgError:
            self.get_logger().error("Board corners cannot define a homography.")
            return None

    def _sync_board(self):
        aruco_corners = self._detect_aruco_corners()
        if aruco_corners is not None:
            self.board_corners = aruco_corners
        elif self.board_corners is None:
            return

        detections = self.model.get_all_detections(self.img_node)
        board = [[None for _ in range(9)] for _ in range(10)]
        for detection in detections:
            cell = self._pixel_to_board_cell(detection['box'])
            if cell is None:
                continue
            row, col = cell
            current = board[row][col]
            if current is None or detection['score'] > current['score']:
                board[row][col] = {
                    'name': detection['name'],
                    'score': detection['score'],
                }

        payload = {
            'board': [
                [piece['name'] if piece else None for piece in row]
                for row in board
            ],
            'currentTurn': 'red',
        }
        self._send_board(payload)

    def _pixel_to_board_cell(self, box):
        """Map a detection center to the nearest board intersection."""
        center = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        homography = self._board_homography()
        if homography is None:
            return None

        x, y = center
        projected = np.asarray([
            homography[0] * x + homography[1] * y + homography[2],
            homography[3] * x + homography[4] * y + homography[5],
            homography[6] * x + homography[7] * y + 1,
        ])
        if projected[2] == 0:
            return None
        col = round(projected[0] / projected[2])
        row = round(projected[1] / projected[2])
        if 0 <= row < 10 and 0 <= col < 9:
            return row, col
        return None

    def _send_board(self, payload):
        body = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            self.board_api_url,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='PUT',
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                if response.status != 200:
                    self.get_logger().warn(
                        f"Board API returned HTTP {response.status}."
                    )
        except (urllib.error.URLError, TimeoutError) as error:
            self.get_logger().warn(f"Could not update board API: {error}")

    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        self.img_node.spin_once()

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y, win=5):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        h, w = frame.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None
        x0, x1 = max(0, x - win), min(w, x + win + 1)
        y0, y1 = max(0, y - win), min(h, y + win + 1)
        patch = frame[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            self.get_logger().warn(f"No valid depth around ({x},{y}).")
            return None
        return float(np.median(valid))

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            self.img_node.spin_once()
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

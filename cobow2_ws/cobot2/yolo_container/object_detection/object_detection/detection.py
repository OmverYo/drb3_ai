import json
import numpy as np
import rclpy
import urllib.error
import urllib.request
import os
from rclpy.node import Node
from std_srvs.srv import SetBool

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition, SrvAllPositions
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel
from object_detection.aruco import (
    ArucoModel, board_reference_points_mm,
    BOARD_W_MM, BOARD_H_MM, GRID_COLS, GRID_ROWS, GRID_X_MM, GRID_Y_MM,
)


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
        # vision 모드에서 아루코 계산 위치(board_xyz_before)를 보정하기 위한 서비스.
        # keyword(클래스) 매칭이 아니라, 화면에 보이는 모든 detection 후보를
        # 카메라 프레임 좌표로 반환한다. "어느 후보가 가장 가까운가"는
        # 로봇 베이스 프레임/현재 로봇 자세를 알고 있는 robot_control 쪽에서 판단한다.
        self.create_service(
            SrvAllPositions,
            'get_all_positions',
            self.handle_get_all_positions
        )

        self.board_api_url = os.getenv(
            'JANGGI_BOARD_API_URL', 'http://127.0.0.1:5000/api/board'
        )
        self.board_corners = self._load_board_corners()
        self.aruco = ArucoModel()
        sync_interval = float(os.getenv('JANGGI_SYNC_INTERVAL', '2.0'))
        self.board_sync_enabled = False
        self.board_sync_service = self.create_service(
            SetBool,
            '/set_board_sync',
            self.handle_set_board_sync
        )
        self.board_timer = self.create_timer(sync_interval, self._sync_board)
        self.create_timer(sync_interval, self._sync_board)
        self.get_logger().info("ObjectDetectionNode initialized.")
        self.get_logger().info(
            "Board mapping: ArucoCalculator reference corners; "
            "350x350 mm board, 30 mm outward X gaps; same-frame YOLO."
        )

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def _load_board_corners(self):
        """Optional static fallback: actual OUTER BOARD INTERSECTIONS in pixels.

        Order: TL, TR, BR, BL. These are NOT ArUco centers/reference corners.
        Use only with a fixed camera and board. ArUco takes priority when visible.
        """
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
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            self.get_logger().error(
                "JANGGI_BOARD_CORNERS must contain four [x, y] points."
            )
            return None
        return corners

    def _detect_aruco_corners(self, frame):
        """Select the same inner reference corners as ArucoCalculator."""
        result = self.aruco.detect(frame)
        points = self.aruco.get_board_reference_pixels(result)
        if points is None:
            self.get_logger().warn("ArUco reference corners unavailable; IDs 0-3 required.")
        return points

    def _board_homography(self, source=None, from_aruco=False):
        """Image pixels -> board millimetres, with an explicit source convention.

        ArUco references span x=-30..380; actual board spans x=0..350.
        Manual JANGGI_BOARD_CORNERS already represent actual intersections.
        Raw image homography retains the existing lens-distortion limitation.
        """
        if source is None:
            source = self.board_corners
        if source is None:
            return None
        source = np.asarray(source, dtype=np.float64)
        if source.shape != (4, 2) or not np.all(np.isfinite(source)):
            return None
        target = board_reference_points_mm() if from_aruco else np.asarray([
            [0.0, 0.0], [BOARD_W_MM, 0.0],
            [BOARD_W_MM, BOARD_H_MM], [0.0, BOARD_H_MM],
        ], dtype=np.float64)
        matrix = []
        values = []
        for (x, y), (u, v) in zip(source, target):
            matrix.extend([
                [x, y, 1, 0, 0, 0, -u * x, -u * y],
                [0, 0, 0, x, y, 1, -v * x, -v * y],
            ])
            values.extend([u, v])
        try:
            h = np.linalg.solve(
                np.asarray(matrix, dtype=np.float64),
                np.asarray(values, dtype=np.float64),
            )
            homography = np.append(h, 1.0).reshape(3, 3)
            if not np.all(np.isfinite(homography)) or np.linalg.matrix_rank(homography) < 3:
                return None
            return homography
        except np.linalg.LinAlgError:
            self.get_logger().error("Board reference points cannot define a homography.")
            return None

    def handle_set_board_sync(self, request, response):
        """robot_control의 요청: False이면 동기화 중지, True이면 동기화 시작."""
        self.board_sync_enabled = bool(request.data)
        response.success = True
        response.message = (
            "Board sync enabled." if self.board_sync_enabled else "Board sync disabled."
        )
        self.get_logger().info(response.message)
        return response

    def _sync_board(self):
        # 2초 타이머는 유지하고, 꺼져 있으면 동기화/API 전송을 건너뜀.
        if not self.board_sync_enabled:
            return
        # One snapshot for BOTH detectors; never average boxes from another pose.
        self.img_node.spin_once()
        frame = self.img_node.get_color_frame()
        if frame is None:
            return
        frame = frame.copy()
        aruco_corners = self._detect_aruco_corners(frame)
        if aruco_corners is not None:
            homography = self._board_homography(aruco_corners, from_aruco=True)
        elif self.board_corners is not None:
            # Only the explicitly configured static fallback; no cached ArUco pose.
            homography = self._board_homography(self.board_corners, from_aruco=False)
        else:
            return
        if homography is None:
            return

        # Board-only inference; general object services retain multi-frame YOLO.
        detections = self.model.get_board_detections(frame)
        board = [[None for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        for detection in detections:
            cell = self._pixel_to_board_cell(detection['box'], homography)
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

    def _pixel_to_board_cell(self, box, homography=None):
        """Pixel center -> board mm -> nearest 0-based intersection."""
        box = np.asarray(box, dtype=np.float64)
        if box.shape != (4,) or not np.all(np.isfinite(box)):
            return None
        if box[2] <= box[0] or box[3] <= box[1]:
            return None
        if homography is None:
            homography = self._board_homography()
        if homography is None:
            return None
        center = np.array([
            (box[0] + box[2]) / 2, (box[1] + box[3]) / 2, 1.0,
        ])
        projected = homography @ center
        if not np.all(np.isfinite(projected)) or abs(projected[2]) < 1e-12:
            return None
        board_x, board_y = projected[:2] / projected[2]
        col_float = board_x / GRID_X_MM
        row_float = board_y / GRID_Y_MM
        # Do not clamp off-board objects onto an edge intersection.
        if not (-0.5 < col_float < GRID_COLS - 0.5
                and -0.5 < row_float < GRID_ROWS - 0.5):
            return None
        col = int(round(col_float))
        row = int(round(row_float))
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
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

    def handle_get_all_positions(self, request, response):
        self.get_logger().info(
            f"Received get_all_positions request (min_score={request.min_score})"
        )
        xs, ys, zs, scores = self._compute_all_positions(min_score=request.min_score)
        response.x = xs
        response.y = ys
        response.z = zs
        response.score = scores
        return response

    def _compute_all_positions(self, min_score=0.0):
        # get_all_detections()가 이미 img_node.spin_once()/프레임 수집을 내부에서 처리한다.
        detections = self.model.get_all_detections(self.img_node)

        xs, ys, zs, scores = [], [], [], []
        for det in detections:
            score = det["score"]
            if score < min_score:
                continue
            box = det["box"]
            cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2 + 12.5])
            cz = self._get_depth(cx, cy)
            if cz is None:
                continue
            x, y, z = self._pixel_to_camera_coords(cx, cy, cz)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            scores.append(float(score))

        if not xs:
            self.get_logger().warn("No detections found for get_all_positions.")

        return xs, ys, zs, scores

    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        target_name, board_cell = self._parse_target_selector(target)
        if board_cell is not None:
            return self._compute_board_position(target_name, board_cell)

        self.img_node.spin_once()

        box, score = self.model.get_best_detection(self.img_node, target_name)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2 + 12.5])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _parse_target_selector(self, target):
        """Parse ``class`` or ``class@row,col``/``class@row-col`` selectors."""
        target = str(target).strip()
        if '@' not in target:
            return target, None

        target_name, cell_text = target.rsplit('@', 1)
        parts = cell_text.replace('-', ',').split(',')
        if len(parts) != 2:
            self.get_logger().warn(
                f"Invalid target selector '{target}'. Use class@row,col."
            )
            return target_name.strip(), None
        try:
            row, col = (int(part.strip()) for part in parts)
        except ValueError:
            self.get_logger().warn(
                f"Invalid target selector '{target}'. Use class@row,col."
            )
            return target_name.strip(), None
        if not (1 <= row <= GRID_ROWS and 1 <= col <= GRID_COLS):
            self.get_logger().warn(
                f"Target board cell out of range: row={row}, col={col}."
            )
            return target_name.strip(), None
        return target_name.strip(), (row - 1, col - 1)

    def _compute_board_position(self, target_name, board_cell):
        """Select a class instance by its board cell from one camera snapshot."""
        self.img_node.spin_once()
        frame = self.img_node.get_color_frame()
        if frame is None:
            self.get_logger().warn("No color frame for board-positioned target.")
            return 0.0, 0.0, 0.0

        frame = frame.copy()
        aruco_corners = self._detect_aruco_corners(frame)
        if aruco_corners is not None:
            homography = self._board_homography(aruco_corners, from_aruco=True)
        elif self.board_corners is not None:
            homography = self._board_homography(self.board_corners, from_aruco=False)
        else:
            homography = None
        if homography is None:
            self.get_logger().warn(
                "Cannot select a board-positioned target without board references."
            )
            return 0.0, 0.0, 0.0

        detections = self.model.get_board_detections(frame)
        matches = [
            detection for detection in detections
            if detection['name'] == target_name
            and self._pixel_to_board_cell(detection['box'], homography) == board_cell
        ]
        if not matches:
            self.get_logger().warn(
                f"No '{target_name}' detection found at board cell "
                f"({board_cell[0] + 1},{board_cell[1] + 1})."
            )
            return 0.0, 0.0, 0.0

        detection = max(matches, key=lambda item: item['score'])
        box = detection['box']
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2 + 12.5])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0
        self.get_logger().info(
            f"Selected {target_name} at board cell "
            f"({board_cell[0] + 1},{board_cell[1] + 1}), score={detection['score']:.3f}"
        )
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

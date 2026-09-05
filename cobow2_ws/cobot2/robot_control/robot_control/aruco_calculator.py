import os
import time

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation


# ============================================================
# 장기판 / ArUco 설정
# ============================================================

BOARD_W_MM = 350.0
BOARD_H_MM = 350.0

GRID_COLS = 9
GRID_ROWS = 10

GRID_X_MM = BOARD_W_MM / (GRID_COLS - 1)   # 43.75 mm
GRID_Y_MM = BOARD_H_MM / (GRID_ROWS - 1)   # 38.888... mm

# 실제 설치된 ArUco reference corner가
# 장기판 좌/우 끝 교차점보다 바깥으로 30 mm 이동된 상태
LEFT_OFFSET_MM = 30.0
RIGHT_OFFSET_MM = 30.0

# 각 marker에서 장기판 쪽을 향하는 reference corner
BOARD_CORNER_INDEX_BY_ID = {
    0: 2,  # top-left marker -> BR corner
    1: 3,  # top-right marker -> BL corner
    2: 0,  # bottom-right marker -> TL corner
    3: 1,  # bottom-left marker -> TR corner
}

REQUIRED_MARKER_IDS = (0, 1, 2, 3)

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# solvePnP reprojection error가 이 값보다 크면 board pose를 사용하지 않음
MAX_REPROJECTION_ERROR_PX = 3.0

COLOR_TOPIC = "/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"


class ArucoCalculator:
    def __init__(
        self,
        node,
        get_current_posx_fn,
        t_gripper_camera_path=None,
        t_gripper_camera=None,
        color_topic=COLOR_TOPIC,
        camera_info_topic=CAMERA_INFO_TOPIC,
    ):
        self.node = node
        self.get_current_posx_fn = get_current_posx_fn

        self.bridge = CvBridge()

        self.latest_color_frame = None
        self.latest_color_stamp = None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.last_reprojection_error_px = None
        self.last_T_camera_board = None
        self.last_T_base_board = None

        # ----------------------------------------------------
        # Hand-Eye transform: T_gripper_camera = ^gripper T_camera
        # ----------------------------------------------------
        if t_gripper_camera is not None:
            self.T_gripper_camera = np.asarray(t_gripper_camera, dtype=np.float64)
        elif t_gripper_camera_path is not None:
            if not os.path.isfile(t_gripper_camera_path):
                raise FileNotFoundError(
                    f"T_gripper2camera.npy 파일을 찾을 수 없습니다: {t_gripper_camera_path}"
                )
            self.T_gripper_camera = np.load(t_gripper_camera_path).astype(np.float64)
        else:
            raise ValueError("t_gripper_camera_path 또는 t_gripper_camera 중 하나는 반드시 필요합니다.")

        if self.T_gripper_camera.shape != (4, 4):
            raise ValueError(
                f"T_gripper_camera shape은 (4,4)여야 합니다. 현재 shape={self.T_gripper_camera.shape}"
            )

        # ----------------------------------------------------
        # ArUco detector
        # ----------------------------------------------------
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # ----------------------------------------------------
        # RealSense ROS subscribers (별도 realsense.py 필요 없음)
        # ----------------------------------------------------
        self.color_sub = self.node.create_subscription(
            Image, color_topic, self._color_callback, 10
        )
        self.camera_info_sub = self.node.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 10
        )

        self.node.get_logger().info("BoardLocalizer initialized")
        self.node.get_logger().info(f"  color topic: {color_topic}")
        self.node.get_logger().info(f"  camera info: {camera_info_topic}")

    # ========================================================
    # ROS callbacks
    # ========================================================

    def _color_callback(self, msg):
        try:
            self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color_stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        except Exception as e:
            self.node.get_logger().warn(f"color image 변환 실패: {e}")

    def _camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        dist = np.asarray(msg.d, dtype=np.float64)
        # 일부 드라이버에서 D가 비어 있을 경우 대비
        self.dist_coeffs = dist if dist.size > 0 else np.zeros(5, dtype=np.float64)

    # ========================================================
    # 좌표 / Transform
    # ========================================================

    @staticmethod
    def get_robot_pose_matrix(x, y, z, rx, ry, rz):
        """
        Doosan get_current_posx()의 pose를 ^base T_gripper 4x4 matrix로 변환.
        기존 프로젝트와 동일하게 ZYZ Euler 사용.
        """
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [float(x), float(y), float(z)]
        return T

    @staticmethod
    def board_point_mm(row0, col0):
        """
        내부 장기판 좌표 -> board frame XYZ.
        row0: 0~9, col0: 0~8
        """
        if not (0 <= row0 < GRID_ROWS and 0 <= col0 < GRID_COLS):
            raise ValueError(f"장기판 내부 좌표 범위 오류: row0={row0}, col0={col0}")
        return np.array([col0 * GRID_X_MM, row0 * GRID_Y_MM, 0.0, 1.0], dtype=np.float64)

    # ========================================================
    # Camera 준비
    # ========================================================

    def wait_until_ready(self, timeout_sec=5.0):
        """
        color image와 CameraInfo가 들어올 때까지 기다림.
        robot_control.py가 별도 spin thread를 사용하지 않는 구조를
        고려해서 여기서 rclpy.spin_once()를 수행한다.
        """
        start = time.monotonic()
        while rclpy.ok():
            if (self.latest_color_frame is not None
                    and self.camera_matrix is not None
                    and self.dist_coeffs is not None):
                return True
            if time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.01)
        return False


    '''def update_once(self, timeout_sec=0.1):
        """최신 camera callback을 한 번 처리. robot_control에서 직접 호출 가능."""
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)'''

    # ========================================================
    # ArUco -> Board Pose
    # ========================================================

    def _get_reference_image_points(self, frame):
        """
        ArUco ID 0,1,2,3의 지정 reference corner pixel을
        [ID0, ID1, ID2, ID3] 순서로 반환.

        반환: np.ndarray shape (4,2) / 실패: None
        """
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)

        if ids is None:
            return None

        marker_map = {int(mid): c[0] for c, mid in zip(corners, ids.flatten().tolist())}

        # 4개 모두 보여야만 board pose 계산
        if not all(mid in marker_map for mid in REQUIRED_MARKER_IDS):
            return None

        reference_px = [
            [float(marker_map[mid][BOARD_CORNER_INDEX_BY_ID[mid]][0]),
             float(marker_map[mid][BOARD_CORNER_INDEX_BY_ID[mid]][1])]
            for mid in REQUIRED_MARKER_IDS
        ]

        return np.asarray(reference_px, dtype=np.float64)

    def estimate_camera_to_board(self, frame=None):
        """
        현재 RGB frame에서 ArUco 4개를 이용해 ^camera T_board 계산.
        성공: 4x4 numpy matrix / 실패: None
        """
        if frame is None:
            frame = self.latest_color_frame
        if frame is None or self.camera_matrix is None or self.dist_coeffs is None:
            return None

        image_points = self._get_reference_image_points(frame)
        if image_points is None:
            return None

        # 실제 설치된 4개 reference corner의 BOARD frame 위치
        # ID0=(-30,0,0), ID1=(380,0,0), ID2=(380,350,0), ID3=(-30,350,0)
        object_points = np.array([
            [-LEFT_OFFSET_MM, 0.0, 0.0],
            [BOARD_W_MM + RIGHT_OFFSET_MM, 0.0, 0.0],
            [BOARD_W_MM + RIGHT_OFFSET_MM, BOARD_H_MM, 0.0],
            [-LEFT_OFFSET_MM, BOARD_H_MM, 0.0],
        ], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            return None

        # 가능하면 LM refinement
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points, image_points, self.camera_matrix, self.dist_coeffs, rvec, tvec
            )
        except Exception:
            # OpenCV build에 따라 refine가 없더라도 solvePnP 결과는 그대로 사용 가능
            pass

        # reprojection error 검사
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        error_px = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)))
        self.last_reprojection_error_px = error_px

        if error_px > MAX_REPROJECTION_ERROR_PX:
            self.node.get_logger().warn(f"ArUco reprojection error too high: {error_px:.3f}px")
            return None

        R_camera_board, _ = cv2.Rodrigues(rvec)
        T_camera_board = np.eye(4, dtype=np.float64)
        T_camera_board[:3, :3] = R_camera_board
        T_camera_board[:3, 3] = tvec.reshape(3)

        self.last_T_camera_board = T_camera_board.copy()
        return T_camera_board

    # ========================================================
    # 최종 API
    # ========================================================

    def get_base_point(self, row0, col0, wait_timeout_sec=3.0):
        """
        robot_control.py에서 주로 사용할 함수.

        입력: row0=0~9, col0=0~8
        반환: np.array([base_x, base_y, base_z]) / 실패: None
        """
        # 좌표 먼저 검사
        try:
            p_board = self.board_point_mm(row0, col0)
        except ValueError as e:
            self.node.get_logger().error(str(e))
            return None

        # camera data 준비
        if not self.wait_until_ready(timeout_sec=wait_timeout_sec):
            self.node.get_logger().warn("RealSense image / CameraInfo 수신 대기 timeout")
            return None


        # Camera -> Board
        T_camera_board = self.estimate_camera_to_board(self.latest_color_frame)
        if T_camera_board is None:
            self.node.get_logger().warn("ArUco 4개 board pose 계산 실패")
            return None

        # 현재 로봇 pose
        try:
            robot_pose = self.get_current_posx_fn()[0]
        except Exception as e:
            self.node.get_logger().error(f"get_current_posx 실패: {e}")
            return None

        if robot_pose is None or len(robot_pose) < 6:
            self.node.get_logger().error("잘못된 robot pose")
            return None

        T_base_gripper = self.get_robot_pose_matrix(*robot_pose[:6])

        # 좌표계 연결: ^base T_board = ^base T_gripper @ ^gripper T_camera @ ^camera T_board
        T_base_board = T_base_gripper @ self.T_gripper_camera @ T_camera_board
        self.last_T_base_board = T_base_board.copy()

        # 목표 한 점만 계산
        base_xyz = (T_base_board @ p_board)[:3].copy()

        self.node.get_logger().info(
            f"Board internal ({row0},{col0}) -> BASE XYZ "
            f"({base_xyz[0]:.3f}, {base_xyz[1]:.3f}, {base_xyz[2]:.3f}) "
            f"| reproj={self.last_reprojection_error_px:.3f}px"
        )

        return base_xyz

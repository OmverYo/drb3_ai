"""
키
- q / ESC : 종료
- p       : 90개 전체 BASE XYZ 출력
- s       : 현재 화면 저장
- f       : 현재 T_base_board freeze / unfreeze
- c       : 선택한 점 해제

마우스
- 왼쪽 클릭 : 가장 가까운 장기판 교차점 선택 + BASE XYZ 출력
"""

import os
from datetime import datetime

# pyrefly: ignore [missing-import]
import cv2
import numpy as np
import rclpy
import DR_init
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory



# ============================================================
# RealSense ROS2 직접 구독
# - 별도 realsense.py / ImgNode 필요 없음
# - 현재 DSR node에 subscriber를 직접 붙임
# ============================================================

class RealSenseSubscriber:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        self.color_frame = None
        self.color_frame_stamp = None
        self.intrinsics = None

        self.color_sub = node.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._color_callback,
            10,
        )

        self.camera_info_sub = node.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self._camera_info_callback,
            10,
        )

    def _color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )
        self.color_frame_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )

    def _camera_info_callback(self, msg):
        dist_coeffs = np.asarray(
            msg.d,
            dtype=np.float64,
        )

        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros(
                5,
                dtype=np.float64,
            )

        self.intrinsics = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "ppx": float(msg.k[2]),
            "ppy": float(msg.k[5]),

            "camera_matrix": np.asarray(
                msg.k,
                dtype=np.float64,
            ).reshape(3, 3),

            "dist_coeffs": dist_coeffs,

            "width": int(msg.width),
            "height": int(msg.height),
            "distortion_model": msg.distortion_model,
        }

    def get_color_frame(self):
        return self.color_frame

    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    def get_camera_intrinsic(self):
        return self.intrinsics


# ============================================================
# 사용자 설정
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_2FG"
TCP_NAME = "2FG_TCP"

PACKAGE_NAME = "robot_control"

PACKAGE_PATH = get_package_share_directory(
    PACKAGE_NAME
)

T_GRIPPER_CAMERA_PATH = os.path.join(
    PACKAGE_PATH,
    "resource",
    "T_gripper2camera.npy"
)

BOARD_W_MM = 350.0
BOARD_H_MM = 350.0

LEFT_OFFSET_MM = 30.0
RIGHT_OFFSET_MM = 30.0

GRID_COLS = 9
GRID_ROWS = 10
GRID_X_MM = BOARD_W_MM / (GRID_COLS - 1)
GRID_Y_MM = BOARD_H_MM / (GRID_ROWS - 1)

REQUIRED_IDS = {0, 1, 2, 3}
ARUCO_DICT = cv2.aruco.DICT_4X4_50

SAVE_DIR = "./aruco_base_test"

# 각 마커에서 장기판 쪽 안쪽 corner index
BOARD_CORNER_INDEX_BY_ID = {
    0: 2,  # ID0 bottom-right -> board TL
    1: 3,  # ID1 bottom-left  -> board TR
    2: 0,  # ID2 top-left     -> board BR
    3: 1,  # ID3 top-right    -> board BL
}


# ============================================================
# Robot / Transform
# ============================================================

def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def transform_board_to_base(T_base_gripper, T_gripper_camera, T_camera_board):
    return T_base_gripper @ T_gripper_camera @ T_camera_board


def board_point_mm(row, col):
    return np.array([col * GRID_X_MM, row * GRID_Y_MM, 0.0, 1.0], dtype=np.float64)


def make_all_base_points(T_base_board):
    result = {}
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            p_base = T_base_board @ board_point_mm(row, col)
            result[(row, col)] = p_base[:3].copy()
    return result


# ============================================================
# ArUco / Board Pose
# ============================================================

def create_aruco_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return dictionary, detector, True
    params = cv2.aruco.DetectorParameters_create()
    return dictionary, params, False


def detect_aruco(frame, dictionary, detector_or_params, new_api):
    if new_api:
        return detector_or_params.detectMarkers(frame)
    return cv2.aruco.detectMarkers(frame, dictionary, parameters=detector_or_params)


def get_marker_dict(corners, ids):
    marker_dict = {}
    if ids is None:
        return marker_dict
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_dict[int(marker_id)] = np.asarray(marker_corners[0], dtype=np.float32)
    return marker_dict


def get_board_endpoints(marker_dict):
    if not all(mid in marker_dict for mid in REQUIRED_IDS):
        return None
    tl = marker_dict[0][BOARD_CORNER_INDEX_BY_ID[0]]
    tr = marker_dict[1][BOARD_CORNER_INDEX_BY_ID[1]]
    br = marker_dict[2][BOARD_CORNER_INDEX_BY_ID[2]]
    bl = marker_dict[3][BOARD_CORNER_INDEX_BY_ID[3]]
    return np.asarray([tl, tr, br, bl], dtype=np.float32)


def estimate_board_pose(board_endpoints_px, camera_matrix, dist_coeffs):
    """
    solvePnP 결과:
        P_camera = T_camera_board @ P_board

    image_points는 ArUco 안쪽 corner 4개이지만,
    실제 장기판 끝점보다 좌우 30 mm 바깥에 있다.
    장기판 좌표계 기준:
      ID0 ref = (-30,   0, 0)
      ID1 ref = (380,   0, 0)
      ID2 ref = (380, 350, 0)
      ID3 ref = (-30, 350, 0)
    """
    object_points = np.array([
        [-LEFT_OFFSET_MM, 0.0, 0.0],
        [BOARD_W_MM + RIGHT_OFFSET_MM, 0.0, 0.0],
        [BOARD_W_MM + RIGHT_OFFSET_MM, BOARD_H_MM, 0.0],
        [-LEFT_OFFSET_MM, BOARD_H_MM, 0.0],
    ], dtype=np.float32)

    image_points = np.asarray(board_endpoints_px, dtype=np.float32)

    flag = cv2.SOLVEPNP_IPPE if hasattr(cv2, "SOLVEPNP_IPPE") else cv2.SOLVEPNP_ITERATIVE

    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs, flags=flag
    )

    if not ok:
        return None

    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec
            )
        except cv2.error:
            pass

    R, _ = cv2.Rodrigues(rvec)
    T_camera_board = np.eye(4, dtype=np.float64)
    T_camera_board[:3, :3] = R
    T_camera_board[:3, 3] = tvec.reshape(3)

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    reprojection_error_px = float(np.mean(np.linalg.norm(projected - image_points, axis=1)))

    return {
        "T_camera_board": T_camera_board,
        "rvec": rvec,
        "tvec": tvec,
        "reprojection_error_px": reprojection_error_px,
    }


# ============================================================
# Grid Projection
# ============================================================

def make_grid_object_points():
    keys, pts = [], []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            keys.append((row, col))
            pts.append([col * GRID_X_MM, row * GRID_Y_MM, 0.0])
    return keys, np.asarray(pts, dtype=np.float32)


def project_grid_to_image(rvec, tvec, camera_matrix, dist_coeffs):
    keys, object_points = make_grid_object_points()
    image_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    image_points = image_points.reshape(-1, 2)
    return {key: image_points[i] for i, key in enumerate(keys)}


def project_actual_board_corners_to_image(rvec, tvec, camera_matrix, dist_coeffs):
    """
    실제 장기판 경계 TL/TR/BR/BL (0~350 mm)을 pixel로 투영.
    ArUco 기준점은 이 경계보다 좌우 30 mm 바깥에 있다.
    """
    board_corners = np.array([
        [0.0, 0.0, 0.0],
        [BOARD_W_MM, 0.0, 0.0],
        [BOARD_W_MM, BOARD_H_MM, 0.0],
        [0.0, BOARD_H_MM, 0.0],
    ], dtype=np.float32)
    image_points, _ = cv2.projectPoints(board_corners, rvec, tvec, camera_matrix, dist_coeffs)
    return image_points.reshape(-1, 2)


# ============================================================
# UI state
# ============================================================

class RuntimeState:
    def __init__(self):
        self.grid_image = {}
        self.base_points = {}
        self.T_base_board = None
        self.selected = None
        self.frozen = False
        self.frozen_T_base_board = None
        self.frozen_base_points = None

    def active_base_points(self):
        if self.frozen and self.frozen_base_points is not None:
            return self.frozen_base_points
        return self.base_points

    def active_T_base_board(self):
        if self.frozen and self.frozen_T_base_board is not None:
            return self.frozen_T_base_board
        return self.T_base_board


state = RuntimeState()


def select_nearest_grid(x, y):
    if not state.grid_image:
        print("아직 grid가 계산되지 않았습니다.")
        return

    click = np.array([x, y], dtype=np.float64)
    best_key, best_dist = None, float("inf")

    for key, p in state.grid_image.items():
        d = float(np.linalg.norm(click - np.asarray(p, dtype=np.float64)))
        if d < best_dist:
            best_dist = d
            best_key = key

    state.selected = best_key
    base_points = state.active_base_points()

    if best_key is not None and best_key in base_points:
        xyz = base_points[best_key]
        row, col = best_key
        print("=" * 72)
        print(f"Selected grid: row={row}, col={col}")
        print(f"Board coordinate [mm]: X={col * GRID_X_MM:.3f}, Y={row * GRID_Y_MM:.3f}, Z=0.000")
        print(f"BASE coordinate [mm]: X={xyz[0]:.3f}, Y={xyz[1]:.3f}, Z={xyz[2]:.3f}")
        print(f"Pixel selection distance: {best_dist:.2f}px")
        print("=" * 72)


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        select_nearest_grid(x, y)


# ============================================================
# 영상 표현
# ============================================================

def put_text(img, text, x, y, color=(255, 255, 255), scale=0.5, thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def draw_grid(frame, corners, ids, marker_reference_px, actual_board_corners_px,
              grid_image, pose_info, robot_pos):
    out = frame.copy()

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(out, corners, ids)

    # ArUco 기준점 4개: 실제 장기판보다 좌우 30 mm 바깥
    if marker_reference_px is not None:
        marker_polygon = marker_reference_px.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [marker_polygon], True, (255, 0, 255), 1)
        for name, p in zip(["M0", "M1", "M2", "M3"], marker_reference_px):
            px, py = np.round(p).astype(int)
            cv2.circle(out, (px, py), 5, (0, 0, 255), -1)
            put_text(out, name, px + 7, py - 7, (0, 0, 255), 0.45, 1)

    # 실제 장기판 (0~350 mm) 경계
    if actual_board_corners_px is not None:
        board_polygon = actual_board_corners_px.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [board_polygon], True, (255, 255, 0), 2)
        for name, p in zip(["TL", "TR", "BR", "BL"], actual_board_corners_px):
            px, py = np.round(p).astype(int)
            cv2.circle(out, (px, py), 5, (255, 255, 0), -1)
            put_text(out, name, px + 7, py + 14, (255, 255, 0), 0.45, 1)

    # grid line
    if grid_image:
        for row in range(GRID_ROWS):
            pts = [np.round(grid_image[(row, col)]).astype(int) for col in range(GRID_COLS)]
            cv2.polylines(out, [np.asarray(pts, dtype=np.int32)], False, (80, 180, 80), 1)

        for col in range(GRID_COLS):
            pts = [np.round(grid_image[(row, col)]).astype(int) for row in range(GRID_ROWS)]
            cv2.polylines(out, [np.asarray(pts, dtype=np.int32)], False, (80, 180, 80), 1)

        for key, p in grid_image.items():
            row, col = key
            px, py = np.round(p).astype(int)
            selected = (key == state.selected)
            color = (255, 0, 0) if selected else (0, 255, 255)
            cv2.circle(out, (px, py), 6 if selected else 3, color, -1)
            if selected or row in (0, GRID_ROWS - 1) or col in (0, GRID_COLS - 1):
                put_text(out, f"{row},{col}", px + 5, py - 5, color, 0.35, 1)

    y = 24

    if pose_info is None:
        put_text(out, "Need ArUco IDs 0,1,2,3", 15, y, (0, 0, 255), 0.65, 2)
        return out

    reproj = pose_info["reprojection_error_px"]
    put_text(out, f"PnP reprojection error: {reproj:.3f}px", 15, y, (255, 255, 255), 0.55, 2)
    y += 22

    put_text(out, f"Marker offset: L={LEFT_OFFSET_MM:.1f}mm R={RIGHT_OFFSET_MM:.1f}mm",
             15, y, (255, 255, 255), 0.5, 1)
    y += 22

    if robot_pos is not None:
        put_text(out, f"Robot TCP BASE: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f}, {robot_pos[2]:.1f})",
                 15, y, (255, 255, 255), 0.5, 1)
        y += 22

    T = state.active_T_base_board()
    if T is not None:
        origin = T[:3, 3]
        put_text(out, f"Board origin BASE: ({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm",
                 15, y, (255, 255, 255), 0.5, 1)
        y += 22

    put_text(out, "Transform: " + ("FROZEN" if state.frozen else "LIVE"),
             15, y, (0, 255, 255) if state.frozen else (0, 255, 0), 0.55, 2)
    y += 22

    if state.selected is not None:
        base_points = state.active_base_points()
        if state.selected in base_points:
            row, col = state.selected
            xyz = base_points[state.selected]
            put_text(out,
                     f"Selected ({row},{col}) BASE = ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) mm",
                     15, y, (255, 0, 0), 0.55, 2)

    return out


def print_all_base_points():
    points = state.active_base_points()
    if not points:
        print("아직 BASE 좌표가 계산되지 않았습니다.")
        return

    print("=" * 92)
    print(" row col | board_x board_y | base_x      base_y      base_z   [mm]")
    print("=" * 92)

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            xyz = points[(row, col)]
            print(f"{row:4d} {col:3d} | {col * GRID_X_MM:7.2f} {row * GRID_Y_MM:7.2f} | "
                  f"{xyz[0]:10.3f} {xyz[1]:10.3f} {xyz[2]:10.3f}")

    print("=" * 92)


# ============================================================
# Main
# ============================================================

def main(args=None):
    os.makedirs(SAVE_DIR, exist_ok=True)

    if not hasattr(cv2, "aruco"):
        print("cv2.aruco가 없습니다. opencv-contrib-python을 확인하세요.")
        return

    # ----------------------------------------
    # ROS2 / Doosan 연결
    # ----------------------------------------
    rclpy.init(args=args)
    node = rclpy.create_node("aruco_base_validation", namespace=ROBOT_ID)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_current_posx, set_tool, set_tcp
    except ImportError as e:
        print("DSR_ROBOT2 import 실패:", e)
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tool(TOOL_NAME)
    set_tcp(TCP_NAME)

    # ----------------------------------------
    # RealSense ROS2 직접 subscriber
    # 별도 realsense.py / ImgNode 없음
    # ----------------------------------------
    camera = RealSenseSubscriber(node)

    print("[INFO] RealSense CameraInfo 대기: /camera/color/camera_info")

    intrinsics = None
    while rclpy.ok() and intrinsics is None:
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )
        intrinsics = camera.get_camera_intrinsic()

    if intrinsics is None:
        print("[ERROR] /camera/color/camera_info를 수신하지 못했습니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    camera_matrix = intrinsics["camera_matrix"]
    dist_coeffs = intrinsics["dist_coeffs"]

    print("[INFO] CameraInfo 수신 완료")
    print("[INFO] image size:", (intrinsics["width"], intrinsics["height"]))
    print("[INFO] distortion model:", intrinsics["distortion_model"])
    print("[INFO] camera_matrix:\n", camera_matrix)
    print("[INFO] dist_coeffs:", dist_coeffs.ravel())

    # ----------------------------------------
    # Hand-Eye calibration 결과 load
    # ----------------------------------------
    if not os.path.exists(T_GRIPPER_CAMERA_PATH):
        print("[ERROR] 파일 없음:", T_GRIPPER_CAMERA_PATH)
        node.destroy_node()
        rclpy.shutdown()
        return

    print(
        "[INFO] Hand-Eye file:",
        T_GRIPPER_CAMERA_PATH
    )

    T_gripper_camera = np.load(
        T_GRIPPER_CAMERA_PATH
    )
    
    if T_gripper_camera.shape != (4, 4):
        print("[ERROR] T_gripper2camera.npy가 4x4 행렬이 아닙니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("[INFO] T_gripper_camera:\n", T_gripper_camera)

    # ----------------------------------------
    # ArUco detector
    # ----------------------------------------
    dictionary, detector, new_api = create_aruco_detector()

    cv2.namedWindow("ArUco Board BASE Validation", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("ArUco Board BASE Validation", mouse_callback)

    print("=" * 76)
    print("ArUco 장기판(offset 30mm) -> M0609 BASE 좌표 검증")
    print(f"Marker board coordinates: "
          f"ID0=(-{LEFT_OFFSET_MM:.1f},0), "
          f"ID1=({BOARD_W_MM + RIGHT_OFFSET_MM:.1f},0), "
          f"ID2=({BOARD_W_MM + RIGHT_OFFSET_MM:.1f},{BOARD_H_MM:.1f}), "
          f"ID3=(-{LEFT_OFFSET_MM:.1f},{BOARD_H_MM:.1f})")
    print("Image topic : /camera/color/image_raw (direct subscription)")
    print("CameraInfo : /camera/color/camera_info")
    print("로봇은 움직이지 않고 현재 pose만 읽습니다.")
    print("마우스 클릭 : 해당 교차점 BASE XYZ 출력")
    print("p : 90개 BASE 좌표 출력")
    print("f : T_base_board freeze/unfreeze")
    print("s : 화면 저장")
    print("c : 선택 해제")
    print("q / ESC : 종료")
    print("=" * 76)

    last_stamp = None
    first_frame_checked = False

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            frame = camera.get_color_frame()
            stamp = camera.get_color_frame_stamp()

            if frame is None:
                continue

            if stamp is not None and stamp == last_stamp:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                continue

            last_stamp = stamp

            if not first_frame_checked:
                h, w = frame.shape[:2]
                print("[INFO] color image size:", (w, h))
                info_size = (intrinsics["width"], intrinsics["height"])
                if (w, h) != info_size:
                    print("[WARNING] color image와 CameraInfo 해상도가 다릅니다.")
                    print("image:", (w, h), "camera_info:", info_size)
                first_frame_checked = True

            corners, ids, rejected = detect_aruco(frame, dictionary, detector, new_api)
            marker_dict = get_marker_dict(corners, ids)
            board_endpoints = get_board_endpoints(marker_dict)

            pose_info = None
            robot_pos = None
            actual_board_corners_px = None

            if board_endpoints is not None:
                pose_info = estimate_board_pose(board_endpoints, camera_matrix, dist_coeffs)

            if pose_info is not None:
                actual_board_corners_px = project_actual_board_corners_to_image(
                    pose_info["rvec"], pose_info["tvec"], camera_matrix, dist_coeffs
                )
                grid_image = project_grid_to_image(
                    pose_info["rvec"], pose_info["tvec"], camera_matrix, dist_coeffs
                )
                state.grid_image = grid_image

                # freeze 상태에서는 BASE transform은 그대로 두되 image grid는 계속 live 표시.
                if not state.frozen:
                    robot_pos = get_current_posx()[0]
                    T_base_gripper = get_robot_pose_matrix(*robot_pos)
                    T_base_board = transform_board_to_base(
                        T_base_gripper, T_gripper_camera, pose_info["T_camera_board"]
                    )
                    state.T_base_board = T_base_board
                    state.base_points = make_all_base_points(T_base_board)
                else:
                    robot_pos = get_current_posx()[0]

            out = draw_grid(
                frame, corners, ids, board_endpoints,
                actual_board_corners_px, state.grid_image, pose_info, robot_pos
            )

            cv2.imshow("ArUco Board BASE Validation", out)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break
            elif key == ord("p"):
                print_all_base_points()
            elif key == ord("c"):
                state.selected = None
            elif key == ord("f"):
                if not state.frozen:
                    if state.T_base_board is None:
                        print("아직 T_base_board가 없습니다.")
                    else:
                        state.frozen = True
                        state.frozen_T_base_board = state.T_base_board.copy()
                        state.frozen_base_points = {k: v.copy() for k, v in state.base_points.items()}
                        print("[FREEZE] 현재 T_base_board 고정")
                else:
                    state.frozen = False
                    state.frozen_T_base_board = None
                    state.frozen_base_points = None
                    print("[LIVE] T_base_board 실시간 갱신 재개")
            elif key == ord("s"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = os.path.join(SAVE_DIR, "base_" + timestamp + ".jpg")
                cv2.imwrite(path, out)
                print("saved:", path)

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

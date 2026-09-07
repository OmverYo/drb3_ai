
import cv2
import numpy as np


# Same physical board/reference geometry as aruco_calculator(4).py.
# Offsets are from the selected marker CORNER to the outer intersection.
BOARD_W_MM = 350.0
BOARD_H_MM = 350.0
GRID_COLS = 9
GRID_ROWS = 10
GRID_X_MM = BOARD_W_MM / (GRID_COLS - 1)
GRID_Y_MM = BOARD_H_MM / (GRID_ROWS - 1)
LEFT_OFFSET_MM = 30.0
RIGHT_OFFSET_MM = 30.0
REQUIRED_MARKER_IDS = (0, 1, 2, 3)
BOARD_CORNER_INDEX_BY_ID = {0: 2, 1: 3, 2: 0, 3: 1}


def board_reference_points_mm():
    """ID0, ID1, ID2, ID3 reference corners, not marker centers."""
    return np.asarray([
        [-LEFT_OFFSET_MM, 0.0],
        [BOARD_W_MM + RIGHT_OFFSET_MM, 0.0],
        [BOARD_W_MM + RIGHT_OFFSET_MM, BOARD_H_MM],
        [-LEFT_OFFSET_MM, BOARD_H_MM],
    ], dtype=np.float64)


class ArucoModel:
    """
    YOLO의 YoloModel과 역할을 비슷하게 맞춘 ArUco 검출 래퍼.
    - 입력: OpenCV BGR frame
    - 출력: corners, ids, centers
    """

    def __init__(self, required_ids=(0, 1, 2, 3)):
        self.required_ids = set(required_ids)

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                params
            )
            self.use_new_api = True
        else:
            self.params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_api = False

    def detect(self, frame):
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame,
                self.dictionary,
                parameters=self.params,
            )

        centers = {}
        detected_ids = set()

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                marker_id = int(marker_id)
                pts = marker_corners[0]

                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))

                centers[marker_id] = (cx, cy)
                detected_ids.add(marker_id)

        all_required = self.required_ids.issubset(detected_ids)

        return {
            "corners": corners,
            "ids": ids,
            "rejected": rejected,
            "centers": centers,
            "detected_ids": detected_ids,
            "all_required": all_required,
        }

    @staticmethod
    def get_board_reference_pixels(result):
        """Use the exact ID/corner convention of ArucoCalculator.

        Existing centers remain available for aruco_test.py diagnostics.
        A missing/duplicate required ID invalidates the board mapping.
        """
        if result['ids'] is None:
            return None
        marker_map = {}
        for corners, marker_id in zip(result['corners'], result['ids'].flatten()):
            marker_id = int(marker_id)
            if marker_id not in REQUIRED_MARKER_IDS:
                continue
            if marker_id in marker_map:
                return None
            marker_map[marker_id] = np.asarray(corners).reshape(4, 2)
        if not all(mid in marker_map for mid in REQUIRED_MARKER_IDS):
            return None
        points = np.asarray([
            marker_map[mid][BOARD_CORNER_INDEX_BY_ID[mid]]
            for mid in REQUIRED_MARKER_IDS
        ], dtype=np.float64)
        return points if np.all(np.isfinite(points)) else None

    def draw(self, frame, result):
        out = frame.copy()

        corners = result["corners"]
        ids = result["ids"]

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(out, corners, ids)

        for marker_id, (cx, cy) in result["centers"].items():
            cv2.circle(
                out,
                (int(cx), int(cy)),
                6,
                (0, 0, 255),
                -1
            )
            cv2.putText(
                out,
                f"ID {marker_id}",
                (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return out

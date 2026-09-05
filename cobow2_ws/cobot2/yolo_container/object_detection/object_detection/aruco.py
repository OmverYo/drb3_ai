
import cv2
import numpy as np


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

#!/usr/bin/env python3

import csv
import json
import os
import time
from collections import defaultdict, deque
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from object_detection.realsense import ImgNode
from object_detection.aruco import ArucoModel


REQUIRED_IDS = {0, 1, 2, 3}
JITTER_WINDOW = 30
FPS_WINDOW = 30


class ArucoTestNode(Node):
    """
    detection.py 구조를 따라가는 ArUco 성능 검증 노드.

    detection.py와 동일하게:
      - 메인 Node가 ImgNode를 멤버로 보유
      - ImgNode.spin_once()로 RealSense callback 처리
      - object_detection.realsense.ImgNode 재사용

    출력:
      /aruco/debug_image : 검출 결과 영상
      /aruco/status      : JSON 통계
    """

    def __init__(self):
        super().__init__("aruco_test_node")

        self.img_node = ImgNode()
        self.model = ArucoModel(required_ids=REQUIRED_IDS)
        self.bridge = CvBridge()

        self.debug_pub = self.create_publisher(
            Image,
            "/aruco/debug_image",
            10
        )
        self.status_pub = self.create_publisher(
            String,
            "/aruco/status",
            10
        )

        # Docker 환경에서 cv2.imshow()가 안 되는 경우가 많아 기본값 False.
        self.declare_parameter("show_window", False)
        self.show_window = bool(
            self.get_parameter("show_window").value
        )

        self.declare_parameter(
            "save_dir",
            os.path.expanduser("~/aruco_test")
        )
        self.save_dir = str(
            self.get_parameter("save_dir").value
        )
        os.makedirs(self.save_dir, exist_ok=True)

        self.frame_count = 0
        self.all4_count = 0
        self.per_id_detect_count = defaultdict(int)

        self.center_history = {
            marker_id: deque(maxlen=JITTER_WINDOW)
            for marker_id in REQUIRED_IDS
        }

        self.fps_history = deque(maxlen=FPS_WINDOW)
        self.last_process_time = None
        self.last_stamp = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            self.save_dir,
            f"aruco_{timestamp}.csv"
        )

        self.csv_file = open(
            self.csv_path,
            "w",
            newline="",
            encoding="utf-8"
        )
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "timestamp",
            "frame_index",
            "detected_ids",
            "all4_detected",
            "fps",
            "id0_x", "id0_y",
            "id1_x", "id1_y",
            "id2_x", "id2_y",
            "id3_x", "id3_y",
        ])

        # 약 30 Hz로 새 프레임 확인.
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback
        )

        if self.show_window:
            cv2.namedWindow(
                "ArUco Test",
                cv2.WINDOW_NORMAL
            )

        self.get_logger().info(
            "ArucoTestNode initialized. "
            f"required_ids={sorted(REQUIRED_IDS)}, "
            f"csv={self.csv_path}"
        )

    def timer_callback(self):
        # detection.py / yolo.py가 사용하는 방식과 동일하게
        # ImgNode의 전용 executor를 직접 spin.
        self.img_node.spin_once(timeout_sec=0.0)

        frame = self.img_node.get_color_frame()
        stamp = self.img_node.get_color_frame_stamp()

        if frame is None or stamp is None:
            return

        # 같은 카메라 프레임을 여러 번 통계에 넣지 않음.
        if stamp == self.last_stamp:
            return

        self.last_stamp = stamp

        now = time.perf_counter()
        if self.last_process_time is not None:
            dt = now - self.last_process_time
            if dt > 0:
                self.fps_history.append(1.0 / dt)
        self.last_process_time = now

        fps = (
            float(np.mean(self.fps_history))
            if self.fps_history
            else 0.0
        )

        result = self.model.detect(frame)
        debug = self.model.draw(frame, result)

        self.update_statistics(
            result,
            fps
        )
        self.draw_statistics(
            debug,
            result,
            fps
        )

        # Docker에서도 rqt_image_view / RViz 등으로 확인할 수 있도록 publish.
        debug_msg = self.bridge.cv2_to_imgmsg(
            debug,
            encoding="bgr8"
        )
        self.debug_pub.publish(debug_msg)

        status = self.make_status(
            result,
            fps
        )
        self.status_pub.publish(
            String(data=json.dumps(
                status,
                ensure_ascii=False
            ))
        )

        self.write_csv(
            result,
            fps
        )

        if self.show_window:
            cv2.imshow(
                "ArUco Test",
                debug
            )
            cv2.waitKey(1)

    def update_statistics(self, result, fps):
        self.frame_count += 1

        detected_ids = result["detected_ids"]

        for marker_id in REQUIRED_IDS:
            if marker_id in detected_ids:
                self.per_id_detect_count[
                    marker_id
                ] += 1

                self.center_history[
                    marker_id
                ].append(
                    result["centers"][marker_id]
                )

        if result["all_required"]:
            self.all4_count += 1

    def get_jitter(self, marker_id):
        history = self.center_history[
            marker_id
        ]

        if len(history) < 2:
            return 0.0, 0.0

        arr = np.asarray(
            history,
            dtype=np.float64
        )

        return (
            float(np.std(arr[:, 0])),
            float(np.std(arr[:, 1])),
        )

    def make_status(self, result, fps):
        if self.frame_count > 0:
            all4_rate = (
                100.0
                * self.all4_count
                / self.frame_count
            )
        else:
            all4_rate = 0.0

        per_marker = {}

        for marker_id in sorted(REQUIRED_IDS):
            rate = (
                100.0
                * self.per_id_detect_count[marker_id]
                / self.frame_count
                if self.frame_count > 0
                else 0.0
            )

            jitter_x, jitter_y = self.get_jitter(
                marker_id
            )

            per_marker[str(marker_id)] = {
                "detection_rate": rate,
                "jitter_x_px": jitter_x,
                "jitter_y_px": jitter_y,
            }

        return {
            "frame_count": self.frame_count,
            "detected_ids": sorted(
                result["detected_ids"]
            ),
            "all4_detected": bool(
                result["all_required"]
            ),
            "all4_detection_rate": all4_rate,
            "fps": fps,
            "markers": per_marker,
        }

    def draw_statistics(self, image, result, fps):
        status = self.make_status(
            result,
            fps
        )

        y = 30

        lines = [
            f"FPS: {fps:.1f}",
            f"Detected: {status['detected_ids']}",
            (
                "ALL 4: OK"
                if status["all4_detected"]
                else "ALL 4: FAIL"
            ),
            (
                "All-4 rate: "
                f"{status['all4_detection_rate']:.1f}%"
            ),
        ]

        for text in lines:
            cv2.putText(
                image,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            y += 28

        for marker_id in sorted(REQUIRED_IDS):
            item = status["markers"][
                str(marker_id)
            ]

            text = (
                f"ID{marker_id}: "
                f"{item['detection_rate']:.1f}% "
                f"jitter=("
                f"{item['jitter_x_px']:.2f},"
                f"{item['jitter_y_px']:.2f})px"
            )

            cv2.putText(
                image,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )
            y += 25

    def write_csv(self, result, fps):
        def xy(marker_id):
            return result["centers"].get(
                marker_id,
                ("", "")
            )

        p0 = xy(0)
        p1 = xy(1)
        p2 = xy(2)
        p3 = xy(3)

        self.csv_writer.writerow([
            datetime.now().isoformat(),
            self.frame_count,
            " ".join(
                map(
                    str,
                    sorted(result["detected_ids"])
                )
            ),
            int(result["all_required"]),
            f"{fps:.3f}",
            p0[0], p0[1],
            p1[0], p1[1],
            p2[0], p2[1],
            p3[0], p3[1],
        ])

        if self.frame_count % 30 == 0:
            self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, "csv_file"):
            self.csv_file.flush()
            self.csv_file.close()

        if self.show_window:
            cv2.destroyAllWindows()

        if hasattr(self, "img_node"):
            self.img_node.destroy_node()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

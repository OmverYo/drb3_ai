import os
import json
import time
from collections import Counter

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np


PACKAGE_NAME = "object_detection"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "best.pt"
YOLO_CLASS_NAME_JSON = "class_name_tool.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)


class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}

    def get_frames(self, img_node, duration=1.0):
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            img_node.spin_once()
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("No frames captured in %.2f seconds" % duration)

        print("%d frames captured" % len(frames))
        return list(frames.values())

    def get_best_detection(self, img_node, target):
        """Detect a target on one fresh frame.

        The old implementation sent every frame collected for one second to
        YOLO as a single batch.  That needlessly consumed several GB of VRAM
        and could trigger CUDA OOM when SAM was loaded in the same process.
        """
        img_node.spin_once()
        frames = self.get_frames(img_node)
        if not frames:
            return None, None

        # Use only the newest frame.  This keeps the coordinates consistent
        # with the current depth image and prevents a large inference batch.
        results = self.model(frames[-1], verbose=False)
        print("classes: ")
        print(results[0].names)
        detections = self._aggregate_detections(results)
        label_id = self.reversed_class_dict.get(target)
        if label_id is None:
            print(f"Unknown target '{target}' — not in model classes {list(self.reversed_class_dict.keys())}")
            return None, None
        print("label_id: ", label_id)
        print("detections: ", detections)

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"]

    def get_all_detections(self, img_node):
        """Return detections from one newest frame with model class names."""
        img_node.spin_once()
        frames = self.get_frames(img_node)
        if not frames:
            return []

        results = self.model(frames[-1], verbose=False)
        detections = self._aggregate_detections(results)
        return [
            {
                **detection,
                "name": self.model.names[detection["label"]],
            }
            for detection in detections
        ]

    def get_board_detections(self, frame):
        """Board-only path: infer on the exact ArUco snapshot.

        No camera capture or cross-frame aggregation occurs here. General
        object recognition uses get_best_detection/get_all_detections and
        retains the original multi-frame processing. Both paths share weights.
        """
        if frame is None:
            return []

        results = self.model(frame, verbose=False)
        detections = []
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                label = int(label)
                if not np.isfinite(score) or score < 0.1 or not np.all(np.isfinite(box)):
                    continue
                detections.append({
                    'box': box,
                    'score': float(score),
                    'label': label,
                    'name': self.model.names[label],
                })
        return detections

    def detect_frame(self, frame, confidence_threshold=0.1):
        """Run YOLO on exactly the RGB frame supplied by the caller.

        The grasp service must use the same image for YOLO boxes and SAM
        prompts.  Capturing another frame inside this method would shift the
        boxes relative to the masks and aligned depth image.
        """
        if frame is None:
            return []
        threshold = float(confidence_threshold)
        if not np.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0,1]')

        results = self.model(frame, conf=threshold, verbose=False)
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for box, score, label in zip(
                result.boxes.xyxy.cpu().numpy(),
                result.boxes.conf.cpu().numpy(),
                result.boxes.cls.cpu().numpy(),
            ):
                box = np.asarray(box, dtype=np.float32)
                score = float(score)
                label = int(label)
                if score < threshold or not np.all(np.isfinite(box)):
                    continue
                detections.append({
                    'box': box.tolist(),
                    'score': score,
                    'label': label,
                    'name': str(self.model.names[label]),
                })
        return detections

    def _aggregate_detections(self, results, confidence_threshold=0.1, iou_threshold=0.1):
        raw = []
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append(
                {
                    "box": boxes.mean(axis=0).tolist(),
                    "score": float(scores.mean()),
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        return final

    def _iou(self, box1, box2):
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0

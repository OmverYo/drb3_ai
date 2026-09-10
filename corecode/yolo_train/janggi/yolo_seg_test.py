import cv2
import numpy as np
from ultralytics import YOLO, SAM

detector = YOLO("best.pt")
segmenter = SAM("sam2.1_s.pt")

image = cv2.imread("janggi.jpg")
if image is None:
    raise FileNotFoundError("janggi.jpg를 읽을 수 없습니다.")

# YOLO 검출
det = detector(image, imgsz=640, conf=0.5, verbose=False)[0]

boxes = det.boxes.xyxy.cpu().numpy()
class_ids = det.boxes.cls.int().cpu().numpy()
confidences = det.boxes.conf.cpu().numpy()

if len(boxes) == 0:
    raise RuntimeError("검출된 장기말이 없습니다.")

# YOLO 박스를 SAM 프롬프트로 전달
sam_result = segmenter(
    image,
    bboxes=boxes.tolist(),
    verbose=False,
)[0]

if sam_result.masks is None:
    raise RuntimeError("SAM 마스크 생성에 실패했습니다.")

polygons = sam_result.masks.xy

if len(polygons) != len(boxes):
    raise RuntimeError(
        f"YOLO 검출 수({len(boxes)})와 "
        f"SAM 마스크 수({len(polygons)})가 다릅니다."
    )

# 마스크 표시용 영상
mask_overlay = image.copy()
output = image.copy()

rng = np.random.default_rng(42)
colors = rng.integers(30, 230, size=(len(boxes), 3), dtype=np.uint8)

objects = []

for i, (box, class_id, confidence, polygon) in enumerate(
    zip(boxes, class_ids, confidences, polygons)
):
    class_name = detector.names[int(class_id)]
    color = tuple(int(v) for v in colors[i])

    # 원본 영상 크기의 binary mask 생성
    binary_mask = np.zeros(image.shape[:2], dtype=np.uint8)

    if polygon is not None and len(polygon) >= 3:
        contour = np.round(polygon).astype(np.int32)
        cv2.fillPoly(binary_mask, [contour], 1)
        cv2.fillPoly(mask_overlay, [contour], color)

    x1, y1, x2, y2 = np.round(box).astype(int)

    cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        output,
        f"{i}: {class_name} {confidence:.2f}",
        (x1, max(20, y1 - 5)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )

    objects.append({
        "id": i,
        "class_id": int(class_id),
        "class_name": class_name,
        "confidence": float(confidence),
        "box": box,
        "mask": binary_mask.astype(bool),
    })

# 마스크와 클래스 표시 합성
output = cv2.addWeighted(mask_overlay, 0.35, output, 0.65, 0)

cv2.imshow("YOLO + SAM 2.1", output)
cv2.waitKey(0)
cv2.destroyAllWindows()
